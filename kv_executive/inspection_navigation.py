"""Image-bearing guidance, terrain-constrained flight and target reports.

Static ray hits describe obstacles; associated plume walls are labeled proxies.
Fire coordinates require multi-view thermal bearings. Bounded source guidance
uses measured terrain and, when localized, a fire standoff.
"""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Vector3Stamped
from kv_executive.inspection_evidence import InspectionEvidenceMixin
from kv_executive.inspection_heading import InspectionHeadingMixin
from kv_executive.inspection_report_view import InspectionReportViewMixin, retained_wall_valid
from kv_executive.inspection_settling import InspectionSettlingMixin
from kv_executive.mission_geometry import quaternion_from_yaw, transform_pose, yaw_from_quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _stamp_ns(message):
    stamp = getattr(message, 'header', message).stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _finite_point(point):
    return all(math.isfinite(getattr(point, axis)) for axis in 'xyz')


def smoke_wall_standoff_reached(wall, standoff, tolerance=0.3, max_distance=None):
    """Require the conservative wall range to be inside the inspection band."""
    if wall is None:
        return False
    nearest = wall.get('nearest', math.nan)
    uncertainty = wall.get('uncertainty', math.nan)
    if (not all(math.isfinite(value) for value in (nearest, uncertainty, standoff, tolerance))
            or nearest <= 0.0 or uncertainty < 0.0 or standoff <= 0.0 or tolerance <= 0.0):
        return False
    distance = nearest - uncertainty
    maximum = standoff + 2.0 * tolerance if max_distance is None else max_distance
    if (isinstance(maximum, bool) or not math.isfinite(maximum) or maximum < standoff):
        return False
    return standoff - 1e-9 <= distance <= maximum + 1e-9


def _unknown_point(frame='', stamp=None):
    result = PointStamped()
    result.header.frame_id = frame
    if stamp is not None:
        result.header.stamp = deepcopy(stamp)
    result.point = Point(x=math.nan, y=math.nan, z=math.nan)
    return result


def _unknown_fix(stamp=None):
    result = NavSatFix()
    if stamp is not None:
        result.header.stamp = deepcopy(stamp)
    result.status.status = NavSatStatus.STATUS_NO_FIX
    result.latitude = result.longitude = result.altitude = math.nan
    return result


def enu_offset_to_fix(reference, east, north, up, uncertainty=0.0):
    """Approximate a short local ENU offset on the WGS84 reference ellipsoid."""
    values = (reference.latitude, reference.longitude, reference.altitude,
              east, north, up, uncertainty)
    if (not all(math.isfinite(value) for value in values)
            or not -90.0 < reference.latitude < 90.0
            or not -180.0 <= reference.longitude <= 180.0
            or reference.status.status < NavSatStatus.STATUS_FIX
            or uncertainty < 0.0 or math.hypot(east, north, up) > 1000.0):
        return None
    latitude = math.radians(reference.latitude)
    cosine = math.cos(latitude)
    if abs(cosine) < 1e-6:
        return None
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    denominator = 1.0 - eccentricity_squared * math.sin(latitude) ** 2
    prime_vertical = semi_major / math.sqrt(denominator)
    meridian = semi_major * (1.0 - eccentricity_squared) / denominator ** 1.5
    if min(prime_vertical + reference.altitude, meridian + reference.altitude) <= 0.0:
        return None
    result = deepcopy(reference)
    result.latitude += math.degrees(north / (meridian + reference.altitude))
    result.longitude += math.degrees(east / ((prime_vertical + reference.altitude) * cosine))
    result.longitude = (result.longitude + 180.0) % 360.0 - 180.0
    result.altitude += up
    if not -90.0 <= result.latitude <= 90.0:
        return None
    for index in (0, 4, 8):
        existing = result.position_covariance[index]
        result.position_covariance[index] = (
            max(0.0, existing) if math.isfinite(existing) else 0.0) + uncertainty ** 2
    result.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    return result


class InspectionNavigationMixin(
        InspectionHeadingMixin, InspectionSettlingMixin,
        InspectionEvidenceMixin, InspectionReportViewMixin):
    """Support the smoke standoff and three-viewpoint thermal inspection."""

    def _verify_reset_source_localization(self):
        """Do not associate a later verification episode with an old source."""
        from kv_executive.source_localization import ThermalBearingLocalizer
        self._thermal_bearing_localizer = ThermalBearingLocalizer()

    def _inspection_query(self, request, *, require_map=False, timeout=None,
                          report_unavailable=True, allow_timeout=False,
                          allow_transform_failure=False):
        from kv_executive.mission_executive import MissionError, MissionTimeout
        from kv_executive.verification import VerificationMapUnavailable
        self._check_running()
        try:
            response = self._call(
                self._inspection_geometry_client, request, 'Query inspection geometry',
                timeout=min(self.config['planning_timeout'], timeout)
                if timeout is not None else self.config['planning_timeout'],
            )
        except MissionTimeout as error:
            reason = f'Inspection geometry service unavailable: {error}'
            self._verify_map_unavailable_reason = reason
            if require_map and not allow_timeout:
                raise VerificationMapUnavailable(
                    reason, getattr(self, '_verify_last_geometry_stamp_ns', 0)) from error
            if report_unavailable:
                self._report(f'Verify: {reason}')
            return None
        self._verify_control_pose()
        map_ns = _stamp_ns(response.map_header)
        self._verify_last_geometry_stamp_ns = map_ns
        if not response.success:
            if allow_transform_failure and getattr(response, 'error_code', 1) == 3:
                if report_unavailable:
                    self._report(f'Verify: camera association unavailable: {response.message}')
                return None
            if getattr(response, 'error_code', 1) in (2, 3):
                raise MissionError(f'Inspection geometry failed: {response.message}')
            reason = f'Inspection map unavailable: {response.message}'
            self._verify_map_unavailable_reason = reason
            if require_map:
                raise VerificationMapUnavailable(reason, map_ns)
            if report_unavailable:
                self._report(f'Verify: {reason}')
            return None
        frame = response.map_header.frame_id
        if not frame or frame != self.config['verify_planning_frame']:
            raise MissionError('Inspection geometry returned an unexpected map frame')
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - map_ns) / 1e9
        if (now_ns <= 0 or map_ns <= 0 or not -self.config['verify_future_tolerance'] <= age
                <= self.config.get('verify_geometry_timeout', 5.0)):
            reason = ('Inspection map has an invalid or stale map timestamp: '
                      f'source_age={age:.3f} s, source_stamp_ns={map_ns}, '
                      f'ros_now_ns={now_ns}, '
                      f'limit={self.config.get("verify_geometry_timeout", 5.0):.3f} s')
            self._verify_map_unavailable_reason = reason
            if require_map:
                raise VerificationMapUnavailable(reason, map_ns)
            if report_unavailable:
                self._report(f'Verify: {reason}')
            return None
        self._verify_map_unavailable_reason = ''
        return response

    def _verify_fresh_map_stamp(self, timeout=None):
        """Poll map health without requiring terrain or authorizing motion."""
        from kv_executive.srv import QueryInspectionGeometry
        response = self._inspection_query(
            QueryInspectionGeometry.Request(), timeout=timeout, report_unavailable=False)
        return _stamp_ns(response.map_header) if response is not None else 0

    def _verify_wait_for_wall_map(self, context, guard=None):
        """Await a post-settle map update while preserving current-body clearance."""
        from kv_executive.mission_navigation import _inverse
        from kv_executive.srv import QueryInspectionGeometry
        from kv_executive.verification import VerificationStartUnavailable
        timeout = self.config.get('verify_settle_map_timeout', 15.0)
        deadline = time.monotonic() + timeout
        context['map_after_epoch'] = False
        try:
            while time.monotonic() < deadline:
                if guard:
                    guard()
                if not self._verify_wall_observation_ready():
                    return False
                _, pose = self._verify_control_pose()
                request = QueryInspectionGeometry.Request()
                request.corridor.header.frame_id = self.config['verify_planning_frame']
                request.corridor.poses = [PoseStamped(
                    header=deepcopy(request.corridor.header),
                    pose=transform_pose(pose, _inverse(self._path_frame_transform(
                        self.config['verify_planning_frame']))))]
                response = self._inspection_query(
                    request, require_map=True,
                    timeout=min(0.5, max(0.001, deadline - time.monotonic())),
                    allow_timeout=True, report_unavailable=False)
                if response is not None:
                    if not response.corridor_safe:
                        raise VerificationStartUnavailable(
                            'Current body clearance failed while waiting for wall data: '
                            + response.message)
                    stamp = _stamp_ns(response.map_header)
                    if stamp > context['settled_ns']:
                        context['map_after_epoch'] = True
                        self._report(
                            'Verify: post-settle map update received; '
                            f'settled_ns={context["settled_ns"]}, map_stamp_ns={stamp}')
                        return True
                self._stop.wait(min(0.05, max(0.0, deadline - time.monotonic())))
            self._report(
                'Verify: no post-settle map update within observation wait; '
                'wall range remains unavailable at this viewpoint')
            return False
        finally:
            context['map_wait_complete'] = True

    def _inspection_detection_valid(self, message, source):
        if message is None:
            return False
        confirmed = getattr(message, 'CONFIRMED', 3 if source == 'smoke' else 2)
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = _stamp_ns(message)
        age = (now_ns - stamp_ns) / 1e9
        return (
            message.detected and message.status == confirmed
            and (source != 'smoke' or message.background_ready)
            and now_ns > 0 and stamp_ns > 0 and bool(message.header.frame_id)
            and -self.config['verify_future_tolerance'] <= age
            <= self.config['verify_observation_timeout']
            and math.isfinite(message.yaw_correction_rad)
            and math.isfinite(message.vertical_correction_rad)
            and abs(message.yaw_correction_rad) < math.pi / 2.0
            and abs(message.vertical_correction_rad) < math.pi / 2.0
        )

    def _verify_target_geometry(self, message, source='smoke'):
        """Keep obstacle hits separate from geometrically localized fire tracks."""
        from kv_executive.mission_executive import MissionError
        from kv_executive.source_localization import ThermalBearingLocalizer
        from kv_executive.srv import QueryInspectionGeometry
        unknown = {'point': None, 'planning_point': None, 'range': math.nan,
                   'uncertainty': math.nan, 'method': 'unavailable', 'ground_z': None,
                   'ground_uncertainty': math.nan, 'map_header': None, 'source_stamp': None,
                   'source': source, 'target_valid': False, 'obstacle_point': None,
                   'obstacle_range': math.nan, 'estimate_point': None}
        if not self._inspection_detection_valid(message, source):
            return {**unknown, 'method': 'unavailable_detection'}
        request = QueryInspectionGeometry.Request()
        # These simulator camera frames use +X forward, +Y left and +Z up.
        # Native TF transforms both the zero camera origin and the ray at the
        # image acquisition time; no current drone-pose substitution is used.
        request.origin.header = deepcopy(message.header)
        request.direction.header = deepcopy(message.header)
        horizontal = message.yaw_correction_rad
        vertical = message.vertical_correction_rad
        # The detectors expose independent pinhole angles atan(y/x) and
        # atan(z/x), rather than spherical azimuth/elevation coordinates.
        left, up = math.tan(horizontal), math.tan(vertical)
        norm = math.hypot(1.0, left, up)
        request.direction.vector.x = 1.0 / norm
        request.direction.vector.y = left / norm
        request.direction.vector.z = up / norm
        request.max_range = self.config.get('verify_inspection_max_range', 40.0)
        response = self._inspection_query(request)
        if response is None:
            return {**unknown, 'method': 'unavailable_surface_range'}
        if not self._inspection_detection_valid(message, source):
            return {**unknown, 'method': 'unavailable_detection_became_stale'}
        frame = self.config['verify_planning_frame']
        conversion = self._path_frame_transform(frame)
        geometry = {**unknown, 'map_header': deepcopy(response.map_header),
                    'source_stamp': deepcopy(message.header.stamp),
                    'method': 'unavailable_semantic_source_range'}
        if response.surface_valid:
            surface = response.surface
            if (surface.header.frame_id != frame or not _finite_point(surface.point)
                    or not math.isfinite(response.surface_range) or response.surface_range <= 0
                    or response.surface_range > request.max_range
                    or not math.isfinite(response.surface_uncertainty)
                    or response.surface_uncertainty < 0):
                raise MissionError('Inspection geometry returned an invalid surface estimate')
            obstacle = transform_pose(Pose(
                position=deepcopy(surface.point), orientation=quaternion_from_yaw(0.0)),
                conversion).position
            geometry.update(
                obstacle_point=obstacle, obstacle_planning_point=deepcopy(surface),
                obstacle_range=float(response.surface_range),
                obstacle_uncertainty=float(response.surface_uncertainty),
                occupied_samples=int(response.occupied_samples),
                free_samples=int(response.free_samples),
                plume_candidate=bool(response.plume_candidate))
        origin, direction = response.ray_origin, response.ray_direction
        if (origin.header.frame_id != frame or direction.header.frame_id != frame
                or origin.header.stamp != message.header.stamp
                or direction.header.stamp != message.header.stamp
                or not _finite_point(origin.point)
                or not _finite_point(direction.vector)
                or math.hypot(*(getattr(direction.vector, axis) for axis in 'xyz')) < 1e-9):
            raise MissionError('Inspection geometry returned invalid source-stamped ray metadata')
        local_origin = transform_pose(Pose(
            position=deepcopy(origin.point), orientation=quaternion_from_yaw(0.0)),
            conversion).position
        geometry['ray_origin'] = deepcopy(local_origin)
        ray_end = Point(**{
            axis: getattr(origin.point, axis) + getattr(direction.vector, axis)
            for axis in 'xyz'})
        local_end = transform_pose(Pose(
            position=ray_end, orientation=quaternion_from_yaw(0.0)), conversion).position
        dx, dy = local_end.x - local_origin.x, local_end.y - local_origin.y
        if math.hypot(dx, dy) > 1e-6:
            geometry['navigation_bearing'] = math.atan2(dy, dx)
        # A static or flickering map surface is an obstacle observation only.
        # Smoke has no stable point feature/track for emitter triangulation.
        # A history-associated plume hit can locate the observed smoke wall,
        # explicitly as a proxy rather than the emitter or fire position.
        if source != 'fire':
            if (self._verify_smoke_boundary_reached(geometry)
                    and geometry.get('occupied_samples', 0) >= 2
                    and geometry.get('free_samples', 0) >= 1
                    and geometry.get('obstacle_uncertainty', math.inf)
                    <= self.config.get('verify_inspection_max_uncertainty', 2.0)):
                geometry.update(
                    point=deepcopy(geometry['obstacle_point']),
                    planning_point=deepcopy(geometry['obstacle_planning_point']),
                    range=geometry['obstacle_range'],
                    uncertainty=geometry['obstacle_uncertainty'],
                    target_valid=True, method='lidar_smoke_plume_proxy')
            geometry['smoke_boundary_reached'] = self._verify_smoke_boundary_reached(geometry)
            return geometry
        if not hasattr(self, '_thermal_bearing_localizer'):
            self._thermal_bearing_localizer = ThermalBearingLocalizer()
        fit = self._thermal_bearing_localizer.add(
            stamp=_stamp_ns(message) / 1e9, camera=message.header.frame_id,
            origin=tuple(getattr(origin.point, axis) for axis in 'xyz'),
            direction=tuple(getattr(direction.vector, axis) for axis in 'xyz'),
            track_id=int(getattr(message, 'track_id', 0)),
            source_count=int(getattr(message, 'source_count', 1)))
        geometry['localization_reason'] = fit['reason']
        if fit['point'] is None:
            return geometry
        point = Point(**dict(zip('xyz', fit['point'])))
        local_point = transform_pose(Pose(
            position=point, orientation=quaternion_from_yaw(0.0)), conversion).position
        valid = fit['uncertainty'] <= self.config.get('verify_inspection_max_uncertainty', 2.0)
        geometry.update(
            estimate_point=deepcopy(local_point), estimate_uncertainty=fit['uncertainty'],
            target_valid=valid, method='thermal_multiview_bearings',
            localization_views=fit['views'], localization_baseline=fit['baseline'],
            localization_parallax=fit['parallax'], localization_association=fit['association'])
        if valid and self._inspection_detection_valid(message, source):
            geometry.update(
                point=local_point, planning_point=PointStamped(
                    header=deepcopy(origin.header), point=point), range=fit['range'],
                uncertainty=fit['uncertainty'])
        else:
            geometry['target_valid'] = False
            geometry['localization_reason'] = 'unavailable_target_uncertainty'
        return geometry

    def _inspection_ground(self, target, terrain_envelope=False, *, require_map=False):
        from kv_executive.mission_executive import MissionError
        from kv_executive.mission_navigation import _inverse
        from kv_executive.srv import QueryInspectionGeometry
        request = QueryInspectionGeometry.Request()
        request.terrain_envelope = bool(terrain_envelope)
        frame = self.config['verify_planning_frame']
        conversion = self._path_frame_transform(frame)
        request.ground_query.header.frame_id = frame
        request.ground_query.header.stamp = self.get_clock().now().to_msg()
        request.ground_query.point = transform_pose(target, _inverse(conversion)).position
        response = self._inspection_query(request, require_map=require_map)
        if response is None or not response.ground_valid:
            return None
        surface = response.ground_surface
        if (surface.header.frame_id != self.config['verify_planning_frame']
                or not _finite_point(surface.point)
                or not math.isfinite(response.ground_uncertainty)
                or response.ground_uncertainty < 0.0):
            raise MissionError('Inspection geometry returned an invalid support surface')
        conversion = self._path_frame_transform(surface.header.frame_id)
        pose = Pose(position=deepcopy(surface.point), orientation=quaternion_from_yaw(0.0))
        return transform_pose(pose, conversion).position.z, response.ground_uncertainty

    def _inspection_constraints(
        self, request, map_from_local, point=None, minimum_radius=0.0,
        minimum_agl=0.0, allow_initial_climb=False, terrain_envelope=False,
        allow_terrain_escape=False,
    ):
        """Represent local-horizontal constraints explicitly in the map frame."""
        frame = self.config['verify_planning_frame']
        zero = transform_pose(Pose(orientation=quaternion_from_yaw(0.0)), map_from_local)
        up = transform_pose(Pose(position=Point(z=1.0),
                                 orientation=quaternion_from_yaw(0.0)), map_from_local)
        request.keepout_axis = Vector3Stamped()
        request.keepout_axis.header.frame_id = frame
        for axis in 'xyz':
            setattr(request.keepout_axis.vector, axis,
                    getattr(up.position, axis) - getattr(zero.position, axis))
        if point is not None:
            request.keepout_center.header.frame_id = frame
            request.keepout_center.point = transform_pose(Pose(
                position=deepcopy(point), orientation=quaternion_from_yaw(0.0)),
                map_from_local).position
        request.minimum_radius = float(minimum_radius)
        request.minimum_agl = float(minimum_agl)
        request.allow_initial_climb = bool(allow_initial_climb)
        request.terrain_envelope = bool(terrain_envelope)
        request.allow_terrain_escape = bool(allow_terrain_escape)
        return request

    def _inspection_request_builder(
        self, point=None, minimum_radius=0.0, minimum_agl=0.0, allow_initial_climb=False,
        terrain_envelope=False, allow_terrain_escape=False,
    ):
        def build(target, map_from_local):
            from kv_executive.srv import PlanInspectionPath
            request = PlanInspectionPath.Request()
            request.target = target
            return self._inspection_constraints(
                request, map_from_local, point, minimum_radius, minimum_agl,
                allow_initial_climb, terrain_envelope, allow_terrain_escape)
        return build

    def _verify_inspection_pose(self, smoke_geometry, phase='standoff', origin=None):
        """Choose a terrain-relative viewpoint without approaching a fire coordinate."""
        from kv_executive.mission_executive import MissionError
        if phase not in {'standoff', 'ground', 'left', 'right'}:
            raise MissionError(f'Unknown inspection phase: {phase}')
        _, current = self._verify_control_pose()
        target = deepcopy(origin if origin is not None else current)
        bearing = yaw_from_quaternion(target.orientation)
        if smoke_geometry and math.isfinite(smoke_geometry.get('navigation_bearing', math.nan)):
            bearing = smoke_geometry['navigation_bearing']
        if phase in {'left', 'right'}:
            sign = 1.0 if phase == 'left' else -1.0
            offset = self.config.get('verify_inspection_lateral_step', 5.0)
            target.position.x -= sign * offset * math.sin(bearing)
            target.position.y += sign * offset * math.cos(bearing)
        probe = deepcopy(target)
        probe.position.z = max(current.position.z, target.position.z) + self.config.get(
            'verify_navigation_max_agl', 10.0)
        terrain = self._inspection_ground(probe, terrain_envelope=True)
        if terrain is None:
            target.position.z = max(current.position.z, target.position.z)
            self._report('Verify: ground height unavailable; retaining inspection altitude')
        else:
            floor = (terrain[0] + self.config.get('verify_inspection_altitude', 3.0)
                     + terrain[1] + self.config.get('position_tolerance', 0.3))
            target.position.z = (max(self.config['verify_minimum_altitude'], floor)
                                 if phase == 'ground' else max(current.position.z, floor))
            if smoke_geometry is not None:
                smoke_geometry['ground_z'] = terrain[0]
                smoke_geometry['ground_uncertainty'] = terrain[1]
                smoke_geometry['ground_method'] = 'conservative_octomap_terrain_envelope'
        target.orientation = quaternion_from_yaw(bearing)
        return target

    def _inspection_map_corridor(self, path, minimum_agl=0.0, point=None,
                                 allow_initial_climb=False, terrain_envelope=False,
                                 allow_terrain_escape=False, escape_state=None,
                                 allow_completed_cluster_escape=False):
        """Check map collision/terrain clearance after explicit local alignment."""
        from kv_executive.mission_executive import MissionError
        from kv_executive.mission_navigation import _inverse
        from kv_executive.srv import QueryInspectionGeometry
        local_frame = self.config['local_frame']
        if (path.header.frame_id != local_frame
                or any(vertex.header.frame_id not in ('', local_frame) for vertex in path.poses)):
            raise MissionError('Inspection corridor must use the configured local frame')
        if not path.poses or any(not _finite_point(vertex.pose.position) for vertex in path.poses):
            self._report('Verify: inspection corridor has no finite path to validate')
            return False
        request = QueryInspectionGeometry.Request()
        request.allow_completed_cluster_escape = allow_completed_cluster_escape
        # MAVROS local ENU is aligned to the planner map explicitly; no
        # map->px4_odom TF edge is assumed anywhere in this executive.
        frame = self.config['verify_planning_frame']
        conversion = _inverse(self._path_frame_transform(frame))
        request.corridor = Path(header=deepcopy(path.header))
        request.corridor.header.frame_id = frame
        for vertex in path.poses:
            stamped = PoseStamped(header=deepcopy(vertex.header))
            stamped.header.frame_id = frame
            stamped.pose = transform_pose(vertex.pose, conversion)
            request.corridor.poses.append(stamped)
        # Horizontal radius is enforced analytically in PX4 local coordinates
        # before this query. A pitched/rolled map alignment does not preserve
        # horizontal distance, so repeating the scalar constraint in map XY
        # would reject valid local paths. Map collision and AGL checks remain.
        self._inspection_constraints(
            request, conversion, point, 0.0, minimum_agl, allow_initial_climb,
            terrain_envelope, allow_terrain_escape)
        if escape_state is not None:
            origin = escape_state.get('origin')
            if origin is None or not _finite_point(origin):
                raise MissionError('Terrain escape revalidation requires a finite fixed origin')
            request.escape_origin.header.frame_id = frame
            request.escape_origin.point = transform_pose(Pose(
                position=deepcopy(origin), orientation=quaternion_from_yaw(0.0)),
                conversion).position
            request.terrain_escape_complete = bool(escape_state.get('complete', False))
        request.initial_climb_tracking_tolerance = 0.1 if allow_initial_climb else 0.0
        response = self._inspection_query(request, require_map=True)
        if response is None:
            self._report(
                'Verify: inspection corridor validation unavailable: '
                'map geometry is unavailable or stale')
            return False
        if escape_state is not None and getattr(response, 'current_clearance_reached', False):
            escape_state['complete'] = True
        if not response.corridor_safe:
            self._report(f'Verify: inspection corridor rejected: {response.message}')
            return False
        return True

    def _inspection_local_standoff(self, path, point, minimum_radius):
        """Check exact closest XY distance on every local path segment."""
        from kv_executive.mission_executive import MissionError
        if not math.isfinite(minimum_radius) or minimum_radius < 0.0:
            raise MissionError('Inspection standoff radius must be finite and nonnegative')
        if not path.poses or any(not _finite_point(vertex.pose.position) for vertex in path.poses):
            self._report('Verify: inspection corridor has no finite path to validate')
            return False
        for index, vertex in enumerate(path.poses):
            end = vertex.pose.position
            start = path.poses[index - 1].pose.position if index else end
            ax, ay = start.x - point.x, start.y - point.y
            vx, vy = end.x - start.x, end.y - start.y
            length_squared = vx * vx + vy * vy
            if not math.isfinite(length_squared):
                self._report('Verify: inspection corridor exceeds finite local geometry')
                return False
            fraction = (max(0.0, min(1.0, -(ax * vx + ay * vy) / length_squared))
                        if length_squared > 0.0 else 0.0)
            closest = math.hypot(ax + fraction * vx, ay + fraction * vy)
            if not math.isfinite(closest) or closest < minimum_radius - 1e-6:
                self._report(
                    'Verify: inspection corridor violates local standoff radius: '
                    f'closest={closest:.6f} m, required={minimum_radius:.6f} m')
                return False
        return True

    def _verify_inspection_plan_to(
        self, pose, smoke_geometry=None, guard=None, minimum_radius=None, minimum_agl=None,
        navigation_only=False, terrain_envelope=True,
    ):
        """Compatibility entry point; terrain, never fire range, constrains flight."""
        return self._verify_recovery_plan_to(pose, smoke_geometry, guard)

    def _verify_current_terrain_hazard(self, current):
        """Recognize below-floor clearance from fresh terrain below the actual body."""
        cached = getattr(self, '_verify_current_terrain', None)
        now_ns = self.get_clock().now().nanoseconds
        terrain = None
        if cached is not None:
            age = (now_ns - cached['stamp_ns']) / 1e9
            moved = math.dist(tuple(getattr(current.position, a) for a in 'xyz'),
                              tuple(getattr(cached['position'], a) for a in 'xyz'))
            if 0.0 <= age <= self.config.get('verify_geometry_timeout', 5.0) and moved <= 0.05:
                terrain = cached['terrain']
            else:
                cached = None
        if cached is None:
            terrain = self._inspection_ground(
                deepcopy(current), terrain_envelope=True, require_map=True)
        if terrain is None:
            return ''
        clearance = current.position.z - terrain[0]
        if clearance < self.config.get('verify_inspection_altitude', 3.0) - 1e-6:
            return f'current terrain clearance {clearance:.2f} m is below the required floor'
        return ''

    def _verify_path_motion_policy(
            self, path, origin, bearing, state, recovery_reason='', description='planned path'):
        """Bound the whole route, including intermediate backtracking and altitude peaks."""
        points = [vertex.pose.position for vertex in path.poses]
        if not points:
            return False
        if not all(_finite_point(point) for point in points):
            return False
        projections = [((point.x - origin.x) * math.cos(bearing)
                        + (point.y - origin.y) * math.sin(bearing)) for point in points]
        actual_peak = max(state.get('actual_peak', 0.0), projections[0])
        backward = actual_peak - projections[0]
        peak = projections[0]
        for projection in projections:
            peak = max(peak, projection)
            backward = max(backward, peak - projection, -projection)
        rise = max(0.0, *(point.z - origin.z for point in points))
        drop = max(0.0, *(origin.z - point.z for point in points))
        length = sum(math.dist(tuple(getattr(a, c) for c in 'xyz'),
                               tuple(getattr(b, c) for c in 'xyz'))
                     for a, b in zip(points, points[1:]))
        prefix = 'verify_recovery_' if recovery_reason else 'verify_approach_'
        limits = (
            self.config.get(prefix + 'max_rise', 6.0 if recovery_reason else 1.0),
            self.config.get(prefix + 'max_drop', 1.0),
            self.config.get(prefix + 'max_backtrack', 3.3 if recovery_reason else 0.3),
            self.config.get('verify_approach_max_path_length', 12.0))
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError('Path motion limits must be finite and positive')
        accepted = all(value <= limit + 1e-6
                       for value, limit in zip((rise, drop, backward, length), limits))
        signature = (accepted, recovery_reason)
        now = time.monotonic()
        if (state.get('reported') != signature
                or now - state.get('reported_at', -math.inf) >= 5.0):
            self._report(
                f'Verify: {description} {"accepted" if accepted else "rejected"}; '
                f'backward={backward:.2f} m, rise={rise:.2f} m, drop={drop:.2f} m, '
                f'length={length:.2f} m; reason={recovery_reason or "ordinary approach"}')
            state['reported'] = signature
            state['reported_at'] = now
        if accepted:
            state['actual_peak'] = actual_peak
        return accepted

    def _verify_recovery_plan_to(
        self, target, geometry=None, guard=None, terrain=None, preserve_requested_altitude=False,
    ):
        """Fly a collision-checked terrain leg, retaining any localized fire standoff."""
        from kv_executive.mission_executive import MissionError
        # Only a known terrain floor veto is an optional height preference.
        # Other refusals and actual execution failures retain their own handling.
        self._verify_height_rejection = ''
        if target is None:
            return False
        target = deepcopy(target)
        minimum_agl = self.config.get('verify_inspection_altitude', 3.0)
        _, current = self._verify_control_pose()
        recovery_reason = ''
        if ((geometry or {}).get('confirmed_wall_retreat', False)
                and (geometry or {}).get('safety_recovery_reason')
                == 'confirmed_smoke_wall_inside_standoff'):
            for wall in self._verify_smoke_navigation_walls(geometry):
                distance = wall['nearest'] - wall['uncertainty'] - sum(
                    (getattr(current.position, a) - getattr(wall['origin'], a)) * wall['axis'][i]
                    for i, a in enumerate('xyz'))
                if distance < self.config.get('verify_smoke_standoff', 15.0):
                    recovery_reason = 'confirmed smoke wall is inside the minimum standoff'
                    break
        if not recovery_reason:
            recovery_reason = self._verify_current_terrain_hazard(current)
        policy_origin = deepcopy(current.position)
        policy_bearing = (geometry or {}).get('navigation_bearing',
                                              yaw_from_quaternion(target.orientation))
        if not math.isfinite(policy_bearing):
            raise MissionError('Motion policy requires a finite approach bearing')
        policy_state = {}
        requested_height = target.position.z - current.position.z
        minimum_height = self.config.get('verify_vertical_minimum_change', 0.1)
        height_tolerance = self.config.get('verify_vertical_position_tolerance', 0.05)
        if (not all(math.isfinite(value) and value > 0.0 for value in (
                minimum_height, height_tolerance)) or height_tolerance >= minimum_height):
            raise MissionError('Invalid useful height movement or vertical position tolerance')
        if math.hypot(target.position.x - current.position.x,
                      target.position.y - current.position.y) > 10.0 + 1e-6:
            self._report('Verify: recovery destination exceeds the bounded observation leg')
            return False
        probe = deepcopy(target)
        probe.position.z = max(current.position.z, target.position.z) + self.config.get(
            'verify_navigation_max_agl', 10.0)
        if terrain is None:
            terrain = self._inspection_ground(probe, terrain_envelope=True, require_map=True)
        requested_descent = requested_height < -1e-9
        floor = self.config['verify_minimum_altitude']
        if terrain is None:
            if preserve_requested_altitude and requested_descent:
                self._verify_height_rejection = 'terrain_unavailable'
                self._report('Verify: ground height unavailable; downward motion withheld')
                return False
            floor = max(floor, current.position.z)
            self._report(
                'Verify: ground height unavailable; retaining altitude and checking '
                'the full path against mapped obstacles and terrain clearance')
        else:
            floor = max(floor, terrain[0] + minimum_agl + terrain[1]
                        + self.config.get('position_tolerance', 0.3))
        target.position.z = max(target.position.z, floor)
        if preserve_requested_altitude and abs(requested_height) > 1e-9:
            available_height = target.position.z - current.position.z
            signed_height = available_height * math.copysign(1.0, requested_height)
            self._report(
                f'Verify: height request={requested_height:+.2f} m, '
                f'terrain-adjusted change={available_height:+.2f} m, '
                f'current altitude={current.position.z:.2f} m, '
                f'target altitude={target.position.z:.2f} m, terrain floor={floor:.2f} m')
            if (signed_height < minimum_height - 1e-9
                    or signed_height > abs(requested_height) + 1e-9):
                self._verify_height_rejection = (
                    'terrain_floor' if requested_descent else 'height_limit')
                self._report(
                    'Verify: terrain clearance floor prevents downward image centering; '
                    'remaining descent is not useful'
                    if requested_descent else
                    'Verify: terrain clearance floor prevents the bounded upward correction')
                return False
        if not preserve_requested_altitude:
            target.position.z = max(target.position.z, current.position.z)
        endpoint = Path()
        endpoint.poses = [PoseStamped(pose=deepcopy(current)), PoseStamped(pose=deepcopy(target))]
        if not self._verify_path_motion_policy(
                endpoint, policy_origin, policy_bearing, {}, recovery_reason,
                description='proposed destination'):
            return False
        escape_state = {'origin': deepcopy(current.position), 'complete': False}
        keepout_radius = (geometry or {}).get('fire_keepout_radius', 0.0)
        keepout_point = None
        if keepout_radius:
            keepout_point = geometry.get('point')
            if (geometry.get('source') != 'fire' or not geometry.get('target_valid', False)
                    or keepout_point is None or not _finite_point(keepout_point)
                    or not math.isfinite(keepout_radius) or keepout_radius <= 0.0):
                raise MissionError('Fire standoff requires a valid localized source and radius')

        def validate(path):
            if not self._verify_path_motion_policy(
                    path, policy_origin, policy_bearing, policy_state, recovery_reason):
                return False
            if geometry and geometry.get('center_fire_wall') is not None:
                if not self._verify_center_fire_wall_path(path, geometry):
                    return False
            for measured in self._verify_smoke_navigation_walls(geometry):
                if not self._verify_wall_plane_path(path, measured):
                    return False
            if terrain is None and any(
                    vertex.pose.position.z < floor - self.config.get('position_tolerance', 0.3)
                    for vertex in path.poses):
                self._report('Verify: ground height unavailable; descending path withheld')
                return False
            if keepout_point is not None and not self._inspection_local_standoff(
                    path, keepout_point, keepout_radius):
                self._report('Verify: fire approach path violates localized fire standoff')
                return False
            return self._inspection_map_corridor(
                path, point=keepout_point, minimum_agl=minimum_agl, allow_initial_climb=True,
                terrain_envelope=True, allow_terrain_escape=True, escape_state=escape_state)

        fallback = yaw_from_quaternion(target.orientation)
        center_fire = bool(geometry and geometry.get('center_fire_wall') is not None)
        heading = ((lambda _pose: fallback) if center_fire
                   else self._verify_live_source_heading(fallback))
        if math.dist(tuple(getattr(current.position, axis) for axis in 'xyz'),
                     tuple(getattr(target.position, axis) for axis in 'xyz')) < 1e-6:
            corridor = Path()
            corridor.header.frame_id = self.config['local_frame']
            corridor.poses = [PoseStamped(
                header=deepcopy(corridor.header), pose=deepcopy(current))]
            if not validate(corridor):
                return False
            self._verify_rotate(heading(current), guard=guard)
            return True
        return self._verify_plan_to(
            target, guard=guard, speed=self.config['verify_motion_speed'],
            allow_plan_failure=True, path_validator=validate, heading_provider=heading,
            planning_client=self._inspection_planning_client,
            request_builder=self._inspection_request_builder(
                point=keepout_point, minimum_radius=keepout_radius,
                minimum_agl=minimum_agl, allow_initial_climb=True,
                terrain_envelope=True, allow_terrain_escape=True),
            **({'height_tolerance': height_tolerance}
               if preserve_requested_altitude and abs(requested_height) > 1e-9 else {}),
            **({'maximum_path_length': geometry['center_fire_max_path_length'],
                'motion_timeout': geometry['center_fire_motion_timeout']} if center_fire else {}),
        )

    def _verify_center_fire_wall(self, camera_frame):
        """Measure a smoke-wall aperture; never estimate fire range."""
        return self._verify_measure_wall(camera_frame)

    def _verify_opportunistic_wall(self, camera_frame, bearing):
        """Probe once along the next leg; absent returns never imply a map outage."""
        timeout = self.config.get('verify_wall_probe_timeout', 0.5)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('Wall probe timeout must be finite and positive')
        wall = self._verify_measure_wall(camera_frame, bearing=bearing, timeout=timeout)
        if wall is None:
            self._report('Verify: opportunistic smoke-wall range unavailable; '
                         'retaining existing navigation limits')
        else:
            self._report(
                f'Verify: opportunistic smoke wall mean={wall["mean"]:.2f} m, '
                f'nearest={wall["nearest"]:.2f} m, '
                f'uncertainty={wall["uncertainty"]:.2f} m')
        return wall

    def _verify_measure_wall(self, camera_frame, *, bearing=None, timeout=None):
        """Measure one aperture, optionally oriented along a local navigation bearing."""
        from kv_executive.srv import QueryInspectionGeometry

        def unavailable(reason):
            now = time.monotonic()
            if (not getattr(self, '_verify_wall_rejection', '')
                    or now - getattr(self, '_verify_wall_rejection_at', -math.inf) >= 5.0):
                self._report(f'Verify: smoke-wall measurement rejected: {reason}')
                self._verify_wall_rejection = reason
                self._verify_wall_rejection_at = now
            return None

        def bounds(message):
            if not self._inspection_detection_valid(message, 'smoke'):
                return None
            values = tuple(getattr(message, name, math.nan) for name in (
                'yaw_min_rad', 'yaw_max_rad', 'vertical_min_rad', 'vertical_max_rad'))
            if (not all(math.isfinite(value) and abs(value) < math.pi / 2 for value in values)
                    or not values[0] < values[1] or not values[2] < values[3]
                    or not values[0] <= message.yaw_correction_rad <= values[1]
                    or not values[2] <= message.vertical_correction_rad <= values[3]):
                return None
            return values

        # Flight validators call this helper too. They may retain an earlier
        # plane, but must never block motion waiting for a settled observation.
        if not self._verify_wall_observation_ready():
            return unavailable('vehicle has no current settled observation viewpoint')
        context = self._verify_wall_observation_context
        settled_ns = context['settled_ns']

        smoke = self._verify.fresh_smoke()
        roi_source, roi_bounds = None, None
        if smoke is not None and getattr(smoke, 'angular_bounds_valid', False):
            roi_bounds = bounds(smoke)
            if roi_bounds is None:
                return unavailable('smoke-camera bounds are stale, invalid or inconsistent')
            roi_source = smoke
            # A thermal observation may guide flight, but wall identity is
            # associated with the confirmed RGB smoke region in its own frame.
            camera_frame = smoke.header.frame_id

        image = roi_source
        image_source = 'smoke'
        if image is None:
            for candidate, source in ((smoke, 'smoke'), (self._verify.stable_fire(), 'fire')):
                if (self._inspection_detection_valid(candidate, source)
                        and candidate.header.frame_id == camera_frame):
                    image = candidate
                    image_source = source
                    break
        if image is None or _stamp_ns(image.header) <= settled_ns:
            return unavailable('a new camera observation after settling is required')

        request = QueryInspectionGeometry.Request()
        request.origin.header.frame_id = request.direction.header.frame_id = camera_frame
        request.smoke_wall_reject_terrain = True
        request.smoke_wall_extended_evidence = True
        # Latest camera TF: this is a current map measurement, not an old image ray.
        request.direction.vector.x = 1.0
        conversion = None
        if roi_source is not None:
            request.smoke_wall_camera_roi = True
            request.smoke_camera_header = deepcopy(roi_source.header)
            (request.smoke_yaw_min_rad, request.smoke_yaw_max_rad,
             request.smoke_vertical_min_rad, request.smoke_vertical_max_rad) = roi_bounds
            request.direction.vector.y = math.tan(roi_source.yaw_correction_rad)
            request.direction.vector.z = math.tan(roi_source.vertical_correction_rad)
        elif bearing is not None:
            from kv_executive.mission_navigation import _inverse
            if not camera_frame or not math.isfinite(bearing):
                raise ValueError('Wall probe requires a camera frame and finite bearing')
            # PX4 local odometry is aligned through the observed body pose, not
            # assumed to be a TF frame. Rotate the local bearing into the map.
            conversion = self._path_frame_transform(self.config['verify_planning_frame'])
            inverse = _inverse(conversion)
            zero = transform_pose(Pose(orientation=quaternion_from_yaw(0.0)), inverse).position
            endpoint = transform_pose(Pose(
                position=Point(x=math.cos(bearing), y=math.sin(bearing)),
                orientation=quaternion_from_yaw(0.0)), inverse).position
            request.direction.header.frame_id = self.config['verify_planning_frame']
            for axis in 'xyz':
                setattr(request.direction.vector, axis,
                        getattr(endpoint, axis) - getattr(zero, axis))
        request.max_range = self.config.get('verify_inspection_max_range', 40.0)
        request.smoke_wall_width = 4.5
        response = self._inspection_query(
            request, require_map=True, timeout=timeout, allow_timeout=timeout is not None,
            report_unavailable=timeout is None, allow_transform_failure=roi_source is not None)
        if response is None:
            return unavailable('geometry query or camera association returned no measurement')
        if (_stamp_ns(response.map_header) <= settled_ns
                or not self._verify_wall_observation_ready()
                or self._verify_wall_observation_context is not context):
            return unavailable('map predates settling or the observation viewpoint moved')
        if not response.smoke_wall_valid:
            return unavailable(
                f'aperture not qualified ({response.smoke_wall_samples} rays); '
                + getattr(response, 'message', 'native qualification details unavailable'))
        evidence = getattr(response, 'smoke_wall_evidence', 0)
        samples, mixed, rows, columns = tuple(getattr(response, name, None) for name in (
            'smoke_wall_samples', 'smoke_wall_mixed_samples',
            'smoke_wall_rows', 'smoke_wall_columns'))
        details = (f'evidence={evidence}, support={samples}/49, mixed={mixed}, '
                   f'rows={rows}/7, columns={columns}/7')
        if (any(type(value) is not int for value in (evidence, samples, mixed, rows, columns))
                or not 0 <= mixed <= samples <= 49
                or not 1 <= rows <= min(7, samples)
                or not 1 <= columns <= min(7, samples)
                or samples > rows * columns):
            return unavailable(f'malformed wall evidence counts: {details}')
        if rows < 3 or columns < 3:
            return unavailable(f'wall support requires at least 3 rows and 3 columns: {details}')
        if evidence == QueryInspectionGeometry.Response.WALL_EVIDENCE_FLICKER:
            if samples < 3 or mixed < 1:
                return unavailable(
                    f'flicker evidence needs 3 support rays and 1 mixed ray: {details}')
            evidence_name = 'flicker'
        elif evidence == QueryInspectionGeometry.Response.WALL_EVIDENCE_PERSISTENT:
            if samples < 6 or roi_source is None:
                return unavailable(
                    f'persistent evidence needs 6 support rays and confirmed smoke-camera bounds: '
                    f'{details}')
            evidence_name = 'persistent'
        else:
            return unavailable(f'unknown wall evidence route: {details}')
        mean, nearest, uncertainty = (
            response.smoke_wall_range, response.smoke_wall_nearest_range,
            response.smoke_wall_uncertainty)
        if (not all(math.isfinite(v) for v in (mean, nearest, uncertainty))
                or not 0.0 < nearest <= mean <= request.max_range):
            return unavailable(f'invalid range statistics: mean={mean}, nearest={nearest}')
        if not 0.0 <= uncertainty <= self.config['verify_inspection_max_uncertainty']:
            return unavailable(
                f'uncertainty={uncertainty:.2f} m exceeds the permitted '
                f'{self.config["verify_inspection_max_uncertainty"]:.2f} m')
        if conversion is None:
            conversion = self._path_frame_transform(self.config['verify_planning_frame'])
        origin = response.ray_origin.point
        direction = response.ray_direction.vector
        if (response.ray_origin.header.frame_id != self.config['verify_planning_frame']
                or response.ray_direction.header.frame_id != self.config['verify_planning_frame']
                or not _finite_point(origin) or not _finite_point(direction)):
            return unavailable('invalid map-frame ray origin or direction')
        nearest_point_valid = bool(getattr(response, 'smoke_wall_nearest_point_valid', False))
        nearest_point = getattr(response, 'smoke_wall_nearest_point', None)
        if nearest_point_valid and (
                nearest_point is None or not _finite_point(nearest_point.point)
                or nearest_point.header.frame_id != self.config['verify_planning_frame']
                or nearest_point.header.stamp != response.map_header.stamp):
            return unavailable('invalid nearest smoke-wall point or measurement timestamp')
        local = transform_pose(Pose(position=deepcopy(origin),
                                    orientation=quaternion_from_yaw(0.0)), conversion).position
        end = transform_pose(Pose(position=Point(**{
            a: getattr(origin, a) + getattr(direction, a) for a in 'xyz'}),
            orientation=quaternion_from_yaw(0.0)), conversion).position
        axis = tuple(getattr(end, a) - getattr(local, a) for a in 'xyz')
        norm = math.hypot(*axis)
        if norm < 1e-6:
            return unavailable('transformed ray direction has no usable length')
        # TF lookup can wait after the query's initial map freshness check.
        age = (self.get_clock().now().nanoseconds - _stamp_ns(response.map_header)) / 1e9
        if not -self.config['verify_future_tolerance'] <= age <= self.config.get(
                'verify_geometry_timeout', 5.0):
            return unavailable(f'map aged during transform: {age:.2f} source seconds')
        if roi_source is not None:
            current = self._verify.fresh_smoke()
            current_bounds = bounds(current) if current is not None and getattr(
                current, 'angular_bounds_valid', False) else None
            if (not self._inspection_detection_valid(roi_source, 'smoke')
                    or current_bounds is None or current.header.frame_id != camera_frame
                    or max(roi_bounds[0], current_bounds[0]) > min(
                        roi_bounds[1], current_bounds[1])
                    or max(roi_bounds[2], current_bounds[2]) > min(
                        roi_bounds[3], current_bounds[3])):
                return unavailable('confirmed smoke-camera association changed during wall query')
        current_image = (self._verify.fresh_smoke() if image_source == 'smoke'
                         else self._verify.stable_fire())
        if (not self._inspection_detection_valid(current_image, image_source)
                or current_image.header.frame_id != image.header.frame_id
                or _stamp_ns(current_image.header) <= settled_ns
                or not self._verify_wall_observation_ready()
                or self._verify_wall_observation_context is not context):
            return unavailable('camera observation or settled viewpoint changed during query')
        now = time.monotonic()
        signature = (evidence, roi_source is not None)
        if (getattr(self, '_verify_wall_evidence_reported', None) != signature
                or now - getattr(self, '_verify_wall_evidence_reported_at', -math.inf) >= 5.0):
            self._report(f'Verify: smoke-wall evidence={evidence_name}, support={samples}/49, '
                         f'mixed={mixed}, rows={rows}/7, columns={columns}/7; '
                         f'camera associated={roi_source is not None}')
            self._verify_wall_evidence_reported = signature
            self._verify_wall_evidence_reported_at = now
        self._verify_wall_rejection = ''
        return {'origin': local, 'axis': tuple(v / norm for v in axis), 'mean': mean,
                'nearest': nearest, 'uncertainty': uncertainty, 'camera_frame': camera_frame,
                'bearing': bearing, 'map_header': deepcopy(response.map_header),
                'map_stamp_ns': _stamp_ns(response.map_header),
                'evidence': evidence_name, 'evidence_code': evidence, 'samples': samples,
                'mixed_samples': mixed, 'rows': rows, 'columns': columns,
                'camera_associated': roi_source is not None,
                'smoke_camera_stamp_ns': _stamp_ns(roi_source.header) if roi_source else 0,
                'nearest_point_valid': nearest_point_valid,
                'nearest_point': deepcopy(nearest_point) if nearest_point_valid else None,
                'map_origin': PointStamped(header=deepcopy(response.map_header),
                                           point=deepcopy(origin))}

    @staticmethod
    def _verify_valid_wall(wall):
        """Validate a measured plane before retaining it as a motion constraint."""
        if not isinstance(wall, dict):
            return False
        origin, axis = wall.get('origin'), wall.get('axis', ())
        if origin is None or not _finite_point(origin) or len(axis) != 3:
            return False
        values = (*axis, wall.get('nearest', math.nan), wall.get('uncertainty', math.nan))
        return (all(math.isfinite(value) for value in values)
                and abs(math.hypot(*axis) - 1.0) < 1e-6
                and wall['nearest'] > 0.0 and wall['uncertainty'] >= 0.0)

    def _verify_smoke_wall_at_standoff(self, wall):
        """Require fresh measured proximity at the current body position."""
        if not self._verify_valid_wall(wall):
            return False
        map_header = wall.get('map_header')
        if map_header is None or map_header.frame_id != self.config['verify_planning_frame']:
            return False
        stamp = _stamp_ns(map_header)
        now = self.get_clock().now().nanoseconds
        age = (now - stamp) / 1e9
        if (now <= 0 or stamp <= 0
                or not -self.config['verify_future_tolerance'] <= age
                <= self.config.get('verify_geometry_timeout', 5.0)):
            return False
        if wall['uncertainty'] > self.config['verify_inspection_max_uncertainty']:
            return False
        _, current = self._verify_control_pose()
        advance = sum((getattr(current.position, axis) - getattr(wall['origin'], axis))
                      * wall['axis'][index] for index, axis in enumerate('xyz'))
        return smoke_wall_standoff_reached(
            {**wall, 'nearest': wall['nearest'] - advance},
            self.config['verify_smoke_standoff'], self.config.get('position_tolerance', 0.3),
            max_distance=self.config.get('verify_smoke_inspection_max_distance'))

    def _verify_center_fire_wall_path(self, path, geometry):
        """Protect the observed wall plane through a short maneuver and map flicker."""
        wall = geometry['center_fire_wall']
        live = self._verify_center_fire_wall(wall['camera_frame'])
        # The original observed plane remains a bound if the live wall flickers
        # away. A fresh closer wall can tighten that bound, never loosen it.
        for measured in (wall, live):
            if measured is None:
                continue
            if not self._verify_wall_plane_path(path, measured):
                return False
        return True

    def _verify_wall_plane_path(self, path, wall):
        """Keep every path vertex behind the same measured smoke-wall plane."""
        standoff = self.config.get('verify_smoke_standoff', 15.0)
        allowed = max(0.0, wall['nearest'] - wall['uncertainty'] - standoff
                      - self.config.get('position_tolerance', 0.3))
        _, current = self._verify_control_pose()
        previous = sum((getattr(current.position, axis) - getattr(wall['origin'], axis))
                       * wall['axis'][i] for i, axis in enumerate('xyz'))
        for vertex in path.poses:
            advance = sum((getattr(vertex.pose.position, axis) - getattr(wall['origin'], axis))
                          * wall['axis'][i] for i, axis in enumerate('xyz'))
            # A newly observed closer wall or tracking error may put the actual
            # path start inside the limit. Permit retreat, never deeper entry.
            limit = max(allowed, previous)
            if advance > limit + 1e-6:
                self._report(
                    f'Verify: path would cross the {standoff:g} m smoke-wall boundary')
                return False
            previous = advance
        return True

    def _verify_smoke_navigation_wall(self, geometry):
        """Use only a fresh history-associated plume hit to bound a smoke leg."""
        committed = (geometry or {}).get('smoke_navigation_wall')
        if committed is not None:
            if not self._verify_valid_wall(committed):
                raise ValueError('Invalid committed smoke-wall plane')
            # An accepted plane remains a limit through image/map flicker.
            # It cannot independently authorize a completed inspection report.
            return deepcopy(committed)
        if (geometry or {}).get('wall_probe_only', False):
            return None
        if (not geometry or geometry.get('source') != 'smoke'
                or not geometry.get('plume_candidate', False)
                or geometry.get('occupied_samples', 0) < 2
                or geometry.get('free_samples', 0) < 1):
            return None
        origin, point = geometry.get('ray_origin'), geometry.get('obstacle_point')
        distance = geometry.get('obstacle_range', math.nan)
        uncertainty = geometry.get('obstacle_uncertainty', math.nan)
        stamp, map_header = geometry.get('source_stamp'), geometry.get('map_header')
        if (origin is None or point is None or not _finite_point(origin)
                or not _finite_point(point) or stamp is None or map_header is None
                or map_header.frame_id != self.config['verify_planning_frame']
                or not all(math.isfinite(v) for v in (distance, uncertainty))
                or not 0.0 < distance <= self.config.get('verify_inspection_max_range', 40.0)
                or not 0.0 <= uncertainty <= self.config.get(
                    'verify_inspection_max_uncertainty', 2.0)):
            return None
        now_ns = self.get_clock().now().nanoseconds
        for source_stamp, timeout in (
                (stamp, self.config['verify_observation_timeout']),
                (map_header.stamp, self.config.get('verify_geometry_timeout', 5.0))):
            stamp_ns = source_stamp.sec * 1_000_000_000 + source_stamp.nanosec
            age = (now_ns - stamp_ns) / 1e9
            if stamp_ns <= 0 or not -self.config['verify_future_tolerance'] <= age <= timeout:
                return None
        axis = tuple(getattr(point, a) - getattr(origin, a) for a in 'xyz')
        norm = math.hypot(*axis)
        if norm < 1e-6:
            return None
        return {'origin': deepcopy(origin), 'axis': tuple(v / norm for v in axis),
                'nearest': distance, 'uncertainty': uncertainty}

    def _verify_smoke_navigation_walls(self, geometry):
        """Keep all accepted planes when later measurements change direction."""
        walls = list((geometry or {}).get('smoke_navigation_walls', ()))
        measured = self._verify_smoke_navigation_wall(geometry)
        if measured is not None and measured not in walls:
            walls.append(measured)
        if (geometry or {}).get('smoke_navigation_wall') is not None:
            # A retained aperture plane must not hide a newer, closer plume-ray
            # hit. Both observations constrain this leg, even if a probe missed.
            observed = {key: value for key, value in geometry.items()
                        if key not in ('smoke_navigation_wall', 'smoke_navigation_walls')}
            measured = self._verify_smoke_navigation_wall(observed)
            if measured is not None and measured not in walls:
                walls.append(measured)
        if any(not self._verify_valid_wall(wall) for wall in walls):
            raise ValueError('Invalid retained smoke-wall plane')
        return walls

    def _verify_localization_viewpoint(
        self, geometry=None, index=0, guard=None, origin=None, bearing=None,
    ):
        """Gather a translated view without inventing a source distance."""
        from kv_executive.mission_executive import MissionError
        if not isinstance(index, int) or isinstance(index, bool) or index not in range(6):
            raise MissionError('Localization viewpoint index must be between zero and five')
        _, current = self._verify_control_pose()
        if bearing is None:
            bearing = yaw_from_quaternion(current.orientation)
        return self._verify_navigation_viewpoint(
            bearing, index=index, guard=guard, geometry=geometry)

    def _verify_navigation_preflight(self, guard=None):
        """Check the actual body clearance once before generating any destinations."""
        from kv_executive.mission_executive import MissionError
        from kv_executive.mission_navigation import _inverse
        from kv_executive.srv import QueryInspectionGeometry
        from kv_executive.verification import VerificationStartUnavailable
        if guard:
            guard()
        _, current = self._verify_control_pose()
        request = QueryInspectionGeometry.Request()
        frame = self.config['verify_planning_frame']
        conversion = _inverse(self._path_frame_transform(frame))
        request.terrain_envelope = True
        request.corridor.header.frame_id = frame
        request.corridor.poses = [PoseStamped(
            header=deepcopy(request.corridor.header), pose=transform_pose(current, conversion))]
        # Current clearance must use support below the body, not a raised
        # endpoint query that can mistake an overhead canopy for current ground.
        probe = deepcopy(current)
        request.ground_query.header.frame_id = frame
        request.ground_query.header.stamp = self.get_clock().now().to_msg()
        request.ground_query.point = transform_pose(probe, conversion).position
        response = self._inspection_query(request, require_map=True)
        self._verify_current_terrain = None
        if not response.corridor_safe:
            raise VerificationStartUnavailable(
                'Current body clearance check failed: '
                + (response.message if response is not None else 'no fresh map geometry'))
        if response.ground_valid:
            if (response.ground_surface.header.frame_id != frame
                    or not _finite_point(response.ground_surface.point)
                    or not math.isfinite(response.ground_uncertainty)
                    or response.ground_uncertainty < 0.0):
                raise MissionError('Navigation preflight returned an invalid terrain envelope')
            ground = transform_pose(Pose(
                position=deepcopy(response.ground_surface.point),
                orientation=quaternion_from_yaw(0.0)), self._path_frame_transform(frame)).position
            self._report(
                f'Verify: current terrain-envelope clearance '
                f'{current.position.z - ground.z:.2f} m '
                f'(uncertainty {response.ground_uncertainty:.2f} m); '
                f'local altitude {current.position.z:.2f} m')
            self._verify_current_terrain = {
                'position': deepcopy(current.position),
                'stamp_ns': _stamp_ns(response.map_header),
                'terrain': (ground.z, response.ground_uncertainty),
            }
        else:
            self._report(
                'Verify: current body collision check passed; terrain-envelope height unavailable')
        return self._verify_control_pose()[1]

    def _verify_navigation_viewpoint(
        self, bearing, index=0, guard=None, geometry=None, maximum_forward=None,
        forward_only=False, vertical_offset=0.0, allow_oblique=False,
    ):
        """Advance along a source bearing with terrain clearance and parallax.

        An observation-directed endpoint is not a source position or range.
        Height estimates only select candidates; the native planner and every
        in-flight validation still check mapped obstacles and measured terrain.
        """
        from kv_executive.mission_executive import MissionError
        self._verify_navigation_failure_reason = ''
        if (not isinstance(index, int) or isinstance(index, bool) or index < 0
                or not isinstance(bearing, (float, int)) or not math.isfinite(bearing)
                or not isinstance(forward_only, bool)
                or not isinstance(allow_oblique, bool)
                or isinstance(vertical_offset, bool)
                or not isinstance(vertical_offset, (float, int))
                or not math.isfinite(vertical_offset)
                or abs(vertical_offset) > self.config.get('verify_vertical_step', 0.5) + 1e-9):
            raise MissionError(
                'Navigation viewpoint requires a finite heading, valid index and bounded '
                'vertical offset')
        # Accept the wall while its image and map evidence are fresh. A slow
        # preflight must not silently discard an already observed boundary and
        # replace a limited final leg with an unrestricted forward destination.
        smoke_walls = self._verify_smoke_navigation_walls(geometry)
        current = self._verify_navigation_preflight(guard)
        if geometry and math.isfinite(geometry.get('navigation_bearing', math.nan)):
            stamp = geometry.get('source_stamp')
            if stamp is not None:
                source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
                if -self.config['verify_future_tolerance'] <= age <= self.config[
                        'verify_observation_timeout']:
                    bearing = geometry['navigation_bearing']
        step = self.config.get('verify_navigation_step', 3.0)
        if maximum_forward is not None:
            if not math.isfinite(maximum_forward) or maximum_forward <= 0.0:
                raise MissionError('Navigation forward limit must be finite and positive')
            step = min(step, maximum_forward)
        if smoke_walls:
            # Commit this measured bound with the route. Image ageing or loss
            # during flight must not discard it or interrupt the selected leg.
            geometry = {**geometry, 'smoke_navigation_walls': smoke_walls,
                        'smoke_navigation_wall': smoke_walls[-1]}
        for smoke_wall in smoke_walls:
            remaining = (smoke_wall['nearest'] - smoke_wall['uncertainty']
                         - self.config.get('verify_smoke_standoff', 15.0)
                         - self.config.get('position_tolerance', 0.3)
                         - sum((getattr(current.position, a) - getattr(smoke_wall['origin'], a))
                               * smoke_wall['axis'][i] for i, a in enumerate('xyz')))
            projection = (math.cos(bearing) * smoke_wall['axis'][0]
                          + math.sin(bearing) * smoke_wall['axis'][1])
            if remaining <= 1e-6:
                self._verify_navigation_failure_reason = 'measured smoke-wall boundary reached'
                self._report('Verify: further smoke advance withheld at the measured boundary')
                return False
            if projection > 1e-6:
                step = min(step, remaining / projection)
        minimum_agl = self.config.get('verify_inspection_altitude', 3.0)
        preferred_agl = self.config.get('verify_navigation_preferred_agl', 7.5)
        maximum_agl = self.config.get('verify_navigation_max_agl', 10.0)
        limit = self.config.get('verify_navigation_candidate_limit', 18)
        if (not all(math.isfinite(value) and value > 0.0 for value in (
                step, minimum_agl, preferred_agl, maximum_agl))
                or not minimum_agl <= preferred_agl <= maximum_agl
                or not isinstance(limit, int) or isinstance(limit, bool) or limit <= 0):
            raise MissionError(
                'Invalid terrain navigation step, clearance band or candidate limit')
        minimum_step = max(0.1, 2.0 * self.config.get('position_tolerance', 0.3))
        if geometry and (geometry.get('center_fire_wall') is not None or smoke_walls):
            # The last wall-limited advance may be shorter than a survey leg.
            minimum_step = min(minimum_step, step)
        sign = 1.0 if index % 2 == 0 else -1.0
        # Both oblique sides precede a short direct step. Each successful move
        # starts the next fan at actual odometry, never at an old lateral anchor.
        forward_offsets = [
            (step, sign * step * 0.5), (step, -sign * step * 0.5),
            (step * 0.5, sign * step * 0.25),
            (step * 0.5, -sign * step * 0.25), (step * 0.25, 0.0),
        ]
        approach_offsets = [
            (step, 0.0), (step, sign * step * 0.5), (step, -sign * step * 0.5),
            (step * 0.5, 0.0), (step * 0.5, sign * step * 0.25),
            (step * 0.5, -sign * step * 0.25), (step * 0.25, 0.0),
        ]
        # Tangential and slightly retreating views can follow a convex terrain
        # boundary even when every direct/oblique advance is blocked.
        fallback_offsets = [(0.0, sign * step * 0.5), (0.0, -sign * step * 0.5),
                            (-step * 0.25, sign * step),
                            (-step * 0.25, -sign * step), (0.0, 0.0)]
        terrain_cache = {}
        attempted = set()
        missing_terrain = set()
        height_limited = set()
        minimum_height = self.config.get('verify_vertical_minimum_change', 0.1)
        if not math.isfinite(minimum_height) or minimum_height <= 0.0:
            raise MissionError('Invalid useful height movement')
        attempts = 0
        range_status = 'image-bearing guidance; terrain clearance verified separately'
        reserved = min(5, limit // 3)
        forward_limit = limit - reserved

        def candidates(offsets, clearances, vertical_bias, level_only=False):
            for forward, lateral in offsets:
                if guard:
                    guard()
                target = deepcopy(current)
                target.position.x += forward * math.cos(bearing) - lateral * math.sin(bearing)
                target.position.y += forward * math.sin(bearing) + lateral * math.cos(bearing)
                target.orientation = quaternion_from_yaw(bearing)
                key = (forward, lateral)
                if key not in terrain_cache:
                    probe = deepcopy(target)
                    probe.position.z += maximum_agl + minimum_agl
                    terrain_cache[key] = self._inspection_ground(
                        probe, terrain_envelope=True, require_map=True)
                terrain = terrain_cache[key]
                if terrain is None:
                    missing_terrain.add(key)
                for clearance in clearances if terrain is not None else (None,):
                    requested_agl = minimum_agl if clearance is None else clearance
                    target.position.z = max(current.position.z + vertical_bias,
                                            self.config['verify_minimum_altitude'])
                    if terrain is not None:
                        ground_z, uncertainty = terrain
                        target.position.z = max(
                            target.position.z, ground_z + requested_agl + uncertainty,
                            ground_z + minimum_agl + uncertainty
                            + self.config.get('position_tolerance', 0.3))
                    if level_only and target.position.z > current.position.z + 1e-6:
                        # Examine lower oblique routes before accepting a
                        # climb over a high straight destination.
                        continue
                    if vertical_bias:
                        signed_height = ((target.position.z - current.position.z)
                                         * math.copysign(1.0, vertical_bias))
                        if ((vertical_bias < 0.0 and terrain is None)
                                or signed_height < minimum_height - 1e-9
                                or signed_height > abs(vertical_bias) + 1e-9):
                            height_limited.add(key)
                            # The controller tries one height-only correction
                            # before selecting an independent forward fallback.
                            continue
                        if (maximum_forward is not None and forward > maximum_forward
                                * signed_height / abs(vertical_bias) + 1e-9):
                            # The image-angle limit used the requested height
                            # change. A terrain-clipped correction can support
                            # proportionally less forward travel.
                            continue
                    displacement = math.dist(
                        tuple(getattr(current.position, axis) for axis in 'xyz'),
                        tuple(getattr(target.position, axis) for axis in 'xyz'))
                    # This is a local observation leg, not an unbounded climb
                    # to an arbitrary overhead structure in a large map.
                    if (displacement + 1e-9 < minimum_step
                            or displacement > max(10.0, step * 4.0) + 1e-9):
                        continue
                    coordinates = tuple(round(getattr(target.position, axis), 6) for axis in 'xyz')
                    if coordinates in attempted:
                        continue
                    attempted.add(coordinates)
                    yield deepcopy(target), terrain, forward, lateral, clearance

        if vertical_offset:
            offsets = (approach_offsets if forward_only and allow_oblique else
                       [(step, 0.0), (step * 0.5, 0.0), (step * 0.25, 0.0)]
                       if forward_only else forward_offsets + fallback_offsets)
            # The fire selects a small change from the current altitude. The
            # preferred AGL bands must not replace it with an unrelated climb.
            batches = ((offsets, (None,), limit, False),)
        elif forward_only and allow_oblique:
            # First examine both sides without extra climb. Then use minimum
            # terrain-required rises before higher survey-altitude preferences.
            batches = ((approach_offsets, (None,), limit, True),
                       (approach_offsets, (None,), limit, False),
                       (approach_offsets, (preferred_agl, maximum_agl), limit, False))
        elif forward_only:
            straight = [(step, 0.0), (step * 0.5, 0.0), (step * 0.25, 0.0)]
            # Try all straight distances near current altitude before requesting
            # extra height. The measured terrain floor can still require a climb.
            batches = ((straight, (None,), limit, False),
                       (straight, (preferred_agl, minimum_agl, maximum_agl), limit, False))
        else:
            batches = (
                (forward_offsets, (preferred_agl, minimum_agl, maximum_agl), forward_limit, False),
                (fallback_offsets, (minimum_agl,), limit, False))

        def try_candidates(offsets, clearances, budget, vertical_bias, level_only):
            nonlocal attempts
            for target, terrain, forward, lateral, clearance in candidates(
                    offsets, clearances, vertical_bias, level_only):
                if attempts >= budget:
                    break
                self._check_running()
                if guard:
                    guard()
                attempts += 1
                requested_agl = minimum_agl if clearance is None else clearance
                terrain_status = (
                    f'observed clearance={target.position.z - terrain[0]:.2f} m'
                    if terrain is not None else 'ground height unavailable; retaining altitude')
                self._report(
                    f'Verify: terrain navigation candidate {attempts}/{limit}; '
                    f'forward={forward:.2f} m, lateral={lateral:.2f} m, '
                    f'local target=({target.position.x:.2f}, {target.position.y:.2f}, '
                    f'{target.position.z:.2f}) m, requested AGL={requested_agl:.2f} m, '
                    + ('level preferred; ' if clearance is None else '') +
                    f'{terrain_status}, ' + range_status)
                if self._verify_recovery_plan_to(
                        target, geometry, guard, terrain=terrain,
                        preserve_requested_altitude=bool(vertical_bias)):
                    return True
            return False

        for offsets, clearances, budget, level_only in batches:
            if try_candidates(offsets, clearances, budget, vertical_offset, level_only):
                return True
        mode = 'forward' if forward_only else 'terrain navigation'
        if attempts:
            reason = (f'{attempts} {mode} candidates rejected by planning or clearance checks; '
                      f'{len(missing_terrain)} sampled destinations lack observed terrain')
        elif missing_terrain:
            reason = (f'No observed terrain envelope at {len(missing_terrain)} sampled '
                      f'{mode} destinations')
        elif height_limited:
            reason = ('No combined movement candidate permits a useful bounded height change; '
                      'a height-only correction must be considered before forward fallback')
        else:
            reason = f'No {mode} candidate satisfies the bounded movement constraints'
        self._verify_navigation_failure_reason = reason
        self._report(
            f'Verify: no terrain navigation path after {attempts} bounded candidates; '
            + reason)
        return False

    def _verify_adaptive_localization_target(self, target, geometry=None, guard=None):
        """Try a useful translated view with the configured collision policy.

        A fixed viewpoint can lie beyond the current sensor coverage even when
        a shorter step is available. Every candidate still goes through the
        planner and in-flight obstacle/terrain checks.
        """
        _, current = self._verify_control_pose()
        distance = math.dist(
            tuple(getattr(current.position, axis) for axis in 'xyz'),
            tuple(getattr(target.position, axis) for axis in 'xyz'))
        minimum_step = max(0.1, 2.0 * self.config.get('position_tolerance', 0.3))
        if not math.isfinite(distance) or distance < minimum_step:
            self._report('Verify: localization candidate has no useful translation')
            return False
        for scale in (1.0, 0.5, 0.25, 0.125):
            if distance * scale < minimum_step:
                break
            self._check_running()
            if guard:
                guard()
            candidate = deepcopy(target)
            for axis in 'xyz':
                start = getattr(current.position, axis)
                finish = getattr(target.position, axis)
                setattr(candidate.position, axis, start + scale * (finish - start))
            if scale < 1.0:
                self._report(
                    f'Verify: trying shorter localization step of {distance * scale:.2f} m '
                    'with obstacle and terrain checks')
            if self._verify_recovery_plan_to(candidate, geometry, guard):
                return True
        self._report(
            'Verify: no useful localization step has a collision-checked terrain path')
        return False

    def _verify_smoke_boundary_reached(self, geometry):
        """Recognize a fresh observed smoke-wall proxy, never a fire range."""
        if (not geometry or geometry.get('source') != 'smoke'
                or not geometry.get('plume_candidate', False)):
            return False
        point = geometry.get('obstacle_point')
        uncertainty = geometry.get('obstacle_uncertainty', math.nan)
        distance = geometry.get('obstacle_range', math.nan)
        stamp = geometry.get('source_stamp')
        map_header = geometry.get('map_header')
        if (point is None or not _finite_point(point) or stamp is None or map_header is None
                or map_header.frame_id != self.config['verify_planning_frame']
                or not all(math.isfinite(value) for value in (uncertainty, distance))
                or not 0.0 <= uncertainty <= self.config.get(
                    'verify_inspection_max_uncertainty', 2.0)
                or distance <= 0.0):
            return False
        now_ns = self.get_clock().now().nanoseconds
        for source_stamp, timeout in (
                (stamp, self.config['verify_observation_timeout']),
                (map_header.stamp, self.config.get('verify_geometry_timeout', 5.0))):
            stamp_ns = source_stamp.sec * 1_000_000_000 + source_stamp.nanosec
            age = (now_ns - stamp_ns) / 1e9
            if stamp_ns <= 0 or not -self.config['verify_future_tolerance'] <= age <= timeout:
                return False
        _, current = self._verify_control_pose()
        current_distance = math.dist(
            tuple(getattr(current.position, axis) for axis in 'xyz'),
            tuple(getattr(point, axis) for axis in 'xyz'))
        # A steep downward ray must not claim arrival solely from its XY distance.
        # Match the outer arrival band so a final smoke leg need not be smaller
        # than the controller's position tolerance. Positive reports separately
        # enforce the conservative range's strict lower standoff bound.
        limit = (self.config.get('verify_smoke_standoff', 15.0) + uncertainty
                 + 2.0 * self.config.get('position_tolerance', 0.3))
        return max(distance, current_distance) <= limit + 1e-9

    def _verify_source_at_standoff(self, geometry):
        """Compatibility alias for smoke-wall arrival; fire range never qualifies."""
        return self._verify_smoke_boundary_reached(geometry)

    def _inspection_fresh_fix(self):
        with self._data_lock:
            fix = deepcopy(self._global_position)
            received = self._global_position_received
        if not isinstance(fix, NavSatFix):
            return None
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - _stamp_ns(fix)) / 1e9
        if (now_ns <= 0 or _stamp_ns(fix) <= 0
                or not -self.config['verify_future_tolerance'] <= age
                <= self.config['global_position_timeout']
                or not 0.0 <= time.monotonic() - received
                <= self.config['global_position_timeout']
                or fix.status.status < NavSatStatus.STATUS_FIX
                or not all(math.isfinite(value) for value in (
                    fix.latitude, fix.longitude, fix.altitude))
                or not -90.0 < fix.latitude < 90.0
                or not -180.0 <= fix.longitude <= 180.0):
            return None
        return fix

    def _inspection_target_fields(
            self, report, prefix, geometry, fix, observer, stamp, *, retained=False):
        local_frame = self.config['local_frame']
        planning_frame = self.config['verify_planning_frame']
        setattr(report, prefix + '_local_position', _unknown_point(local_frame, stamp))
        setattr(report, prefix + '_planning_position', _unknown_point(planning_frame, stamp))
        setattr(report, prefix + '_global_position', _unknown_fix(stamp))
        setattr(report, prefix + '_global_valid', False)
        setattr(report, prefix + '_uncertainty_m', math.nan)
        setattr(report, prefix + '_location_method', 'unavailable')
        point = geometry.get('point') if geometry else None
        planning = geometry.get('planning_point') if geometry else None
        uncertainty = geometry.get('uncertainty', math.nan) if geometry else math.nan
        source_stamp = geometry.get('source_stamp') if geometry else None
        source_ns = (source_stamp.sec * 1_000_000_000 + source_stamp.nanosec
                     if source_stamp is not None else 0)
        age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
        valid = (
            geometry is not None and geometry.get('target_valid', False)
            and point is not None and _finite_point(point) and planning is not None
            and planning.header.frame_id == planning_frame and _finite_point(planning.point)
            and math.isfinite(uncertainty) and uncertainty >= 0.0 and source_ns > 0
            and geometry.get('source', prefix) == prefix
            and (retained or -self.config['verify_future_tolerance'] <= age
                 <= self.config['verify_observation_timeout'])
        )
        setattr(report, prefix + '_location_valid', valid)
        if not valid:
            return
        local = PointStamped()
        local.header.frame_id = local_frame
        local.header.stamp = deepcopy(source_stamp)
        local.point = deepcopy(point)
        setattr(report, prefix + '_local_position', local)
        setattr(report, prefix + '_planning_position', deepcopy(planning))
        setattr(report, prefix + '_uncertainty_m', float(uncertainty))
        setattr(report, prefix + '_location_method', geometry['method'])
        if fix is not None:
            target_fix = enu_offset_to_fix(
                fix, point.x - observer.position.x, point.y - observer.position.y,
                point.z - observer.position.z, uncertainty)
            if target_fix is not None:
                setattr(report, prefix + '_global_valid', True)
                setattr(report, prefix + '_global_position', target_fix)

    def _inspection_retained_smoke_geometry(self, wall, conversion=None):
        """Transform the confirmed historical hit, never reproject an old range."""
        from kv_executive.verification import VerificationObservationLost
        frame = self.config['verify_planning_frame']
        if not retained_wall_valid(wall, frame, self.config['verify_inspection_max_uncertainty']):
            raise VerificationObservationLost('Retained smoke-wall confirmation is invalid')
        point = wall['nearest_point']
        uncertainty = wall['uncertainty']
        if conversion is None:
            conversion = self._path_frame_transform(frame)
        local = transform_pose(Pose(
            position=deepcopy(point.point), orientation=quaternion_from_yaw(0.0)),
            conversion).position
        return {'point': local, 'planning_point': deepcopy(point),
                'uncertainty': uncertainty, 'source_stamp': deepcopy(point.header.stamp),
                'source': 'smoke', 'target_valid': True,
                'method': 'confirmed_smoke_wall_nearest_hit'}

    def _verify_report_inspection(
        self, smoke_geometry, fire_message=None, attempts=1, *, wall=None, guard=None,
    ):
        """Publish target estimates and a found/not-found result separately from the UAV."""
        from kv_executive.mission_executive import MissionError
        from kv_executive.msg import InspectionReport
        from kv_executive.verification import VerificationObservationLost
        if not isinstance(attempts, int) or isinstance(attempts, bool) or not 1 <= attempts <= 3:
            raise MissionError('Inspection attempts must be between one and three')

        def check():
            self._check_running()
            if guard:
                guard()

        check()
        retained_wall = getattr(self, '_verify_retained_report_wall', None)
        smoke_geometry = self._inspection_retained_smoke_geometry(retained_wall)
        self._verify_control_pose()
        current_smoke = self._verify_prepare_report_view(
            retained_wall, fire_message=fire_message, guard=check)
        view_after_ns = getattr(self, '_verify_report_view_after_ns', 0)
        if fire_message is None:
            # A track may become persistent during the final smoke query.
            # Its appearance cannot be reported as a negative inspection.
            fire_message = self._verify.stable_fire()
        if (fire_message is None
                and attempts < self.config.get('verify_inspection_max_attempts', 3)):
            raise VerificationObservationLost('No-fire inspection attempts are incomplete')

        def visual_smoke_valid():
            smoke = self._verify.fresh_smoke()
            return (self._inspection_detection_valid(smoke, 'smoke')
                    and _stamp_ns(smoke) > view_after_ns
                    and self._verify_wall_observation_ready()
                    and getattr(self, '_verify_retained_report_wall', None) is retained_wall)

        fire_geometry = None
        if fire_message is not None:
            current_fire = self._verify.stable_fire()
            if (not self._inspection_detection_valid(current_fire, 'fire')
                    or current_fire.header.frame_id != fire_message.header.frame_id
                    or getattr(current_fire, 'track_id', 0)
                    != getattr(fire_message, 'track_id', 0)):
                raise VerificationObservationLost('Tracked fire changed before inspection report')
            fire_geometry = self._verify_target_geometry(current_fire, 'fire')
        elif current_smoke is None:
            raise VerificationObservationLost('Smoke lost before not-found inspection report')
        self._verify_control_pose()
        _, odometry = self._snapshot(require_armed=True)
        now_ns = self.get_clock().now().nanoseconds
        odometry_age = (now_ns - _stamp_ns(odometry)) / 1e9
        if (_stamp_ns(odometry) <= 0
                or not -self.config['verify_future_tolerance'] <= odometry_age
                <= self.config['telemetry_timeout']):
            raise MissionError('Cannot report inspection with stale observation pose')
        if fire_message is not None:
            latest = self._verify.stable_fire()
            if (not self._inspection_detection_valid(latest, 'fire')
                    or latest.header.frame_id != fire_message.header.frame_id
                    or getattr(latest, 'track_id', 0) != getattr(fire_message, 'track_id', 0)):
                raise VerificationObservationLost('Fire changed while locating inspection target')
        else:
            if self._verify.stable_fire() is not None:
                raise VerificationObservationLost('Fire became persistent during no-fire report')
            if self._verify.fresh_smoke() is None:
                raise VerificationObservationLost(
                    'Smoke disappeared while locating inspection target')
        fix = self._inspection_fresh_fix()
        report = InspectionReport()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = self.config['verify_planning_frame']
        report.fire_found = fire_message is not None
        report.inspection_attempts = attempts
        report.status = 'fire_found' if report.fire_found else 'fire_not_found'
        source_count = int(getattr(fire_message, 'source_count', 1)) if report.fire_found else 0
        track_id = int(getattr(fire_message, 'track_id', 0)) if report.fire_found else 0
        if (not 0 <= source_count < 2 ** 32 or not 0 <= track_id < 2 ** 64
                or (report.fire_found and source_count == 0)):
            raise MissionError('Invalid fire source identity in inspection report')
        report.source_count = source_count
        report.track_id = track_id
        report.smoke_wall_confirmed = True
        report.smoke_wall_nearest_depth_m = retained_wall['nearest']
        report.smoke_wall_evidence = str(retained_wall.get('evidence', 'unknown'))
        report.observer_local_pose.header = deepcopy(odometry.header)
        report.observer_local_pose.pose = deepcopy(odometry.pose.pose)
        report.observer_planning_pose.header = deepcopy(report.header)
        conversion = self._path_frame_transform(self.config['verify_planning_frame'])
        from kv_executive.mission_navigation import _inverse
        report.observer_planning_pose.pose = transform_pose(
            odometry.pose.pose, _inverse(conversion))
        report.observer_global_valid = fix is not None
        report.observer_global_position = deepcopy(fix) if fix is not None else _unknown_fix(
            report.header.stamp)
        if not report.fire_found and self._verify.had_fire_confirmation():
            raise VerificationObservationLost(
                'Confirmed fire was observed during this inspection; no-fire result withheld')
        if not visual_smoke_valid():
            raise VerificationObservationLost(
                'Fresh forward-camera smoke is required for the inspection result')
        self._inspection_target_fields(
            report, 'smoke', smoke_geometry, fix, odometry.pose.pose,
            report.header.stamp, retained=True)
        self._inspection_target_fields(
            report, 'fire', fire_geometry, fix, odometry.pose.pose, report.header.stamp)
        legacy = None
        if report.fire_found:
            latest = self._verify.stable_fire()
            if (not self._inspection_detection_valid(latest, 'fire')
                    or latest.header.frame_id != fire_message.header.frame_id
                    or getattr(latest, 'track_id', 0) != getattr(fire_message, 'track_id', 0)):
                raise VerificationObservationLost(
                    'Fire changed while preparing the inspection result')
            fix = self._inspection_fresh_fix()
        if report.fire_found and fix is not None:
            # Preserve the existing observation-viewpoint topic for consumers
            # that have not migrated. Target coordinates live only above.
            legacy = self._verify_record_fire(
                latest, yaw_from_quaternion(odometry.pose.pose.orientation)
                + latest.yaw_correction_rad, require_centering=False, publish=False,
            )
        fix = self._inspection_fresh_fix()
        report.observer_global_valid = fix is not None
        report.observer_global_position = deepcopy(fix) if fix is not None else _unknown_fix(
            report.header.stamp)
        self._inspection_target_fields(
            report, 'smoke', smoke_geometry, fix, odometry.pose.pose,
            report.header.stamp, retained=True)
        self._inspection_target_fields(
            report, 'fire', fire_geometry, fix, odometry.pose.pose, report.header.stamp)
        self._inspection_capture_evidence(
            report, fire_message=fire_message if report.fire_found else None,
            guard=check, after_ns=view_after_ns)
        # Decide classification after the last potentially blocking query too.
        check()
        # Map/local alignment may have changed during viewing or image capture.
        # Keep the saved map hit fixed and use one current conversion for both
        # targets and the observer before deriving current GPS offsets.
        conversion = self._path_frame_transform(self.config['verify_planning_frame'])
        smoke_geometry = self._inspection_retained_smoke_geometry(retained_wall, conversion)
        planning_fire = fire_geometry.get('planning_point') if fire_geometry else None
        if (planning_fire is not None
                and planning_fire.header.frame_id == self.config['verify_planning_frame']
                and _finite_point(planning_fire.point)):
            fire_geometry = {**fire_geometry, 'point': transform_pose(Pose(
                position=deepcopy(planning_fire.point), orientation=quaternion_from_yaw(0.0)),
                conversion).position}
        check()
        if not visual_smoke_valid():
            raise VerificationObservationLost(
                'Forward smoke or the settled report viewpoint changed before publishing')
        latest = self._verify.stable_fire()
        if report.fire_found:
            if (not self._inspection_detection_valid(latest, 'fire')
                    or latest.header.frame_id != fire_message.header.frame_id
                    or getattr(latest, 'track_id', 0) != getattr(fire_message, 'track_id', 0)):
                raise VerificationObservationLost(
                    'Fire changed before publishing inspection result')
        else:
            if self._verify.had_fire_confirmation():
                raise VerificationObservationLost(
                    'Confirmed fire was observed during this inspection; no-fire result withheld')
            current_smoke = self._verify.fresh_smoke()
            if latest is not None or current_smoke is None:
                raise VerificationObservationLost(
                    'Smoke or no-fire evidence changed before publishing')
        # Refresh observer telemetry after resolving the final common transform.
        self._verify_control_pose()
        _, odometry = self._snapshot(require_armed=True)
        now_ns = self.get_clock().now().nanoseconds
        odometry_age = (now_ns - _stamp_ns(odometry)) / 1e9
        if (_stamp_ns(odometry) <= 0
                or not -self.config['verify_future_tolerance'] <= odometry_age
                <= self.config['telemetry_timeout']):
            raise MissionError('Observation pose became stale before publishing inspection')
        report.header.stamp = self.get_clock().now().to_msg()
        report.observer_local_pose.header = deepcopy(odometry.header)
        report.observer_local_pose.pose = deepcopy(odometry.pose.pose)
        report.observer_planning_pose.header = deepcopy(report.header)
        report.observer_planning_pose.pose = transform_pose(
            odometry.pose.pose, _inverse(conversion))
        fix = self._inspection_fresh_fix()
        report.observer_global_valid = fix is not None
        report.observer_global_position = deepcopy(fix) if fix is not None else _unknown_fix(
            report.header.stamp)
        self._inspection_target_fields(
            report, 'smoke', smoke_geometry, fix, odometry.pose.pose,
            report.header.stamp, retained=True)
        self._inspection_target_fields(
            report, 'fire', fire_geometry, fix, odometry.pose.pose, report.header.stamp)
        if report.fire_found:
            source_count = int(getattr(latest, 'source_count', 1))
            if not 0 < source_count < 2 ** 32:
                raise MissionError(
                    'Invalid current fire source count before publishing inspection')
            report.source_count = source_count
        check()
        if legacy is not None and fix is not None:
            legacy.header = deepcopy(report.header)
            legacy.local_pose = deepcopy(report.observer_local_pose)
            legacy.planning_pose = deepcopy(report.observer_planning_pose)
            legacy.global_position = deepcopy(fix)
            legacy.source_count = report.source_count
            legacy.bearing_rad = float(math.remainder(
                yaw_from_quaternion(odometry.pose.pose.orientation) + latest.yaw_correction_rad,
                math.tau))
            report.site_id = int(self._verify_publish_fire_report(legacy).site_id)
        else:
            with self._data_lock:
                self._site_sequence += 1
                report.site_id = self._site_sequence
        self._inspection_report_publisher.publish(report)
        complete = getattr(self, '_verify_complete_site', None)
        if complete is not None:
            self._verify_report_completion_context = {
                'smoke': deepcopy(self._verify.fresh_smoke()),
                'fire': deepcopy(latest) if report.fire_found else None,
                'planning_to_local': deepcopy(conversion)}
            if retained_wall.get('map_origin') is not None:
                self._verify_report_completion_context['wall_observer'] = deepcopy(
                    retained_wall['map_origin'])
            try:
                saved = complete(report, message=latest if report.fire_found else current_smoke,
                                 source='fire' if report.fire_found else 'smoke')
            finally:
                self._verify_report_completion_context = None
            self._report(f'Verify: completed cluster recorded={bool(saved)}; '
                         f'site_id={report.site_id}')
            publish_clusters = getattr(self, '_publish_completed_clusters', None)
            if publish_clusters is not None:
                publish_clusters()
        self._report(f'Verify: inspection result {report.status}; '
                     f'smoke location valid={report.smoke_location_valid}, '
                     f'fire location valid={report.fire_location_valid}')
        return report
