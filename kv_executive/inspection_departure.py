"""Leave completed inspection clusters without crossing their vertical footprints."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Pose, PoseStamped
from kv_executive.mission_geometry import quaternion_from_yaw, transform_pose
from nav_msgs.msg import Path


DEPARTURE_DEFAULTS = {
    'verify_departure_timeout': 180.0,
    'verify_departure_step': 3.0,
    'verify_departure_max_legs': 12,
    'verify_departure_clearance': 1.0,
}


def cluster_path_safe(points, regions, escape_origin=None):
    """Check swept XY segments, including a monotonic escape from an inflated buffer."""
    if not points or any(not all(math.isfinite(getattr(p, a)) for a in 'xyz') for p in points):
        return False
    for region in regions:
        center = region['center'].point
        radius = region['radius']
        if not math.isfinite(radius) or radius <= 0.0:
            return False
        anchor = escape_origin
        escape = (anchor is not None
                  and math.hypot(anchor.x - center.x, anchor.y - center.y) < radius)
        anchor_distance = (math.hypot(anchor.x - center.x, anchor.y - center.y)
                           if escape else radius)
        for index, end in enumerate(points):
            start = points[max(0, index - 1)]
            ax, ay = start.x - center.x, start.y - center.y
            vx, vy = end.x - start.x, end.y - start.y
            distance = math.hypot(ax, ay)
            if escape and distance < radius - 1e-6:
                if distance < anchor_distance - 1e-6 or ax * vx + ay * vy < -1e-6:
                    return False
            else:
                length = vx * vx + vy * vy
                fraction = max(0.0, min(1.0, -(ax * vx + ay * vy) / length)) if length else 0.0
                if math.hypot(ax + fraction * vx, ay + fraction * vy) < radius - 1e-6:
                    return False
    return True


class InspectionDepartureMixin:
    """Bound departure separately from the already completed inspection."""

    def _departure_regions(self):
        return self._completed_cluster_regions()

    def _departure_map_path(self, path):
        from kv_executive.mission_navigation import _inverse
        conversion = _inverse(self._path_frame_transform(self.config['verify_planning_frame']))
        return [transform_pose(p.pose, conversion).position for p in path.poses], conversion

    def _verify_cluster_path(self, path, escape_origin=None):
        """Enforce every retained footprint, independent of the native planner response."""
        points, conversion = self._departure_map_path(path)
        anchor = (transform_pose(Pose(position=deepcopy(escape_origin),
                                      orientation=quaternion_from_yaw(0.0)), conversion).position
                  if escape_origin is not None else None)
        return cluster_path_safe(points, self._departure_regions(), anchor)

    def _verify_departure_leg(self, target, current, guard):
        minimum_agl = self.config['verify_inspection_altitude']
        probe = deepcopy(target)
        probe.position.z += self.config['verify_navigation_max_agl']
        terrain = self._inspection_ground(probe, terrain_envelope=True, require_map=True)
        if terrain is None:
            return False
        target.position.z = max(target.position.z, terrain[0] + terrain[1] + minimum_agl
                                + self.config['position_tolerance'])
        anchor = deepcopy(current.position)
        state = {'origin': anchor, 'complete': False}

        def validate(path):
            return (self._verify_cluster_path(path, escape_origin=anchor)
                    and self._inspection_map_corridor(
                        path, minimum_agl=minimum_agl, terrain_envelope=True,
                        allow_initial_climb=True, allow_terrain_escape=True,
                        escape_state=state, allow_completed_cluster_escape=True))

        build = self._inspection_request_builder(
            minimum_agl=minimum_agl, allow_initial_climb=True,
            terrain_envelope=True, allow_terrain_escape=True)

        def request(target_map, conversion):
            result = build(target_map, conversion)
            result.allow_completed_cluster_escape = True
            return result

        heading = math.atan2(target.position.y - current.position.y,
                             target.position.x - current.position.x)
        target.orientation = quaternion_from_yaw(heading)
        # The flight helper validates the plan before OFFBOARD, then turns
        # toward this fixed heading before translating. Do not enter OFFBOARD
        # twice or turn toward a candidate that planning will refuse.
        stationary = Path()
        stationary.header.frame_id = self.config['local_frame']
        stationary.poses = [PoseStamped(pose=deepcopy(current))]
        if not self._inspection_map_corridor(stationary, terrain_envelope=True):
            return False
        return self._verify_plan_to(
            target, guard=guard, allow_plan_failure=True,
            path_validator=validate, heading_provider=lambda _pose: heading,
            planning_client=self._inspection_planning_client, request_builder=request,
            maximum_path_length=self.config['verify_departure_step'] * 3.0,
            motion_timeout=min(45.0, self.config['verify_departure_timeout']),
            motion_label='DEPART_SITE')

    def _verify_depart_completed_site(self, report, guard=None):
        from kv_executive.mission_executive import MissionError
        from kv_executive.mission_navigation import _inverse
        from kv_executive.verification import VerificationObservationLost
        regions = self._departure_regions()
        if not any(r['site_id'] == report.site_id for r in regions):
            raise MissionError('DEPART_SITE: completed report has no saved cluster; holding')
        deadline = time.monotonic() + self.config['verify_departure_timeout']

        def check():
            self._check_running()
            if guard:
                guard()
            if not any(r['site_id'] == report.site_id for r in self._departure_regions()):
                raise MissionError('DEPART_SITE: cluster registry changed epoch; holding')
            if time.monotonic() >= deadline:
                raise MissionError('DEPART_SITE: departure time budget exhausted; '
                                   'inspection remains reported; holding')

        for leg in range(self.config['verify_departure_max_legs'] + 1):
            check()
            _, current = self._verify_control_pose()
            conversion = self._path_frame_transform(self.config['verify_planning_frame'])
            regions = self._departure_regions()
            map_position = transform_pose(current, _inverse(conversion)).position
            centers = [(r['center'].point, r['radius']) for r in regions]
            inside = [(c, radius) for c, radius in centers
                      if math.hypot(map_position.x - c.x, map_position.y - c.y)
                      < radius + self.config['verify_departure_clearance']]
            nearest_clearance = min(math.hypot(map_position.x - c.x, map_position.y - c.y)
                                    - radius for c, radius in centers)
            self._report(f'Verify/DEPART_SITE: nearest zone clearance={nearest_clearance:.2f} m; '
                         f'exit margin={self.config["verify_departure_clearance"]:.2f} m')
            if not inside:
                self._report('Verify/DEPART_SITE: outside completed cluster buffers; '
                             'looking toward uninspected space')
                # A new target is used only if it survived the shared completion filter.
                candidate = self._verify.stable_fire() or self._verify.fresh_smoke()
                bearing = self._verify_image_bearing(candidate) if candidate is not None else None
                if bearing is None:
                    nearest, _ = min(centers, key=lambda c: math.hypot(
                        map_position.x - c[0].x, map_position.y - c[0].y))
                    away = deepcopy(map_position)
                    away.x += map_position.x - nearest.x
                    away.y += map_position.y - nearest.y
                    local_away = transform_pose(Pose(
                        position=away, orientation=quaternion_from_yaw(0.0)), conversion).position
                    bearing = math.atan2(local_away.y - current.position.y,
                                         local_away.x - current.position.x)
                self._verify_rotate(bearing, guard=check)
                if not self._verify_prepare_wall_observation(
                        guard=check, align_fire=False, wait_for_map=False):
                    raise MissionError('DEPART_SITE: final viewpoint failed to settle; holding')
                # Braking or a final yaw can drift back across the exit margin.
                # Recheck at the actual settled pose before admitting Exploration.
                check()
                _, settled = self._verify_control_pose()
                settled_map = transform_pose(settled, _inverse(self._path_frame_transform(
                    self.config['verify_planning_frame']))).position
                if any(math.hypot(settled_map.x - r['center'].point.x,
                                  settled_map.y - r['center'].point.y)
                       < r['radius'] + self.config['verify_departure_clearance']
                       for r in self._departure_regions()):
                    self._report('Verify/DEPART_SITE: settled viewpoint drifted inside exit '
                                 'margin; continuing checked departure')
                    continue
                self._report('Verify/DEPART_SITE: handoff complete; resuming Exploration')
                return
            if leg == self.config['verify_departure_max_legs']:
                break
            dx = sum((map_position.x - c.x) / max(0.1, math.hypot(
                map_position.x - c.x, map_position.y - c.y)) for c, _ in inside)
            dy = sum((map_position.y - c.y) / max(0.1, math.hypot(
                map_position.x - c.x, map_position.y - c.y)) for c, _ in inside)
            base = math.atan2(dy, dx)
            moved = False
            for offset in (0.0, math.pi / 6, -math.pi / 6, math.pi / 3, -math.pi / 3,
                           math.pi / 2, -math.pi / 2, math.pi):
                check()
                bearing = base + offset
                endpoint = deepcopy(map_position)
                endpoint.x += self.config['verify_departure_step'] * math.cos(bearing)
                endpoint.y += self.config['verify_departure_step'] * math.sin(bearing)
                target = transform_pose(Pose(position=endpoint,
                                             orientation=quaternion_from_yaw(0.0)), conversion)
                candidate = Path()
                candidate.header.frame_id = self.config['local_frame']
                candidate.poses = [PoseStamped(pose=deepcopy(p)) for p in (current, target)]
                if not self._verify_cluster_path(candidate, escape_origin=current.position):
                    continue
                self._report(f'Verify/DEPART_SITE: checking outward leg {leg + 1}; '
                             'routing around completed clusters')
                try:
                    result = self._verify_departure_leg(target, current, check)
                except VerificationObservationLost as error:
                    # The flight helper first recovers LOITER. Never retry a
                    # stopped/partial leg until the actual FCU state confirms it.
                    self._snapshot('AUTO.LOITER', require_armed=True)
                    self._report(f'Verify/DEPART_SITE: candidate unavailable: {error}')
                    result = False
                if result is not False:
                    moved = True
                    break
                _, actual = self._verify_control_pose()
                actual_map = transform_pose(actual, _inverse(self._path_frame_transform(
                    self.config['verify_planning_frame']))).position
                if (math.hypot(actual_map.x - map_position.x,
                               actual_map.y - map_position.y) >= 0.1
                        and cluster_path_safe([map_position, actual_map],
                                              self._departure_regions(), map_position)):
                    self._report('Verify/DEPART_SITE: retaining checked outward progress; '
                                 'refreshing departure viewpoint')
                    moved = True
                    break
            if not moved:
                raise MissionError('DEPART_SITE: no safe outward route; '
                                   'inspection remains reported; holding')
        raise MissionError('DEPART_SITE: departure movement budget exhausted; '
                           'inspection remains reported; holding')
