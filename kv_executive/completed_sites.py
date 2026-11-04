"""Associate completed sources and defer failed viewpoints until exploration advances."""

from collections import deque, OrderedDict
from copy import deepcopy
import math
import time

from geometry_msgs.msg import Point, PointStamped, Pose, Transform, Vector3
from kv_executive.mission_geometry import (
    interpolate_pose, quaternion_from_yaw, transform_pose,
)
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


COMPLETED_SITE_DEFAULTS = {
    'verify_completed_bearing_tolerance': math.radians(10.0),
    'verify_retry_translation': 2.0,
    'verify_retry_bearing_tolerance': math.radians(15.0),
    'verify_completed_cluster_radius': 20.0,
    'verify_completed_cluster_buffer': 5.0,
    'verify_completed_outside_frames': 3,
    'verify_completed_candidate_history': 128,
    'verify_completed_projection_margin': math.radians(3.0),
    'verify_completed_followup_attempts': 2,
    'verify_completed_followup_baseline': 6.0,
    'verify_completed_followup_lease_timeout': 60.0,
    'verify_completed_followup_observation_time': 2.0,
}


def _stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _coordinates(point):
    return tuple(getattr(point, axis) for axis in 'xyz')


def _finite_point(point):
    return point is not None and all(math.isfinite(value) for value in _coordinates(point))


def _angle(left, right):
    cosine = sum(a * b for a, b in zip(left, right)) / (math.hypot(*left) * math.hypot(*right))
    return math.acos(max(-1.0, min(1.0, cosine)))


class CompletedSitesMixin:
    """Keep bounded observer/bearing evidence, independent of detector track IDs."""

    def _initialize_completed_sites(self):
        self._completed_sites = deque(maxlen=100)
        self._completed_clusters = deque(maxlen=100)
        self._completed_candidate_history = OrderedDict()
        self._completed_candidate_sequence = 0
        self._completed_followups = OrderedDict()
        self._completed_diagnostics = {}
        self._completed_clusters_version = 0
        self._completed_clock_ns = self.get_clock().now().nanoseconds
        self._inspection_odometry_history = deque(maxlen=512)
        # An inconclusive inspection is not a completed source location. Keep
        # only its observer and bearing so a sweep can actually make progress.
        self._deferred_viewpoints = deque(maxlen=32)

    def _completed_clock_locked(self):
        """Reset spatial evidence only when the actual source clock rewinds."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < getattr(self, '_completed_clock_ns', 0):
            self._inspection_odometry_history.clear()
            self._completed_sites.clear()
            self._completed_clusters.clear()
            self._deferred_viewpoints.clear()
            self._completed_candidate_history.clear()
            self._completed_followups.clear()
            self._completed_diagnostics.clear()
            self._completed_clusters_version += 1
        self._completed_clock_ns = now_ns

    def _completed_odometry_locked(self, message):
        """Append validated local poses while the caller holds the telemetry lock."""
        history = getattr(self, '_inspection_odometry_history', None)
        if history is None or message.header.frame_id != self.config['local_frame']:
            return
        self._completed_clock_locked()
        stamp_ns = _stamp_ns(message)
        if (stamp_ns <= 0 or stamp_ns > self._completed_clock_ns
                + max(self.config['verify_future_tolerance'],
                      self.config['telemetry_timeout']) * 1e9):
            return
        try:
            pose = interpolate_pose(message.pose.pose, message.pose.pose, 0.0)
        except (ValueError, AttributeError):
            return
        if history and stamp_ns < history[-1][0]:
            return  # Ordinary reordered telemetry does not erase completed sites.
        if history and stamp_ns == history[-1][0]:
            history[-1] = (stamp_ns, pose)
        else:
            history.append((stamp_ns, pose))
        current = tuple(getattr(pose.position, axis) for axis in 'xyz')
        distance = self.config.get('verify_retry_translation', 2.0)
        completed = getattr(self, '_completed_sites', None)
        if completed:
            retained = [entry for entry in completed if math.dist(current, entry[1])
                        + 1e-6 < distance]
            completed.clear()
            completed.extend(retained)
        deferred = getattr(self, '_deferred_viewpoints', None)
        if deferred:
            retained = [entry for entry in deferred if math.dist(current, entry[1])
                        + 1e-6 < distance]
            deferred.clear()
            deferred.extend(retained)

    def _completed_source_pose(self, stamp_ns):
        with self._data_lock:
            history = list(getattr(self, '_inspection_odometry_history', ()))
        tolerance_ns = self.config['telemetry_timeout'] * 1e9
        for index, (sample_ns, pose) in enumerate(history):
            if sample_ns == stamp_ns:
                return deepcopy(pose)
            if sample_ns > stamp_ns:
                if index == 0:
                    return None
                previous_ns, previous = history[index - 1]
                if max(stamp_ns - previous_ns, sample_ns - stamp_ns) > tolerance_ns:
                    return None
                return interpolate_pose(
                    previous, pose, (stamp_ns - previous_ns) / (sample_ns - previous_ns))
        return None  # Never extrapolate a moving camera from its latest pose.

    def _completed_detection_ray(self, message, source):
        """Transform a fresh selected camera ray at its image's actual local pose."""
        if message is None:
            return None
        confirmed = getattr(message, 'CONFIRMED', 3 if source == 'smoke' else 2)
        if (not message.detected or message.status != confirmed
                or (source == 'smoke' and not message.background_ready)):
            return None
        now_ns = self.get_clock().now().nanoseconds
        source_ns = _stamp_ns(message)
        age = (now_ns - source_ns) / 1e9
        if (now_ns <= 0 or source_ns <= 0 or not message.header.frame_id
                or not -self.config['verify_future_tolerance'] <= age
                <= self.config['verify_observation_timeout']):
            return None
        angles = (message.yaw_correction_rad, message.vertical_correction_rad)
        if any(not math.isfinite(angle) or abs(angle) >= math.pi / 2 for angle in angles):
            return None
        body_pose = self._completed_source_pose(source_ns)
        if body_pose is None:
            return None
        try:
            camera = self._tf_buffer.lookup_transform(
                self.config['robot_base_frame'], message.header.frame_id,
                Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.0),
            ).transform
            body = Transform(
                translation=Vector3(**{
                    axis: getattr(body_pose.position, axis) for axis in 'xyz'}),
                rotation=deepcopy(body_pose.orientation),
            )
            origin = Pose(orientation=quaternion_from_yaw(0.0))
            end = Pose(position=Point(
                x=1.0, y=math.tan(angles[0]), z=math.tan(angles[1])),
                orientation=quaternion_from_yaw(0.0))
            origin = transform_pose(transform_pose(origin, camera), body).position
            end = transform_pose(transform_pose(end, camera), body).position
        except (TransformException, ValueError, AttributeError):
            return None
        direction = tuple(getattr(end, axis) - getattr(origin, axis) for axis in 'xyz')
        norm = math.hypot(*direction)
        if not math.isfinite(norm) or norm <= 0.0:
            return None
        direction = tuple(value / norm for value in direction)
        return origin, direction, body_pose.position

    def _completed_legacy_detection(self, message, source):
        """Match fresh camera bearings only while the observer has not translated."""
        with self._data_lock:
            sites = list(getattr(self, '_completed_sites', ()))
        if not sites:
            return False
        ray = self._completed_detection_ray(message, source)
        if ray is None:
            return False
        _, direction, observer = ray
        position = tuple(getattr(observer, axis) for axis in 'xyz')
        translation = self.config.get('verify_retry_translation', 2.0)
        angular_limit = self.config.get(
            'verify_completed_bearing_tolerance', COMPLETED_SITE_DEFAULTS[
                'verify_completed_bearing_tolerance'])
        for _, previous_observer, previous_direction, previous_source in sites:
            if math.dist(position, previous_observer) + 1e-6 >= translation:
                continue
            if source == 'fire':
                # A previous negative smoke inspection cannot veto a newly
                # confirmed fire, even if both share the same image bearing.
                if previous_source != 'fire':
                    continue
                cosine = max(-1.0, min(1.0, sum(
                    left * right for left, right in zip(direction, previous_direction))))
                if math.acos(cosine) <= angular_limit:
                    return True
            else:
                horizontal = math.hypot(*direction[:2])
                previous_horizontal = math.hypot(*previous_direction[:2])
                if min(horizontal, previous_horizontal) <= 1e-6:
                    continue
                bearing = math.atan2(direction[1], direction[0])
                previous_bearing = math.atan2(previous_direction[1], previous_direction[0])
                difference = abs(math.atan2(math.sin(bearing - previous_bearing),
                                            math.cos(bearing - previous_bearing)))
                if difference > angular_limit:
                    continue
                # Smoke may rise above a completed fire. A lower, unrelated
                # selected ray must remain eligible for another inspection.
                if (previous_source == 'fire'
                        and math.atan2(direction[2], horizontal)
                        < math.atan2(previous_direction[2], previous_horizontal) - angular_limit):
                    continue
                return True
        return False

    def _completed_raw_observation(self, source):
        controller = self._verify
        getter = getattr(controller, 'raw_fresh_' + source, None)
        if getter is None:
            getter = getattr(controller, 'fresh_' + source)
        return getter()

    def _completed_planning_ray(self, message, source):
        """Express an image-time camera ray in the durable cluster frame."""
        ray = self._completed_detection_ray(message, source)
        if ray is None:
            return None
        origin, direction, observer = ray
        frame = self.config.get('verify_planning_frame', self.config['local_frame'])
        if frame == self.config['local_frame']:
            return origin, direction, observer
        try:
            from kv_executive.mission_navigation import _inverse
            context = getattr(self, '_verify_report_completion_context', None) or {}
            planning_to_local = context.get('planning_to_local')
            if planning_to_local is None:
                image_transform = getattr(self, '_completed_image_frame_transform', None)
                if image_transform is not None:
                    planning_to_local = image_transform(frame, message.header.stamp)
                else:
                    transform = (getattr(self, '_completed_path_frame_transform', None)
                                 or getattr(self, '_path_frame_transform'))
                    planning_to_local = transform(frame)
            conversion = _inverse(planning_to_local)
            start = transform_pose(Pose(position=origin, orientation=quaternion_from_yaw(0.0)),
                                   conversion).position
            end = transform_pose(Pose(position=Point(**{
                axis: getattr(origin, axis) + direction[index]
                for index, axis in enumerate('xyz')}), orientation=quaternion_from_yaw(0.0)),
                conversion).position
            observer = transform_pose(Pose(position=observer,
                                           orientation=quaternion_from_yaw(0.0)),
                                      conversion).position
        except (TransformException, ValueError, AttributeError, RuntimeError):
            return None
        direction = tuple(getattr(end, axis) - getattr(start, axis) for axis in 'xyz')
        norm = math.hypot(*direction)
        if not math.isfinite(norm) or norm <= 0.0:
            return None
        return start, tuple(value / norm for value in direction), observer

    def _completed_cluster_regions(self):
        """Copy the bounded spatial registry for planning without exposing mutable records."""
        with self._data_lock:
            self._completed_clock_locked()
            return [{'site_id': entry['site_id'], 'center': deepcopy(entry['center']),
                     'radius': entry['radius']} for entry in self._completed_clusters]

    def _completed_cluster_snapshot(self):
        """Publish the complete registry, including an empty snapshot after a clock reset."""
        from kv_executive.msg import CompletedInspectionCluster, CompletedInspectionClusters
        snapshot = CompletedInspectionClusters()
        snapshot.header.frame_id = self.config.get('verify_planning_frame',
                                                   self.config['local_frame'])
        snapshot.header.stamp = self.get_clock().now().to_msg()
        snapshot.clusters = [CompletedInspectionCluster(**entry)
                             for entry in self._completed_cluster_regions()]
        return snapshot

    def _completed_independent_point(self, message, source):
        """Use an existing semantic estimate only; association never starts a ranging RPC."""
        getter = getattr(self, '_verify_candidate_planning_position', None)
        if getter is None:
            return None
        result = getter(message, source)
        uncertainty = 0.0
        if isinstance(result, dict):
            if not result.get('target_valid', False):
                return None
            uncertainty = result.get('uncertainty', math.nan)
            result = result.get('planning_point')
        frame = self.config.get('verify_planning_frame', self.config['local_frame'])
        if (result is None or result.header.frame_id != frame
                or not _finite_point(result.point) or _stamp_ns(result) <= 0
                or not math.isfinite(uncertainty) or uncertainty < 0.0
                or abs(_stamp_ns(result) - _stamp_ns(message)) / 1e9
                > self.config['verify_observation_timeout']):
            return None
        return result.point, uncertainty

    def _completed_candidate_extents(self, message, source, direction):
        """Project this component's image bounds, including camera attitude at image time."""
        if not getattr(message, 'angular_bounds_valid', False):
            return 0.0, 0.0
        bounds = [getattr(message, name, math.nan) for name in (
            'yaw_min_rad', 'yaw_max_rad', 'vertical_min_rad', 'vertical_max_rad')]
        if (not all(math.isfinite(v) and abs(v) < math.pi / 2 for v in bounds)
                or bounds[0] > bounds[1] or bounds[2] > bounds[3]):
            return 0.0, 0.0
        yaw = math.atan2(direction[1], direction[0])
        elevation = math.atan2(direction[2], math.hypot(*direction[:2]))
        horizontal = vertical = 0.0
        for left in bounds[:2]:
            for upper in bounds[2:]:
                corner = deepcopy(message)
                corner.yaw_correction_rad, corner.vertical_correction_rad = left, upper
                ray = self._completed_planning_ray(corner, source)
                if ray is None:
                    continue
                vector = ray[1]
                horizontal = max(horizontal, abs(math.remainder(
                    math.atan2(vector[1], vector[0]) - yaw, 2 * math.pi)))
                vertical = max(vertical, abs(math.atan2(
                    vector[2], math.hypot(*vector[:2])) - elevation))
        return horizontal, vertical

    @staticmethod
    def _completed_semantic_heights(cluster):
        wall = cluster['center'].point
        lows, highs = [wall.z - 2.0], [wall.z + 2.0]
        for views in cluster['views'].values():
            for view in views:
                depth = math.hypot(view['anchor'][0] - view['origin'][0],
                                   view['anchor'][1] - view['origin'][1])
                height = max(2.0, depth * math.tan(min(
                    math.radians(75.0), view['vertical'])))
                lows.append(view['anchor'][2] - height)
                highs.append(view['anchor'][2] + height)
        return min(lows), max(highs)

    def _completed_projected_overlap(self, cluster, origin, direction, extents):
        """Intersect a component's forward view with bounded semantic coverage.

        Routing cylinders are intentionally infinite in height. Recognition uses
        only the physical core and finite vertical coverage from inspected images.
        There is no maximum ray distance: an old site beyond 70 m remains known.
        """
        wall = cluster['center'].point
        normal = cluster['front_normal']
        radius = cluster['core_radius'] + cluster['uncertainty']
        cx = wall.x + cluster['core_radius'] * normal[0]
        cy = wall.y + cluster['core_radius'] * normal[1]
        dx, dy = cx - origin.x, cy - origin.y
        distance = math.hypot(dx, dy)
        margin = self.config.get('verify_completed_projection_margin', math.radians(3.0))
        delta = abs(math.remainder(math.atan2(direction[1], direction[0])
                                   - math.atan2(dy, dx), 2 * math.pi))
        coverage = math.pi if distance <= radius else math.asin(min(1.0, radius / distance))
        if delta > coverage + extents[0] + margin:
            return False
        low, high = self._completed_semantic_heights(cluster)
        near, far = max(1e-3, distance - radius), max(1e-3, distance + radius)
        angles = [math.atan2(z - origin.z, depth)
                  for z in (low, high) for depth in (near, far)]
        elevation = math.atan2(direction[2], math.hypot(*direction[:2]))
        return (min(angles) - margin <= elevation + extents[1]
                and max(angles) + margin >= elevation - extents[1])

    def _completed_saved_view_identity(self, cluster, source, origin, direction):
        """Use local inspected image coverage without pretending smoke depth fixes a fire."""
        views = cluster['views'].get(source, ())
        if not views and source == 'smoke':
            views = cluster['views'].get('fire', ())
        tolerance = self.config.get('verify_completed_bearing_tolerance', math.radians(10.0))
        for view in views:
            if math.dist(_coordinates(origin), view['origin']) >= self.config.get(
                    'verify_retry_translation', 2.0):
                continue
            previous_yaw = math.atan2(view['direction'][1], view['direction'][0])
            horizontal = abs(math.remainder(math.atan2(direction[1], direction[0])
                                            - previous_yaw, 2 * math.pi))
            vertical = abs(math.atan2(direction[2], math.hypot(*direction[:2]))
                           - math.atan2(view['direction'][2], math.hypot(*view['direction'][:2])))
            if (horizontal <= tolerance + view['horizontal']
                    and vertical <= tolerance + view['vertical']):
                return True
        return False

    def _completed_queue_followup(self, cluster, source, origin, direction):
        """One bounded follow-up per completed site/sector, independent of detector IDs."""
        wall = cluster['center'].point
        bearing = math.atan2(direction[1], direction[0])
        relative_bearing = math.remainder(
            bearing - math.atan2(wall.y - origin.y, wall.x - origin.x), 2 * math.pi)
        # Relative-to-wall sectors survive observer parallax. Absolute image or
        # world bearings would recreate an exhausted task after every side step.
        sector = round(relative_bearing / math.radians(15.0)) % 24
        key = (cluster['site_id'], source, sector)
        with self._data_lock:
            tasks = self._completed_followups
            previous = next((task for task in tasks.values()
                             if task['site_id'] == cluster['site_id']
                             and task['source'] == source
                             and abs(math.remainder(task['relative_bearing'] - relative_bearing,
                                                    2 * math.pi)) <= math.radians(20.0)), None)
            if previous is not None:
                return previous['key']
            saved = next((entry for entry in self._completed_clusters
                          if entry['site_id'] == cluster['site_id']), None)
            sectors = saved.setdefault('followup_sectors', set()) if saved is not None else set()
            if any(kind == source and min((sector - old) % 24, (old - sector) % 24) <= 1
                   for kind, old in sectors):
                return key
            sectors.add((source, sector))
            tasks[key] = {'key': key, 'site_id': cluster['site_id'], 'source': source,
                          'origin': _coordinates(origin), 'direction': tuple(direction),
                          'relative_bearing': relative_bearing,
                          'attempts': 0, 'observations': [], 'lease': False}
            while len(tasks) > 32:
                tasks.popitem(last=False)
        return key

    def _completed_followup_expire_locked(self):
        if getattr(self, '_parent_state', None) == 'Verify':
            return
        timeout = self.config.get('verify_completed_followup_lease_timeout', 60.0)
        expired = False
        for task in self._completed_followups.values():
            if task['lease'] and time.monotonic() - task['lease_started'] >= timeout:
                task['lease'] = False
                expired = True
        if expired:
            self._completed_candidate_history.clear()

    def _completed_followup_task(self):
        """Return the next unresolved direction; consumed tasks cannot recreate a loop."""
        with self._data_lock:
            self._completed_clock_locked()
            self._completed_followup_expire_locked()
            limit = self.config.get('verify_completed_followup_attempts', 2)
            if any(task['lease'] for task in self._completed_followups.values()):
                return None
            sites = {entry['site_id'] for entry in self._completed_clusters}
            eligible = [task for task in self._completed_followups.values()
                        if task['attempts'] < limit and task['site_id'] in sites]
            # A distant thermal source needs its own observation opportunity;
            # a smoke-only lease must never lend guidance to an old fire blob.
            eligible.sort(key=lambda task: task['source'] != 'fire')
            return deepcopy(eligible[0]) if eligible else None

    def _completed_followup_result(self, key, success, observation_origin=None):
        """Retain immutable initial pose and actual settled alternate viewpoints."""
        with self._data_lock:
            task = self._completed_followups.get(key)
            if task is None or task['attempts'] >= self.config.get(
                    'verify_completed_followup_attempts', 2):
                return False
            if any(entry['lease'] for entry in self._completed_followups.values()):
                return False
            task['attempts'] += 1
            point = tuple(observation_origin) if observation_origin is not None else None
            baseline = self.config.get('verify_completed_followup_baseline', 6.0)
            horizontal = math.hypot(*task['direction'][:2])

            def lateral(previous):
                return abs((point[0] - previous[0]) * task['direction'][1]
                           - (point[1] - previous[1]) * task['direction'][0]) / horizontal
            useful = (success and point is not None and len(point) == 3
                      and all(math.isfinite(value) for value in point) and horizontal > 1e-6
                      and all(lateral(previous) + 1e-6 >= baseline
                              for previous in [task['origin']] + task['observations']))
            if useful:
                task['observations'].append(point)
                task['lease'] = True
                task['lease_started'] = time.monotonic()
            return bool(useful)

    def _completed_followup_consume(self):
        """End a bounded investigation permission, retaining attempts across action retries."""
        with self._data_lock:
            for task in self._completed_followups.values():
                task['lease'] = False
            self._completed_candidate_history.clear()

    def _completed_followup_permission(self, cluster, source, direction):
        with self._data_lock:
            self._completed_followup_expire_locked()
            return any(task['lease'] and task['site_id'] == cluster['site_id']
                       and task['source'] == source
                       and _angle(direction, task['direction']) <= math.radians(30.0)
                       for task in self._completed_followups.values())

    def _completed_association_diagnostic(self, source, decision, cluster_id, reason):
        logger = getattr(self, '_report', None)
        if logger is None:
            return
        key = (source, decision, cluster_id, reason)
        now = time.monotonic()
        with self._data_lock:
            previous = self._completed_diagnostics.get(key, -math.inf)
            if now - previous < 5.0:
                return
            self._completed_diagnostics[key] = now
            if len(self._completed_diagnostics) > 64:
                self._completed_diagnostics.pop(next(iter(self._completed_diagnostics)))
        logger(f'Completed-site association: source={source}; decision={decision}; '
               f'site_id={cluster_id}; reason={reason}')

    def _completed_candidate_decision(self, message, source):
        """Require spatial separation or one bounded alternate-view investigation lease."""
        if message is None or source not in ('smoke', 'fire'):
            return 'invalid'
        age = (self.get_clock().now().nanoseconds - _stamp_ns(message)) / 1e9
        confirmed = getattr(message, 'CONFIRMED', 3 if source == 'smoke' else 2)
        if (not message.detected or message.status != confirmed
                or (source == 'smoke' and not message.background_ready)
                or not message.header.frame_id or _stamp_ns(message) <= 0
                or not -self.config['verify_future_tolerance'] <= age
                <= self.config['verify_observation_timeout']
                or not all(math.isfinite(value) and abs(value) < math.pi / 2 for value in (
                    message.yaw_correction_rad, message.vertical_correction_rad))):
            return 'invalid'
        with self._data_lock:
            self._completed_clock_locked()
            self._completed_followup_expire_locked()
            clusters = [{**{name: entry[name] for name in (
                'site_id', 'radius', 'core_radius', 'uncertainty', 'front_normal')},
                'center': deepcopy(entry.get('wall_point', entry['center'])),
                'views': {kind: [{name: deepcopy(view[name]) for name in (
                    'anchor', 'horizontal', 'vertical', 'origin', 'direction')}
                                for view in entries]
                          for kind, entries in entry['views'].items()}}
                for entry in self._completed_clusters if source != 'fire' or entry['fire_found']]
        if not clusters:
            return ('completed' if self._completed_legacy_detection(message, source)
                    else 'novel')
        ray = self._completed_planning_ray(message, source)
        if ray is None:
            self._completed_association_diagnostic(
                source, 'unknown', 0, 'image-time pose or map/camera transform unavailable')
            return 'unknown'
        origin, direction, _ = ray
        independent = self._completed_independent_point(message, source)
        extents = self._completed_candidate_extents(message, source, direction)
        overlaps = [entry for entry in clusters if self._completed_projected_overlap(
            entry, origin, direction, extents)]
        if independent is not None:
            point, uncertainty = independent
            matched = any(math.hypot(point.x - entry['center'].point.x,
                                     point.y - entry['center'].point.y)
                          <= entry['core_radius'] + entry['uncertainty'] + uncertainty
                          and self._completed_semantic_heights(entry)[0] - uncertainty
                          <= point.z <= self._completed_semantic_heights(entry)[1] + uncertainty
                          for entry in clusters)
            # Independent measured location can resolve a ray's depth ambiguity.
            ambiguous = False
        else:
            matched = any(self._completed_saved_view_identity(entry, source, origin, direction)
                          for entry in overlaps)
            ambiguous = bool(overlaps) and not matched
        for entry in overlaps:
            self._completed_queue_followup(entry, source, origin, direction)
        permission = ambiguous and any(
            self._completed_followup_permission(entry, source, direction) for entry in overlaps)
        stamp = _stamp_ns(message)
        identity = int(getattr(message, 'track_id', getattr(message, 'candidate_id', 0)))
        frame = message.header.frame_id
        with self._data_lock:
            history = self._completed_candidate_history
            candidates = [(key, entry) for key, entry in history.items()
                          if entry['source'] == source and entry['frame'] == frame
                          and (entry['identity'] == identity if identity
                               else not entry['identity'])
                          and (identity or _angle(direction, entry['direction'])
                               <= math.radians(5.0))
                          and (stamp != entry['stamp'] or _angle(direction, entry['direction'])
                               <= 1e-6)]
            if candidates:
                key, previous = min(candidates, key=lambda item:
                                    _angle(direction, item[1]['direction']))
                if stamp <= previous['stamp']:
                    return previous['decision']
                continuous = ((stamp - previous['stamp']) / 1e9
                              <= self.config['verify_observation_timeout'])
            else:
                self._completed_candidate_sequence += 1
                key = self._completed_candidate_sequence
                previous, continuous = {}, False
            # An independently positioned component retains a small uncertainty
            # hysteresis. Bearing-only nonmatches never use this to grow a zone.
            if (independent is not None and not matched and continuous
                    and previous.get('decision') == 'completed'):
                old_point = previous.get('position')
                in_band = any(math.hypot(independent[0].x - entry['center'].point.x,
                                         independent[0].y - entry['center'].point.y)
                              <= entry['core_radius'] + entry['uncertainty'] + independent[1]
                              + self.config.get('verify_completed_cluster_buffer', 5.0)
                              for entry in clusters)
                if (in_band and old_point is not None
                        and math.dist(_coordinates(independent[0]), old_point)
                        <= self.config.get('verify_completed_cluster_buffer', 5.0)):
                    matched = True
            outside = not matched and (not ambiguous or permission)
            count = ((previous.get('outside', 0) + 1 if continuous else 1) if outside else 0)
            required = self.config.get('verify_completed_outside_frames', 3)
            decision = ('completed' if matched else 'ambiguous' if ambiguous and not permission
                        else 'novel' if count >= required else 'pending')
            history[key] = {'source': source, 'frame': frame, 'identity': identity,
                            'direction': direction, 'stamp': stamp, 'outside': count,
                            'decision': decision,
                            'position': _coordinates(independent[0]) if independent else None}
            history.move_to_end(key)
            while len(history) > self.config.get('verify_completed_candidate_history', 128):
                history.popitem(last=False)
        reason = ('measured position or inspected image coverage' if matched else
                  'bounded alternate-view investigation' if permission else
                  'projected completed area overlaps this component' if ambiguous else
                  'component is spatially separated from completed coverage')
        self._completed_association_diagnostic(
            source, decision, overlaps[0]['site_id'] if overlaps else 0, reason)
        return decision

    def _completed_candidate_priority(self, message, source):
        """Prefer visibly uninspected components; uncertain depth only affects ordering.

        A synthetic anchor at the smoke-wall depth is never identity proof. Its
        projected coverage is widened with observer translation and used only
        to choose among already eligible candidates during a bounded follow-up.
        """
        if self._completed_candidate_decision(message, source) != 'novel':
            return 3, 0.0
        ray = self._completed_planning_ray(message, source)
        if ray is None:
            return 3, 0.0
        origin, direction, _ = ray
        if self._completed_independent_point(message, source) is not None:
            return 0, 0.0
        with self._data_lock:
            clusters = [{**{name: entry[name] for name in (
                'site_id', 'radius', 'core_radius', 'uncertainty', 'front_normal')},
                'center': deepcopy(entry.get('wall_point', entry['center'])),
                'views': {kind: [{name: deepcopy(view[name]) for name in (
                    'anchor', 'horizontal', 'vertical', 'origin', 'direction', 'localized')}
                                for view in entries]
                          for kind, entries in entry['views'].items()}}
                for entry in self._completed_clusters if source != 'fire' or entry['fire_found']]
        extents = self._completed_candidate_extents(message, source, direction)
        overlaps = [entry for entry in clusters if self._completed_projected_overlap(
            entry, origin, direction, extents)]
        if not overlaps:
            return 0, 0.0
        yaw = math.atan2(direction[1], direction[0])
        elevation = math.atan2(direction[2], math.hypot(*direction[:2]))
        margin = self.config.get('verify_completed_projection_margin', math.radians(3.0))
        scores = []
        for cluster in overlaps:
            views = cluster['views'].get(source, ())
            if not views and source == 'smoke':
                views = cluster['views'].get('fire', ())
            for view in views:
                relative = tuple(a - b for a, b in zip(view['anchor'], _coordinates(origin)))
                depth = math.hypot(*relative)
                if depth <= 1e-6 or sum(a * b for a, b in zip(relative, direction)) <= 0:
                    continue
                uncertainty = cluster['uncertainty']
                if not view['localized']:
                    uncertainty += math.dist(_coordinates(origin), view['origin'])
                padding = margin + math.atan2(uncertainty, depth)
                horizontal = abs(math.remainder(
                    yaw - math.atan2(relative[1], relative[0]), 2 * math.pi))
                vertical = abs(elevation - math.atan2(relative[2], math.hypot(*relative[:2])))
                scores.append(max(horizontal / (padding + extents[0] + view['horizontal']),
                                  vertical / (padding + extents[1] + view['vertical'])))
        # Large separation is a selection preference, never a permanent veto.
        separation = min(scores, default=2.0)
        return (1 if separation > 1.0 else 2), -min(separation, 100.0)

    def _verify_is_completed_detection(self, message, source):
        """Identify completed sources without reading filtered controller observations."""
        return self._completed_candidate_decision(message, source) == 'completed'

    def _verify_candidate_is_eligible(self, message, source):
        """Missing association with saved clusters waits for a fresh source-time transform."""
        return self._completed_candidate_decision(message, source) == 'novel'

    def _verify_defer_viewpoint(self):
        """Remember an inconclusive goal's viewpoint without inventing a source range."""
        if not hasattr(self, '_deferred_viewpoints'):
            return False
        with self._goal_lock:
            goal = self._goal_handle
            if goal is None or not goal.is_active:
                return False
            goal_id = deepcopy(goal.goal_id)
        key = tuple(goal_id.uuid)
        entries = []
        for source, message in (
                ('smoke', self._verify.fresh_smoke()), ('fire', self._verify.fresh_fire())):
            ray = self._completed_detection_ray(message, source)
            if ray is None:
                continue
            _, direction, observer = ray
            if math.hypot(*direction[:2]) <= 1e-6:
                continue
            entries.append((key, tuple(getattr(observer, axis) for axis in 'xyz'),
                            math.atan2(direction[1], direction[0])))
        if not entries:
            return False
        with self._data_lock:
            self._deferred_viewpoints.extend(entries)
        if self._verify_request_is_deferred(goal_id):
            with self._goal_lock:
                goal = self._goal_handle
                if (goal is not None and tuple(goal.goal_id.uuid) == key
                        and self._requested_mission_state == 'Verify'
                        and getattr(self, '_requested_verify_automatic', False)):
                    self._requested_mission_state = None
                    self._requested_verify_automatic = False
        return True

    def _verify_request_is_deferred(self, goal_id):
        """Let a new bearing or translated observer interrupt; yaw/time alone cannot."""
        with self._data_lock:
            deferred = [entry for entry in getattr(self, '_deferred_viewpoints', ())
                        if entry[0] == tuple(goal_id.uuid)]
        if not deferred:
            return False
        visible = [(source, message) for source, message in (
            ('smoke', self._verify.fresh_smoke()), ('fire', self._verify.fresh_fire()))
            if message is not None]
        if not visible:
            return False
        distance = self.config.get('verify_retry_translation', 2.0)
        tolerance = self.config.get('verify_retry_bearing_tolerance', math.radians(15.0))
        for source, message in visible:
            ray = self._completed_detection_ray(message, source)
            if ray is None:
                return False  # Unknown association must not suppress a new source.
            _, direction, observer = ray
            if math.hypot(*direction[:2]) <= 1e-6:
                return False
            position = tuple(getattr(observer, axis) for axis in 'xyz')
            bearing = math.atan2(direction[1], direction[0])
            matched = any(
                math.dist(position, origin) + 1e-6 < distance
                and abs(math.atan2(math.sin(bearing - previous),
                                   math.cos(bearing - previous))) <= tolerance
                for _, origin, previous in deferred)
            if not matched and not self._verify_is_completed_detection(message, source):
                return False
        return True

    def _verify_request_is_completed(self):
        """Veto only when every currently confirmed source is already completed."""
        if (not getattr(self, '_completed_sites', None)
                and not getattr(self, '_completed_clusters', None)):
            return False
        candidates = getattr(self._verify, 'raw_candidates', None)
        sources = [(source, message) for source in ('smoke', 'fire')
                   for message in (candidates(source) if candidates else
                                   [self._completed_raw_observation(source)])]
        visible = [(source, message) for source, message in sources if message is not None]
        decisions = [(source, self._completed_candidate_decision(message, source))
                     for source, message in visible]
        if not decisions or any(decision == 'novel' for _, decision in decisions):
            return False
        # Camera frames can precede their bracketing odometry. Such a newest
        # frame cannot erase a still-fresh earlier association, or qualify alone.
        waiting = {source for source, decision in decisions
                   if decision in ('unknown', 'pending')}
        for source in waiting:
            getter = getattr(self._verify, 'fresh_' + source, None)
            accepted = getter() if getter is not None else None
            if accepted is not None and self._verify_candidate_is_eligible(accepted, source):
                return False
        return True

    def _completed_report_point(self, report, source):
        point = getattr(report, source + '_planning_position', None)
        uncertainty = getattr(report, source + '_uncertainty_m', math.nan)
        frame = self.config.get('verify_planning_frame', self.config['local_frame'])
        if (not getattr(report, source + '_location_valid', False) or point is None
                or point.header.frame_id != frame or _stamp_ns(point) <= 0
                or _stamp_ns(point) > self.get_clock().now().nanoseconds
                + self.config['verify_future_tolerance'] * 1e9
                or not _finite_point(point.point) or not math.isfinite(uncertainty)
                or not 0.0 <= uncertainty <= self.config.get(
                    'verify_inspection_max_uncertainty', 2.0)):
            return None
        return point, uncertainty

    def _completed_report_match(self, report):
        """Match authoritative confirmed wall coordinates before any report/ID publication."""
        measured = self._completed_report_point(report, 'smoke')
        if measured is None or not getattr(report, 'smoke_wall_confirmed', False):
            return None
        wall, _ = measured
        with self._data_lock:
            self._completed_clock_locked()
            matches = [(math.hypot(entry['wall_point'].point.x - wall.point.x,
                                   entry['wall_point'].point.y - wall.point.y), entry)
                       for entry in self._completed_clusters]
            matches = [(distance, entry) for distance, entry in matches
                       if distance <= entry['core_radius']]
            if not matches:
                return None
            entry = min(matches, key=lambda pair: pair[0])[1]
            return deepcopy({name: entry[name] for name in (
                'site_id', 'fire_found', 'wall_point', 'core_radius', 'uncertainty')})

    def _completed_report_views(self, report, center, message, source):
        """Keep bounded coverage for selected and nearby confirmed image components."""
        result = {'smoke': [], 'fire': []}
        context = getattr(self, '_verify_report_completion_context', None)
        candidates = getattr(self._verify, 'raw_candidates', None)
        for kind in ('smoke', 'fire'):
            selected = (context.get(kind) if context is not None
                        else self._completed_raw_observation(kind))
            if selected is None and kind == source:
                selected = message
            observations = [selected] if selected is not None else []
            if candidates is not None:
                observations.extend(candidate for candidate in candidates(kind)[:16]
                                    if candidate != selected)
            for observation in observations:
                if (observation == selected and kind == 'fire'
                        and int(getattr(report, 'track_id', 0))
                        and int(report.track_id) != int(getattr(observation, 'track_id', 0))):
                    continue
                ray = self._completed_planning_ray(observation, kind)
                if ray is None:
                    continue
                origin, direction, observer = ray
                relative = tuple(a - b for a, b in zip(
                    _coordinates(center.point), _coordinates(origin)))
                depth = math.hypot(*relative)
                horizontal, vertical = self._completed_candidate_extents(
                    observation, kind, direction)
                if (depth <= 1e-6 or _angle(relative, direction)
                        > math.radians(30.0) + min(horizontal, math.radians(30.0))):
                    continue  # Do not attach an unrelated side plume to this report.
                known = self._completed_report_point(report, kind)
                localized = (known is not None and kind == 'fire'
                             and observation == selected)
                anchor = (_coordinates(known[0].point) if localized else
                          tuple(value + depth * component for value, component
                                in zip(_coordinates(origin), direction)))
                # A finite anchor helps project image coverage after movement;
                # only independently localized coordinates establish identity.
                result[kind].append({
                    'message': deepcopy(observation), 'origin': _coordinates(origin),
                    'direction': direction, 'observer': _coordinates(observer), 'anchor': anchor,
                    'localized': localized, 'horizontal': min(math.radians(30.0), horizontal),
                    'vertical': min(math.radians(30.0), vertical),
                })
                if len(result[kind]) >= 4:
                    break
        return result

    def _completed_wall_direction(self, report, wall):
        """Use the original measurement observer to place the whole circle behind the wall."""
        context = getattr(self, '_verify_report_completion_context', None) or {}
        observer = context.get('wall_observer')
        if 'wall_observer' not in context:
            viewpoint = getattr(report, 'observer_planning_pose', None)
            if viewpoint is None:
                return None
            observer = PointStamped(header=deepcopy(viewpoint.header),
                                    point=deepcopy(viewpoint.pose.position))
        if (observer is None or observer.header.frame_id != wall.header.frame_id
                or _stamp_ns(observer) <= 0
                or _stamp_ns(observer) > self.get_clock().now().nanoseconds
                + self.config['verify_future_tolerance'] * 1e9
                or not _finite_point(observer.point)):
            return None
        dx, dy = wall.point.x - observer.point.x, wall.point.y - observer.point.y
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance) or distance <= 1e-6:
            return None
        return deepcopy(observer), (dx / distance, dy / distance)

    def _completed_save_cluster(self, report, message, source):
        self._completed_cluster_failure_reason = ''
        measured = self._completed_report_point(report, 'smoke')
        if measured is None or not getattr(report, 'smoke_wall_confirmed', False):
            return False
        wall, uncertainty = measured
        core = self.config.get('verify_completed_cluster_radius', 20.0)
        buffer = self.config.get('verify_completed_cluster_buffer', 5.0)
        if (not all(math.isfinite(value) and value >= 0.0 for value in (core, buffer))
                or core <= 0.0):
            raise ValueError('Completed cluster core and buffer must be finite and nonnegative')
        direction = self._completed_wall_direction(report, wall)
        if direction is None:
            self._completed_cluster_failure_reason = (
                'retained smoke wall has no valid horizontal measurement-observer direction')
            logger = getattr(self, '_report', None)
            if logger is not None:
                logger('Verify: completed routing zone unavailable: '
                       + self._completed_cluster_failure_reason)
            return False
        observer, normal = direction
        center = deepcopy(wall)
        center.point.x += (core + buffer) * normal[0]
        center.point.y += (core + buffer) * normal[1]
        views = self._completed_report_views(report, wall, message, source)
        entry = {'site_id': int(report.site_id), 'center': center, 'wall_point': deepcopy(wall),
                 'wall_observer': observer, 'front_normal': normal,
                 'uncertainty': uncertainty, 'core_radius': core, 'radius': core + buffer,
                 'fire_found': report.status == 'fire_found', 'views': views,
                 'reports': [{'site_id': int(report.site_id), 'status': report.status,
                              'point': deepcopy(wall)}]}
        with self._data_lock:
            self._completed_clock_locked()
            previous = next((old for old in self._completed_clusters
                             if math.hypot(old['wall_point'].point.x - wall.point.x,
                                           old['wall_point'].point.y - wall.point.y)
                             <= old['core_radius']), None)
            if previous is None:
                self._completed_clusters.append(entry)
            else:
                # Never move or expand a historical center by chaining nearby
                # reports; only enrich that fixed cluster's source contexts.
                previous['fire_found'] |= entry['fire_found']
                previous['reports'] = (previous['reports'] + entry['reports'])[-2:]
                for kind in views:
                    previous['views'][kind] = (previous['views'][kind] + views[kind])[-4:]
            self._completed_candidate_history.clear()
            self._completed_clusters_version += 1
        logger = getattr(self, '_report', None)
        if previous is None and logger is not None:
            logger(f'Verify: completed smoke routing zone site_id={entry["site_id"]}; '
                   f'wall=({wall.point.x:.2f}, {wall.point.y:.2f}, {wall.point.z:.2f}) m; '
                   f'center=({center.point.x:.2f}, {center.point.y:.2f}, '
                   f'{center.point.z:.2f}) m; radius={entry["radius"]:.2f} m; '
                   f'front_normal=({normal[0]:+.3f}, {normal[1]:+.3f}); '
                   'routing footprint starts at the measured wall and extends behind it')
        return True

    def _verify_complete_site(self, report, message=None, source=None):
        """Commit the retained measured footprint even if optional image association is absent."""
        if report.status not in ('fire_found', 'fire_not_found'):
            return False
        expected_source = 'fire' if report.status == 'fire_found' else 'smoke'
        if source is not None and source != expected_source:
            return False
        source = expected_source
        saved = self._completed_save_cluster(report, message, source)
        if not saved and self._completed_cluster_failure_reason:
            return False
        if message is None:
            message = self._completed_raw_observation(source)
        if source == 'fire' and message is not None:
            report_track = int(getattr(report, 'track_id', 0))
            if report_track > 0 and report_track != int(getattr(message, 'track_id', 0)):
                return saved
        if not saved:
            # Compatibility for observation-only reports without a measured
            # smoke coordinate: retain the existing conservative bearing gate.
            ray = self._completed_detection_ray(message, source)
            if ray is None:
                return False
            _, direction, observer = ray
            point = _coordinates(observer)
            with self._data_lock:
                self._completed_sites.append((int(report.site_id), point, direction, source))
        if self._verify_request_is_completed():
            with self._goal_lock:
                if (self._requested_mission_state == 'Verify'
                        and getattr(self, '_requested_verify_automatic', False)):
                    self._requested_mission_state = None
                    self._requested_verify_automatic = False
        return True
