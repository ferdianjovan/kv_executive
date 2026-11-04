"""Associate completed sources and defer failed viewpoints until exploration advances."""

from collections import deque, OrderedDict
from copy import deepcopy
import math

from geometry_msgs.msg import Point, Pose, Transform, Vector3
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

    def _completed_view_matches(self, cluster, source, origin, direction):
        """Require a narrow saved view even when the camera is inside the spatial buffer."""
        views = cluster['views'].get(source, ())
        if not views and source == 'smoke':
            views = cluster['views'].get('fire', ())
        if not views:
            # A valid historical wall can be saved even when a camera transform
            # is temporarily absent. Its point remains a narrow fallback, not
            # an all-direction exclusion sphere around a nearby observer.
            views = [{'anchor': _coordinates(cluster['center'].point),
                      'horizontal': 0.0, 'vertical': 0.0}]
        tolerance = self.config.get('verify_completed_bearing_tolerance', math.radians(10.0))
        for view in views:
            relative = tuple(a - b for a, b in zip(view['anchor'], _coordinates(origin)))
            length = math.hypot(*relative)
            if length <= 1e-6:
                continue
            forward = sum(a * b for a, b in zip(relative, direction))
            if forward <= 0.0:
                continue
            # A finite spatial miss protects distinct distant bearings. The
            # angle test is also mandatory: the camera can itself be inside
            # the 25 m buffer, where ray/sphere intersection has no selectivity.
            miss = math.sqrt(max(0.0, length * length - forward * forward))
            if miss > cluster['radius'] + cluster['uncertainty']:
                continue
            horizontal = abs(math.atan2(
                math.sin(math.atan2(direction[1], direction[0])
                         - math.atan2(relative[1], relative[0])),
                math.cos(math.atan2(direction[1], direction[0])
                         - math.atan2(relative[1], relative[0]))))
            vertical = abs(math.atan2(direction[2], math.hypot(*direction[:2]))
                           - math.atan2(relative[2], math.hypot(*relative[:2])))
            if (horizontal <= tolerance + view['horizontal']
                    and vertical <= tolerance + view['vertical']):
                return True
        return False

    def _completed_candidate_decision(self, message, source):
        """Classify one source image once; three supported outside images open a new target."""
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
            # Candidate arrays can contain 16 components per frame. Matching
            # copies only small geometry, never every retained detector message
            # and report context on this frequent path.
            clusters = [{**{name: entry[name] for name in (
                'radius', 'core_radius', 'uncertainty')},
                'center': deepcopy(entry['center']),
                'views': {kind: [{name: view[name] for name in (
                    'anchor', 'horizontal', 'vertical')} for view in entries]
                          for kind, entries in entry['views'].items()}}
                for entry in self._completed_clusters if source != 'fire' or entry['fire_found']]
        if not clusters:
            return ('completed' if self._completed_legacy_detection(message, source)
                    else 'novel')
        ray = self._completed_planning_ray(message, source)
        if ray is None:
            return 'unknown'  # Missing association permits a fresh re-observation.
        origin, direction, _ = ray
        independent = self._completed_independent_point(message, source)
        if independent is not None:
            point, uncertainty = independent
            matched = any(math.hypot(point.x - entry['center'].point.x,
                                     point.y - entry['center'].point.y)
                          <= entry['radius'] + entry['uncertainty'] + uncertainty
                          for entry in clusters)
        else:
            matched = any(self._completed_view_matches(entry, source, origin, direction)
                          for entry in clusters)
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
            if (not matched and continuous and previous.get('decision') == 'completed'
                    and _angle(direction, previous['direction']) <= math.radians(5.0)):
                old_point = previous.get('position')
                jump = (independent is not None and old_point is not None
                        and math.hypot(independent[0].x - old_point[0],
                                       independent[0].y - old_point[1])
                        > self.config.get('verify_completed_cluster_radius', 20.0))
                clearly_far = (independent is not None and all(
                    math.hypot(independent[0].x - entry['center'].point.x,
                               independent[0].y - entry['center'].point.y)
                    > entry['radius'] + entry['uncertainty'] + independent[1]
                    + entry['core_radius'] for entry in clusters))
                if not jump and not clearly_far:
                    # A continuous previously completed component does not
                    # become a new site through a small border/parallax shift.
                    # This changes association only, never the fixed footprint.
                    matched = True
            count = (0 if matched else previous.get('outside', 0) + 1 if continuous else 1)
            required = self.config.get('verify_completed_outside_frames', 3)
            decision = 'completed' if matched else 'novel' if count >= required else 'pending'
            history[key] = {'source': source, 'frame': frame, 'identity': identity,
                            'direction': direction, 'stamp': stamp, 'outside': count,
                            'decision': decision,
                            'position': _coordinates(independent[0]) if independent else None}
            history.move_to_end(key)
            limit = self.config.get('verify_completed_candidate_history', 128)
            while len(history) > limit:
                history.popitem(last=False)
        return decision

    def _verify_is_completed_detection(self, message, source):
        """Identify completed sources without reading filtered controller observations."""
        return self._completed_candidate_decision(message, source) == 'completed'

    def _verify_candidate_is_eligible(self, message, source):
        """Withhold completed/border-pending candidates while allowing unknown re-observation."""
        return self._completed_candidate_decision(message, source) in ('novel', 'unknown')

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
        return bool(visible) and all(
            not self._verify_candidate_is_eligible(message, source) for source, message in visible)

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

    def _completed_report_views(self, report, center, message, source):
        """Keep both image-time contexts; view anchors are association aids, not source fixes."""
        result = {'smoke': [], 'fire': []}
        context = getattr(self, '_verify_report_completion_context', None)
        for kind in ('smoke', 'fire'):
            observation = (context.get(kind) if context is not None
                           else self._completed_raw_observation(kind))
            if observation is None and kind == source:
                observation = message
            if observation is None:
                continue
            if (kind == 'fire' and int(getattr(report, 'track_id', 0))
                    and int(getattr(report, 'track_id', 0))
                    != int(getattr(observation, 'track_id', 0))):
                continue
            ray = self._completed_planning_ray(observation, kind)
            if ray is None:
                continue
            origin, direction, observer = ray
            known = self._completed_report_point(report, kind)
            if known is not None and kind == 'fire':
                anchor = _coordinates(known[0].point)
            else:
                # A saved image ray needs a finite reference depth for later
                # parallax. Use the measured cluster depth only as a narrow
                # association guide; never publish it as a localized fire.
                depth = math.dist(_coordinates(origin), _coordinates(center.point))
                relative = tuple(a - b for a, b in zip(
                    _coordinates(center.point), _coordinates(origin)))
                if depth <= 1e-6 or _angle(relative, direction) > math.radians(30.0):
                    continue  # Do not attach an unrelated side plume to this report.
                anchor = tuple(value + depth * component
                               for value, component in zip(_coordinates(origin), direction))
            horizontal = vertical = 0.0
            if getattr(observation, 'angular_bounds_valid', False):
                bounds = [getattr(observation, name, math.nan) for name in (
                    'yaw_min_rad', 'yaw_max_rad', 'vertical_min_rad', 'vertical_max_rad')]
                if all(math.isfinite(value) for value in bounds):
                    horizontal = min(math.radians(30.0), max(
                        abs(bounds[0] - observation.yaw_correction_rad),
                        abs(bounds[1] - observation.yaw_correction_rad)))
                    vertical = min(math.radians(30.0), max(
                        abs(bounds[2] - observation.vertical_correction_rad),
                        abs(bounds[3] - observation.vertical_correction_rad)))
            result[kind].append({
                'message': deepcopy(observation), 'origin': _coordinates(origin),
                'direction': direction, 'observer': _coordinates(observer), 'anchor': anchor,
                'horizontal': horizontal, 'vertical': vertical,
            })
        return result

    def _completed_save_cluster(self, report, message, source):
        measured = self._completed_report_point(report, 'smoke')
        if measured is None or not getattr(report, 'smoke_wall_confirmed', False):
            return False
        center, uncertainty = measured
        core = self.config.get('verify_completed_cluster_radius', 20.0)
        buffer = self.config.get('verify_completed_cluster_buffer', 5.0)
        if (not all(math.isfinite(value) and value >= 0.0 for value in (core, buffer))
                or core <= 0.0):
            raise ValueError('Completed cluster core and buffer must be finite and nonnegative')
        views = self._completed_report_views(report, center, message, source)
        entry = {'site_id': int(report.site_id), 'center': deepcopy(center),
                 'uncertainty': uncertainty, 'core_radius': core, 'radius': core + buffer,
                 'fire_found': report.status == 'fire_found', 'views': views,
                 'reports': [{'site_id': int(report.site_id), 'status': report.status,
                              'point': deepcopy(center)}]}
        with self._data_lock:
            self._completed_clock_locked()
            previous = next((old for old in self._completed_clusters
                             if math.hypot(old['center'].point.x - center.point.x,
                                           old['center'].point.y - center.point.y)
                             <= old['core_radius']), None)
            if previous is None:
                self._completed_clusters.append(entry)
            else:
                # Never move or expand a historical center by chaining nearby
                # reports; only enrich that fixed cluster's source contexts.
                previous['site_id'] = entry['site_id']
                previous['fire_found'] |= entry['fire_found']
                previous['reports'] = (previous['reports'] + entry['reports'])[-2:]
                for kind in views:
                    previous['views'][kind] = (previous['views'][kind] + views[kind])[-4:]
            self._completed_candidate_history.clear()
            self._completed_clusters_version += 1
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
