"""Associate completed sources and defer failed viewpoints until exploration advances."""

from collections import deque
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
}


def _stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class CompletedSitesMixin:
    """Keep bounded observer/bearing evidence, independent of detector track IDs."""

    def _initialize_completed_sites(self):
        self._completed_sites = deque(maxlen=100)
        self._inspection_odometry_history = deque(maxlen=512)
        # An inconclusive inspection is not a completed source location. Keep
        # only its observer and bearing so a sweep can actually make progress.
        self._deferred_viewpoints = deque(maxlen=32)

    def _completed_odometry_locked(self, message):
        """Append validated local poses while the caller holds the telemetry lock."""
        history = getattr(self, '_inspection_odometry_history', None)
        if history is None or message.header.frame_id != self.config['local_frame']:
            return
        stamp_ns = _stamp_ns(message)
        if stamp_ns <= 0:
            return
        try:
            pose = interpolate_pose(message.pose.pose, message.pose.pose, 0.0)
        except (ValueError, AttributeError):
            return
        if history and stamp_ns < history[-1][0]:
            # A ROS clock reset or reordered telemetry cannot interpolate across
            # epochs. Losing association permits a new inspection, never a veto.
            history.clear()
            if getattr(self, '_deferred_viewpoints', None):
                self._deferred_viewpoints.clear()
            if getattr(self, '_completed_sites', None):
                self._completed_sites.clear()
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

    def _verify_is_completed_detection(self, message, source):
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
        if not getattr(self, '_completed_sites', None):
            return False
        sources = [('smoke', self._verify.fresh_smoke()), ('fire', self._verify.fresh_fire())]
        visible = [(source, message) for source, message in sources if message is not None]
        return bool(visible) and all(
            self._verify_is_completed_detection(message, source) for source, message in visible)

    def _verify_complete_site(self, report, message=None, source=None):
        """Remember completed image evidence without estimating a fire distance."""
        if report.status not in ('fire_found', 'fire_not_found'):
            return False
        expected_source = 'fire' if report.status == 'fire_found' else 'smoke'
        if source is not None and source != expected_source:
            return False
        source = expected_source
        if message is None:
            message = (self._verify.fresh_fire() if source == 'fire'
                       else self._verify.fresh_smoke())
        if source == 'fire' and message is not None:
            report_track = int(getattr(report, 'track_id', 0))
            if report_track > 0 and report_track != int(getattr(message, 'track_id', 0)):
                return False
        ray = self._completed_detection_ray(message, source)
        if ray is None:
            return False  # Missing association permits another inspection.
        _, direction, observer = ray
        point = tuple(getattr(observer, axis) for axis in 'xyz')
        with self._data_lock:
            self._completed_sites.append((int(report.site_id), point, direction, source))
        if self._verify_request_is_completed():
            with self._goal_lock:
                if (self._requested_mission_state == 'Verify'
                        and getattr(self, '_requested_verify_automatic', False)):
                    self._requested_mission_state = None
                    self._requested_verify_automatic = False
        return True
