"""Track image-time source bearings without replaying relative camera errors."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Point, Pose
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, transform_pose, yaw_from_quaternion,
)
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


def _stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _horizontal_centering_turn(ray, camera_forward, camera_left):
    """Solve navigation-Z yaw that makes the fixed ray's camera-left component zero."""
    u, v, z = ray
    left_x, left_y, left_z = camera_left
    a = u * left_x + v * left_y
    b = v * left_x - u * left_y
    c = z * left_z
    radius = math.hypot(a, b)
    if radius < 1e-9 or abs(c) > radius + 1e-9:
        return None
    phase = math.atan2(b, a)
    offset = math.acos(max(-1.0, min(1.0, -c / radius)))
    candidates = []
    for turn in (phase + offset, phase - offset):
        turn = math.remainder(turn, math.tau)
        cosine, sine = math.cos(turn), math.sin(turn)
        forward = (cosine * camera_forward[0] - sine * camera_forward[1],
                   sine * camera_forward[0] + cosine * camera_forward[1], camera_forward[2])
        if sum(first * second for first, second in zip(ray, forward)) > 1e-9:
            candidates.append(turn)
    return min(candidates, key=abs) if candidates else None


class InspectionHeadingMixin:
    """Keep source selection and yaw commands consistent across short legs."""

    def _verify_reset_heading(self):
        self._verify_heading_state = {
            'attempted': {}, 'retry_at': {}, 'geometry': {}, 'sources': {}, 'last_log': None}

    def _heading_state(self):
        if not hasattr(self, '_verify_heading_state'):
            self._verify_reset_heading()
        return self._verify_heading_state

    def _heading_clock_running(self):
        """Expire frozen-clock guidance without shortening a progressing slow clock's grace."""
        state = self._heading_state()
        now_ns, wall = self.get_clock().now().nanoseconds, time.monotonic()
        previous = state.get('clock_ns')
        if previous is None or now_ns > previous:
            state['clock_ns'], state['clock_progress_at'] = now_ns, wall
        elif now_ns < previous:
            return False
        return (now_ns > 0 and wall - state['clock_progress_at']
                <= self.config.get('verify_heading_clock_timeout', 5.0))

    def _verify_image_bearing(self, message):
        """Project a camera ray at image acquisition time into local horizontal yaw."""
        stamp = _stamp_ns(message)
        now = self.get_clock().now().nanoseconds
        angles = (message.yaw_correction_rad, message.vertical_correction_rad)
        if (stamp <= 0 or now <= 0 or not message.header.frame_id
                or not all(math.isfinite(angle) and abs(angle) < math.pi / 2 for angle in angles)
                or not -self.config.get('verify_future_tolerance', 0.25)
                <= (now - stamp) / 1e9 <= self.config.get('verify_observation_timeout', 2.0)):
            return None
        frame = self.config['verify_planning_frame']
        try:
            source = self._tf_buffer.lookup_transform(
                frame, message.header.frame_id, Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.config.get('transform_timeout', 0.5)))
            origin = transform_pose(Pose(orientation=quaternion_from_yaw(0.0)), source.transform)
            endpoint = transform_pose(Pose(
                position=Point(x=1.0, y=math.tan(angles[0]), z=math.tan(angles[1])),
                orientation=quaternion_from_yaw(0.0)), source.transform)
            forward = transform_pose(Pose(position=Point(x=1.0),
                                          orientation=quaternion_from_yaw(0.0)), source.transform)
            left = transform_pose(Pose(position=Point(y=1.0),
                                       orientation=quaternion_from_yaw(0.0)), source.transform)
            conversion = self._path_frame_transform(frame)
            origin = transform_pose(origin, conversion).position
            endpoint = transform_pose(endpoint, conversion).position
            forward = transform_pose(forward, conversion).position
            left = transform_pose(left, conversion).position
        except (TransformException, ValueError) as error:
            self._heading_log_unavailable(message, str(error))
            return None
        except RuntimeError as error:
            # Frame-alignment helpers wrap TF failures in MissionError. Actual
            # telemetry/control faults keep their normal failure handling.
            if not isinstance(error.__cause__, TransformException):
                raise
            self._heading_log_unavailable(message, str(error))
            return None
        dx, dy = endpoint.x - origin.x, endpoint.y - origin.y
        if not all(math.isfinite(value) for value in (dx, dy)) or math.hypot(dx, dy) < 1e-6:
            self._heading_log_unavailable(message, 'camera ray has no horizontal bearing')
            return None
        bearing = math.atan2(dy, dx)
        self._heading_state()['geometry'][message.header.frame_id] = {
            'stamp': stamp,
            'ray': tuple(getattr(endpoint, axis) - getattr(origin, axis) for axis in 'xyz'),
            'camera_forward': tuple(
                getattr(forward, axis) - getattr(origin, axis) for axis in 'xyz'),
            'camera_left': tuple(getattr(left, axis) - getattr(origin, axis) for axis in 'xyz')}
        return bearing

    def _heading_log_unavailable(self, message, reason):
        state = self._heading_state()
        now = time.monotonic()
        if now - state.get('unavailable_at', -math.inf) >= 5.0:
            self._report('Verify: image-time heading unavailable for '
                         f'{message.header.frame_id}: {reason}; retaining absolute heading')
            state['unavailable_at'] = now

    def _heading_observation(self, source, message, *, deadline=None):
        """Consume each source frame once; retain only its fixed-frame bearing."""
        if not self._inspection_detection_valid(message, source):
            return None
        if not self._heading_clock_running():
            return None
        state = self._heading_state()
        key = (source, message.header.frame_id)
        stamp = _stamp_ns(message)
        previous = state['sources'].get(key)
        if previous is not None and previous['stamp'] == stamp:
            return previous
        attempted = state['attempted'].get(key, 0)
        if stamp < attempted:
            return None
        if stamp == attempted and time.monotonic() < state['retry_at'].get(key, math.inf):
            return None
        state['attempted'][key] = stamp
        # A failed TF lookup has consumed no camera error. Retry it slowly
        # while the source image is fresh, allowing asynchronous TF to arrive.
        state['retry_at'][key] = time.monotonic() + 0.25

        def unavailable():
            state['retry_at'][key] = time.monotonic() + 0.25
            return None

        bearing = self._verify_image_bearing(message)
        if (bearing is None or not self._inspection_detection_valid(message, source)
                or (deadline is not None and time.monotonic() >= deadline)):
            return unavailable()
        geometry = state['geometry'][message.header.frame_id]
        turn = _horizontal_centering_turn(
            geometry['ray'], geometry['camera_forward'], geometry['camera_left'])
        if turn is None:
            self._heading_log_unavailable(
                message, 'navigation yaw cannot center this image ray in front of the camera')
            return unavailable()
        try:
            body = self._tf_buffer.lookup_transform(
                self.config['verify_planning_frame'],
                self.config.get('robot_base_frame', 'kv_drone/base_link'),
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=max(0.0, min(
                    self.config.get('transform_timeout', 0.5),
                    deadline - time.monotonic() if deadline is not None else math.inf))))
            body_pose = transform_pose(Pose(orientation=body.transform.rotation),
                                       self._path_frame_transform(
                                           self.config['verify_planning_frame']))
        except (TransformException, ValueError) as error:
            self._heading_log_unavailable(message, str(error))
            return unavailable()
        except RuntimeError as error:
            if not isinstance(error.__cause__, TransformException):
                raise
            self._heading_log_unavailable(message, str(error))
            return unavailable()
        if not self._inspection_detection_valid(message, source):
            return unavailable()
        body_target = math.remainder(yaw_from_quaternion(body_pose.orientation) + turn, math.tau)
        observation = {'source': source, 'frame': message.header.frame_id, 'stamp': stamp,
                       'message': deepcopy(message),
                       'bearing': bearing, 'body_target': body_target,
                       'raw_error': message.yaw_correction_rad,
                       'received_at': time.monotonic()}
        state['sources'][key] = observation
        if source == 'fire':
            state['fire'] = observation
        return observation

    def _heading_fire_memory(self):
        fire = self._heading_state().get('fire')
        if fire is None:
            return None
        eligible = getattr(getattr(self, '_verify', None), '_candidate_eligible', None)
        if eligible is not None and fire.get('message') is not None and not eligible(
                fire['message'], 'fire'):
            return None
        age = (self.get_clock().now().nanoseconds - fire['stamp']) / 1e9
        if (not -self.config.get('verify_future_tolerance', 0.25) <= age
                <= self.config.get('verify_fire_heading_grace', 2.0)
                or not self._heading_clock_running()):
            return None
        return fire

    def _heading_selected(self):
        controller = getattr(self, '_verify', None)
        if controller is None:
            return None
        fire = controller.stable_fire()
        if self._inspection_detection_valid(fire, 'fire'):
            # A visible fire with missing historical TF does not transfer yaw
            # control to the plume or turn an old camera error into a new ray.
            return self._heading_observation('fire', fire) or self._heading_fire_memory()
        remembered = self._heading_fire_memory()
        if remembered is not None:
            return remembered
        return self._heading_observation('smoke', controller.fresh_smoke())

    def _verify_live_source_heading(self, fallback):
        """Slew toward one image-time bearing, using actual pose rather than future vertices."""
        if not math.isfinite(fallback):
            raise ValueError('Heading fallback must be finite')
        output = fallback
        previous_ns = None
        has_target = False

        def heading(_future_pose):
            nonlocal output, previous_ns, has_target
            now_ns = self.get_clock().now().nanoseconds
            selected = self._heading_selected()
            if selected is None:
                previous_ns = now_ns
                return output
            _, actual = self._verify_control_pose()
            actual_yaw = yaw_from_quaternion(actual.orientation)
            lead = self.config.get('verify_reacquire_yaw_step', math.radians(5.0))
            rate = self.config.get('verify_heading_rate', math.radians(12.0))
            if not all(math.isfinite(value) and value > 0.0 for value in (lead, rate)):
                raise ValueError('Heading lead and rate must be finite and positive')
            error = angular_distance(selected['body_target'], actual_yaw)
            desired = actual_yaw + max(-lead, min(lead, error))
            if not has_target:
                output = desired
            elif now_ns > previous_ns:
                change = angular_distance(desired, output)
                limit = rate * (now_ns - previous_ns) / 1e9
                output += max(-limit, min(limit, change))
            # Cancel a lagging command that would steer away from the current
            # target, or beyond it, while the normal slew limit catches up.
            proposed = angular_distance(output, actual_yaw)
            correction = (max(0.0, min(lead, error, proposed)) if error >= 0.0
                          else min(0.0, max(-lead, error, proposed)))
            output = actual_yaw + correction
            has_target = True
            previous_ns = now_ns
            output = math.remainder(output, math.tau)
            self._heading_log(selected, actual_yaw, output)
            return output
        return heading

    def _heading_log(self, observation, actual, commanded):
        state = self._heading_state()
        now = time.monotonic()
        signature = (observation['source'], observation['frame'],
                     round(observation['body_target'], 2), round(commanded, 2))
        changed_source = (state.get('last_log') is None
                          or signature[:2] != state['last_log'][:2])
        correction = angular_distance(commanded, actual)
        reversed_direction = correction * state.get('logged_correction', 0.0) < -1e-6
        if (not changed_source and not reversed_direction and signature == state.get('last_log')
                and now - state.get('logged_at', -math.inf) < 5.0):
            return
        if (not changed_source and not reversed_direction
                and now - state.get('logged_at', -math.inf) < 0.5):
            return
        age = (self.get_clock().now().nanoseconds - observation['stamp']) / 1e9
        self._report(
            f'Verify: heading source={observation["source"]}, '
            f'raw image error={math.degrees(observation["raw_error"]):+.2f} deg, '
            f'actual yaw={math.degrees(actual):+.2f} deg, '
            f'absolute ray target={math.degrees(observation["bearing"]):+.2f} deg, '
            f'body yaw target={math.degrees(observation["body_target"]):+.2f} deg, '
            f'commanded yaw={math.degrees(commanded):+.2f} deg, '
            f'capped correction={math.degrees(angular_distance(commanded, actual)):+.2f} deg, '
            f'image age={age:.2f} s')
        state['last_log'], state['logged_at'] = signature, now
        state['logged_correction'] = correction

    def _verify_fire_heading_ready(self):
        """Check cached fresh fire alignment without blocking on TF or consuming an image."""
        fire = self._verify.stable_fire()
        if not self._inspection_detection_valid(fire, 'fire'):
            return True
        if not self._heading_clock_running():
            return False
        observation = self._heading_state()['sources'].get(('fire', fire.header.frame_id))
        if observation is None or observation['stamp'] != _stamp_ns(fire):
            return False
        _, current = self._verify_control_pose()
        tolerance = self.config.get('verify_fire_heading_tolerance', math.radians(2.0))
        return (abs(fire.yaw_correction_rad) <= tolerance and abs(angular_distance(
            observation['body_target'], yaw_from_quaternion(current.orientation))) <= tolerance)

    def _verify_align_fire_heading(self, guard=None):
        """Best-effort fire alignment before starting a new stationary evidence dwell."""
        from kv_executive.verification import VerificationObservationLost
        deadline = time.monotonic() + self.config.get('verify_fire_heading_align_timeout', 15.0)
        after_ns = 0

        def bounded_guard():
            if guard:
                guard()
            if time.monotonic() >= deadline:
                raise VerificationObservationLost('Stopped fire heading alignment timed out')

        while time.monotonic() < deadline:
            if guard:
                guard()
            fire = self._verify.stable_fire()
            if not self._inspection_detection_valid(fire, 'fire'):
                return True
            if not self._heading_clock_running():
                return False
            if _stamp_ns(fire) <= after_ns:
                if self._stop.wait(0.05):
                    return False
                continue
            observation = self._heading_observation('fire', fire, deadline=deadline)
            if observation is None or time.monotonic() >= deadline:
                return False
            _, current = self._verify_control_pose()
            tolerance = self.config.get('verify_fire_heading_tolerance', math.radians(2.0))
            remaining = abs(angular_distance(observation['body_target'], yaw_from_quaternion(
                current.orientation)))
            if remaining <= tolerance:
                if abs(fire.yaw_correction_rad) <= tolerance:
                    self._report(
                        'Verify: stopped fire heading aligned; '
                        f'image error={math.degrees(fire.yaw_correction_rad):+.2f} deg, '
                        f'body yaw target={math.degrees(observation["body_target"]):+.2f} deg, '
                        f'tolerance={math.degrees(tolerance):.2f} deg')
                    return True
                if remaining <= 1e-6:
                    # Body already faces the fixed target. Wait for a new image
                    # instead of applying the old relative error a second time.
                    after_ns = self.get_clock().now().nanoseconds
                    if self._stop.wait(0.05):
                        return False
                    continue
            turn_tolerance = (min(tolerance, remaining / 2.0)
                              if remaining <= tolerance else tolerance)
            self._verify_wall_observation_context = None
            self._report(
                'Verify: stopped fire heading alignment; '
                f'raw image error={math.degrees(fire.yaw_correction_rad):+.2f} deg, '
                f'actual yaw={math.degrees(yaw_from_quaternion(current.orientation)):+.2f} deg, '
                f'absolute ray target={math.degrees(observation["bearing"]):+.2f} deg, '
                f'body yaw target={math.degrees(observation["body_target"]):+.2f} deg, '
                f'tolerance={math.degrees(turn_tolerance):.2f} deg')
            try:
                self._verify_rotate(observation['body_target'], guard=bounded_guard,
                                    yaw_tolerance=turn_tolerance)
            except VerificationObservationLost:
                return False
            after_ns = self.get_clock().now().nanoseconds
        return False
