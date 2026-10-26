"""Guarded target navigation and confirmed-fire observation reporting."""

from contextlib import contextmanager
from copy import deepcopy
import math
import time

from geometry_msgs.msg import Pose, PoseStamped
from kv_executive.mission_geometry import (
    angular_distance, interpolate_pose, quaternion_from_yaw, relative_transform,
    split_path, transform_pose, yaw_from_quaternion,
)
from kv_executive.verification import VerificationObservationLost
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _distance(left, right):
    return math.dist(
        (left.position.x, left.position.y, left.position.z),
        (right.position.x, right.position.y, right.position.z),
    )


def _inverse(transform):
    """Invert a rigid transform using the validated mission geometry helpers."""
    source = Pose()
    source.position.x = transform.translation.x
    source.position.y = transform.translation.y
    source.position.z = transform.translation.z
    source.orientation = deepcopy(transform.rotation)
    identity = Pose()
    identity.orientation.w = 1.0
    return relative_transform(identity, source)


class VerifyNavigationMixin:
    """Use the executive's bounded OFFBOARD/LOITER primitives for Verify."""

    def _verify_control_pose(self):
        from kv_executive.mission_executive import MissionError
        self._check_running()
        state, odometry = self._snapshot(require_armed=True)
        if not self._owns_flight or state.mode not in {'AUTO.LOITER', 'OFFBOARD'}:
            raise MissionError(f'Verify cannot take control from {state.mode}')
        return state, deepcopy(odometry.pose.pose)

    def _verify_offboard(self):
        state, current = self._verify_control_pose()
        if state.mode == 'AUTO.LOITER':
            self._enter_offboard(current, allowed_modes=('AUTO.LOITER',))
        return deepcopy(self._snapshot('OFFBOARD', require_armed=True)[1].pose.pose)

    @contextmanager
    def _verify_guarded(self, guard):
        from kv_executive.mission_executive import MissionError
        previous_guard = self._motion_guard
        self._motion_guard = guard
        try:
            self._check_running()
            if guard:
                guard()
            yield
            self._check_running()
            if guard:
                guard()
        except VerificationObservationLost:
            self._motion_guard = None
            with self._goal_lock:
                was_recovering = self._recovering
                self._recovering = True
            try:
                recovery = self._recover_to_loiter()
                state, _ = self._snapshot(require_armed=True)
                if state.mode == 'OFFBOARD' or 'failed' in recovery.lower():
                    raise MissionError('Verify observation lost; LOITER recovery failed')
            finally:
                with self._goal_lock:
                    self._recovering = was_recovering
            raise
        finally:
            self._motion_guard = previous_guard

    def _verify_rotate(self, yaw, guard=None, hold_duration=None):
        """Rotate about local Z at the current position and finish in LOITER."""
        target_orientation = quaternion_from_yaw(yaw)
        with self._verify_guarded(guard):
            current = self._verify_offboard()
            target = deepcopy(current)
            target.orientation = target_orientation
            turn = abs(angular_distance(yaw, yaw_from_quaternion(current.orientation)))
            self._ramp(
                current, target,
                max(0.001, turn * self.config['path_turn_step_duration'] / (math.tau / 6.0)),
                hold_duration=hold_duration,
            )
            self._loiter()

    def _verify_hold(self, duration, guard=None):
        """Hold a level position for a measured dwell, then finish in LOITER."""
        from kv_executive.mission_executive import MissionError
        if not math.isfinite(duration) or duration <= 0.0:
            raise MissionError('Verify hold duration must be finite and positive')
        with self._verify_guarded(guard):
            current = self._verify_offboard()
            current.orientation = quaternion_from_yaw(yaw_from_quaternion(current.orientation))
            self._set_target(current)
            self._wait_at_pose(current, hold_duration=duration)
            self._loiter()

    def _verify_plan_to(self, target_local_pose, guard=None, speed=None):
        """Validate a target plan, retain its vertices, and fly in short ramps."""
        from kv_executive.mission_executive import MissionError
        from kv_executive.srv import PlanToPose
        speed = self.config['verify_motion_speed'] if speed is None else speed
        if not math.isfinite(speed) or speed <= 0.0:
            raise MissionError('Verify motion speed must be finite and positive')
        target = interpolate_pose(target_local_pose, target_local_pose, 0.0)
        target.orientation = quaternion_from_yaw(yaw_from_quaternion(target.orientation))
        if target.position.z < self.config['verify_minimum_altitude']:
            raise MissionError('Verify target is below the configured minimum altitude')
        with self._verify_guarded(guard):
            _, current = self._verify_control_pose()
            frame = self.config['verify_planning_frame']
            conversion = self._path_frame_transform(frame)
            request = PlanToPose.Request()
            request.target = PoseStamped()
            request.target.header.frame_id = frame
            request.target.pose = transform_pose(target, _inverse(conversion))
            response = self._call(
                self._target_planning_client, request, 'Plan Verify target',
                timeout=self.config['planning_timeout'],
            )
            self._verify_control_pose()
            if guard:
                guard()
            if not response.success:
                raise MissionError(f'Verify planner failed: {response.message}')
            path = response.path
            if path.header.frame_id != frame:
                raise MissionError('Verify planner returned an unexpected path frame')
            try:
                split_path(path, segments=1)  # Validates every vertex and quaternion.
                vertices = [transform_pose(stamped.pose, conversion) for stamped in path.poses]
            except ValueError as error:
                raise MissionError(f'Invalid Verify path: {error}') from error
            if _distance(vertices[-1], target) > 1e-3:
                raise MissionError('Verify planner did not reach the requested endpoint')
            if any(pose.position.z < self.config['verify_minimum_altitude'] for pose in vertices):
                raise MissionError('Verify path drops below the configured minimum altitude')
            _, current = self._verify_control_pose()
            if _distance(vertices[0], current) > self.config['position_tolerance']:
                raise MissionError('Vehicle moved from the Verify planned start')
            self.latest_path = deepcopy(path)
            current = self._verify_offboard()
            if _distance(vertices[0], current) > self.config['position_tolerance']:
                raise MissionError('Vehicle moved from Verify planned start during warmup')
            # Preserve the requested camera heading throughout a detour. The
            # planner's arbitrary per-vertex orientations are not camera guidance.
            heading = deepcopy(target.orientation)
            facing = deepcopy(current)
            facing.orientation = heading
            turn = abs(angular_distance(
                yaw_from_quaternion(heading), yaw_from_quaternion(current.orientation)))
            if turn > 1e-6:
                self._ramp(
                    current, facing,
                    turn * self.config['path_turn_step_duration'] / (math.tau / 6.0),
                )
            current = facing
            # Never skip a planner vertex. Short setpoint segments bound how
            # far a stale camera observation can advance the commanded position.
            spacing = min(self.config.get('verify_path_step', 0.25),
                          speed / self.config['setpoint_rate'])
            for vertex in vertices:
                vertex.orientation = deepcopy(heading)
                start = deepcopy(current)
                length = _distance(start, vertex)
                pieces = max(1, math.ceil(length / spacing))
                for index in range(1, pieces + 1):
                    next_pose = interpolate_pose(start, vertex, index / pieces)
                    self._ramp(current, next_pose, _distance(current, next_pose) / speed,
                               hold_duration=0.0 if index < pieces else None)
                    current = next_pose
            self._loiter()

    def _verify_record_fire(self, fire_message, bearing_yaw):
        """Publish a fresh GPS/local/map observation viewpoint and exclusion site."""
        from kv_executive.mission_executive import MissionError
        from kv_executive.msg import FireSourceReport
        self._verify_control_pose()
        with self._data_lock:
            fix = deepcopy(self._global_position)
            received = self._global_position_received
        now_ns = self.get_clock().now().nanoseconds
        timeout = self.config['global_position_timeout']
        if not isinstance(fix, NavSatFix):
            raise MissionError('Cannot report fire without global-position telemetry')
        source_ns = fix.header.stamp.sec * 1_000_000_000 + fix.header.stamp.nanosec
        age = (now_ns - source_ns) / 1e9
        if (
            fix.status.status < NavSatStatus.STATUS_FIX
            or now_ns <= 0 or source_ns <= 0
            or not -self.config['verify_future_tolerance'] <= age <= timeout
            or not 0.0 <= time.monotonic() - received <= timeout
            or not all(math.isfinite(value) for value in (
                fix.latitude, fix.longitude, fix.altitude, bearing_yaw))
            or not -90.0 <= fix.latitude <= 90.0
            or not -180.0 <= fix.longitude <= 180.0
        ):
            raise MissionError('Cannot report fire with invalid or stale global position')
        count = int(getattr(fire_message, 'source_count', 1))
        track_id = int(getattr(fire_message, 'track_id', 0))
        if not 0 < count < 2 ** 32 or not 0 <= track_id < 2 ** 64:
            raise MissionError('Cannot report an invalid fire source count or track')
        _, odometry = self._snapshot(require_armed=True)
        fire_stamp = fire_message.header.stamp
        fire_ns = fire_stamp.sec * 1_000_000_000 + fire_stamp.nanosec
        odometry_ns = (
            odometry.header.stamp.sec * 1_000_000_000 + odometry.header.stamp.nanosec)
        if (not fire_message.detected or fire_message.status != fire_message.CONFIRMED
                or fire_ns <= 0 or odometry_ns <= 0):
            raise MissionError('Cannot report an unconfirmed or unstamped fire observation')
        conversion = self._path_frame_transform(self.config['verify_planning_frame'])
        report = FireSourceReport()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = self.config['verify_planning_frame']
        report.label = 'Fire source found'
        report.local_pose = PoseStamped()
        report.local_pose.header = deepcopy(report.header)
        report.local_pose.header.frame_id = self.config['local_frame']
        report.local_pose.pose = deepcopy(odometry.pose.pose)
        report.planning_pose = PoseStamped()
        report.planning_pose.header = deepcopy(report.header)
        report.planning_pose.pose = transform_pose(odometry.pose.pose, _inverse(conversion))
        report.global_position = fix
        report.source_count = count
        report.track_id = track_id
        report.exclusion_radius = float(self.config['fire_exclusion_radius'])
        report.bearing_rad = float(math.remainder(bearing_yaw, math.tau))
        self._verify_control_pose()
        now_ns = self.get_clock().now().nanoseconds
        future = self.config['verify_future_tolerance']
        if (
            not -future <= (now_ns - fire_ns) / 1e9
            <= self.config['verify_observation_timeout']
            or not -future <= (now_ns - odometry_ns) / 1e9
            <= self.config['telemetry_timeout']
            or not -future <= (now_ns - source_ns) / 1e9 <= timeout
            or time.monotonic() - received > timeout
        ):
            raise MissionError('Fire observation telemetry became stale before reporting')
        current_fire = self._verify.stable_fire()
        if (
            current_fire is None
            or current_fire.header.frame_id != fire_message.header.frame_id
            or getattr(current_fire, 'track_id', 0) != track_id
            or (current_fire.header.stamp.sec * 1_000_000_000
                + current_fire.header.stamp.nanosec) < fire_ns
            or not all(math.isfinite(angle) and abs(angle)
                       <= self.config['verify_yaw_tolerance'] for angle in (
                           current_fire.yaw_correction_rad,
                           current_fire.vertical_correction_rad))
        ):
            raise VerificationObservationLost(
                'Tracked fire changed or lost centering before reporting')
        current_count = int(getattr(current_fire, 'source_count', 1))
        if not 0 < current_count < 2 ** 32:
            raise MissionError('Current fire source count is invalid')
        report.source_count = current_count
        with self._data_lock:
            self._site_sequence += 1
            report.site_id = self._site_sequence
            self._fire_sites.append(deepcopy(report))
            del self._fire_sites[:-100]
        self._fire_report_publisher.publish(report)
        self._report('Verify: Fire source found; observation viewpoint recorded')
        return report
