"""Use a retained map coordinate to obtain fresh visual inspection evidence."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Point, Pose
from kv_executive.inspection_heading import _horizontal_centering_turn
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, transform_pose, yaw_from_quaternion,
)
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


def _stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def retained_wall_valid(wall, planning_frame, maximum_uncertainty=2.0):
    """Validate the original point and two distinct confirming map observations."""
    if not isinstance(wall, dict):
        return False
    point, stamps = wall.get('nearest_point'), wall.get('confirmation_stamps', ())
    if (not isinstance(stamps, (tuple, list)) or len(stamps) != 2
            or not all(isinstance(s, int) and not isinstance(s, bool) and s > 0 for s in stamps)
            or stamps[0] == stamps[1]
            or not wall.get('confirmed', False) or not wall.get('nearest_point_valid', False)
            or point is None or point.header.frame_id != planning_frame
            or _stamp_ns(point) != max(stamps)
            or not all(math.isfinite(getattr(point.point, a)) for a in 'xyz')):
        return False
    nearest, uncertainty = wall.get('nearest', math.nan), wall.get('uncertainty', math.nan)
    return (math.isfinite(nearest) and nearest > 0.0 and math.isfinite(uncertainty)
            and 0.0 <= uncertainty <= maximum_uncertainty)


class InspectionReportViewMixin:
    """Look from the current pose without moving a historical wall estimate."""

    def _verify_report_camera_frame(self, retained_wall):
        controller = self._verify
        smoke = controller.fresh_smoke()
        if smoke is not None and smoke.header.frame_id:
            return smoke.header.frame_id
        lock = getattr(controller, '_lock', None)
        if lock is None:
            previous = getattr(controller, '_smoke_last_confirmed', None)
        else:
            with lock:
                previous = deepcopy(getattr(controller, '_smoke_last_confirmed', None))
        return (previous.header.frame_id if previous is not None
                else retained_wall.get('camera_frame', ''))

    def _verify_report_target_yaw(self, retained_wall):
        """Face a fixed map hit using current full camera orientation and body yaw."""
        from kv_executive.verification import VerificationObservationLost
        point = retained_wall.get('nearest_point')
        frame = self.config['verify_planning_frame']
        camera = self._verify_report_camera_frame(retained_wall)
        if (point is None or point.header.frame_id != frame or not camera
                or not all(math.isfinite(getattr(point.point, axis)) for axis in 'xyz')):
            raise VerificationObservationLost(
                'Retained smoke coordinate or camera frame unavailable')
        try:
            # Latest camera pose is intentional: the map point stays fixed
            # while the drone may have translated since wall confirmation.
            camera_transform = self._tf_buffer.lookup_transform(
                frame, camera, Time(),
                timeout=Duration(seconds=self.config.get('transform_timeout', 0.5))).transform
            conversion = self._path_frame_transform(frame)
            target = transform_pose(Pose(
                position=deepcopy(point.point), orientation=quaternion_from_yaw(0.0)),
                conversion).position
            axes = []
            for position in (Point(), Point(x=1.0), Point(y=1.0)):
                pose = transform_pose(Pose(
                    position=position, orientation=quaternion_from_yaw(0.0)), camera_transform)
                axes.append(transform_pose(pose, conversion).position)
        except (TransformException, ValueError) as error:
            raise VerificationObservationLost(
                'Retained smoke viewing transform unavailable') from error
        except RuntimeError as error:
            if not isinstance(error.__cause__, TransformException):
                raise
            raise VerificationObservationLost(
                'Retained smoke viewing transform unavailable') from error
        origin, forward, left = axes
        ray = tuple(getattr(target, a) - getattr(origin, a) for a in 'xyz')
        forward = tuple(getattr(forward, a) - getattr(origin, a) for a in 'xyz')
        left = tuple(getattr(left, a) - getattr(origin, a) for a in 'xyz')
        turn = _horizontal_centering_turn(ray, forward, left)
        if turn is None:
            raise VerificationObservationLost(
                'Retained smoke coordinate cannot be faced by yaw alone')
        _, pose = self._verify_control_pose()
        return math.remainder(yaw_from_quaternion(pose.orientation) + turn, math.tau)

    def _verify_prepare_report_view(self, retained_wall, fire_message=None, guard=None):
        """Make one bounded viewing attempt, then require new smoke in any quadrant."""
        from kv_executive.verification import VerificationObservationLost
        if not retained_wall_valid(
                retained_wall, self.config['verify_planning_frame'],
                self.config.get('verify_inspection_max_uncertainty', 2.0)):
            raise VerificationObservationLost('A confirmed retained smoke coordinate is required')
        timeout = self.config.get('verify_report_view_timeout', 15.0)
        tolerance = self.config.get('verify_report_view_yaw_tolerance', math.radians(30.0))
        if (not all(math.isfinite(v) and v > 0.0 for v in (timeout, tolerance))
                or tolerance >= math.pi / 2):
            raise ValueError('Report viewing limits must be finite, positive and forward-facing')
        deadline = time.monotonic() + timeout
        after_ns = self.get_clock().now().nanoseconds

        def check():
            self._check_running()
            if guard:
                guard()
            if time.monotonic() >= deadline:
                raise VerificationObservationLost(
                    'Fresh forward smoke unavailable for inspection report')

        check()
        fire = self._verify.stable_fire()
        visible_fire = self._inspection_detection_valid(fire, 'fire')
        if not visible_fire:
            try:
                target_yaw = self._verify_report_target_yaw(retained_wall)
                _, pose = self._verify_control_pose()
                if abs(angular_distance(target_yaw, yaw_from_quaternion(
                        pose.orientation))) > tolerance:
                    self._report('Verify: facing retained smoke coordinate for final visual check')
                    self._verify_rotate(target_yaw, guard=check)
                    after_ns = max(after_ns, self.get_clock().now().nanoseconds)
            except VerificationObservationLost as error:
                # A camera transform or unreachable vertical direction cannot
                # manufacture visibility, but a new forward image may arrive.
                self._report('Verify: retained smoke viewing attempt incomplete: ' + str(error))
                check()
        if not self._verify_prepare_wall_observation(
                guard=check, align_fire=False, wait_for_map=False):
            raise VerificationObservationLost('Stationary report viewpoint unavailable')
        context = self._verify_wall_observation_context
        after_ns = max(after_ns, context['settled_ns'])
        self._verify_report_view_after_ns = after_ns
        while True:
            check()
            if not self._verify_wall_observation_ready():
                raise VerificationObservationLost(
                    'Report viewpoint moved before visual confirmation')
            smoke = self._verify.fresh_smoke()
            if (self._inspection_detection_valid(smoke, 'smoke')
                    and _stamp_ns(smoke) > after_ns):
                return smoke
            if self._stop.wait(0.05):
                check()
