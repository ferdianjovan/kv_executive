"""Request Verify on confirmed camera detections during Exploration."""

from collections import deque
from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from kv_executive.msg import MissionPhase, MissionStateRequest
from kv_perception.msg import (
    FireDetection, FireDetectionArray, SmokeDetection, SmokeDetectionArray,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID


DEFAULTS = {
    'smoke_topic': 'smoke/detection',
    'fire_topic': 'fire/detection',
    'action_status_topic': 'execute_mission/_action/status',
    'mission_state_request_topic': 'mission_state_request_for_goal',
    'mission_phase_topic': 'mission_phase',
    'observation_timeout': 2.0,
    'future_tolerance': 0.25,
    'request_retry_interval': 2.0,
}

_CLOSED_GOAL_LIMIT = 64


class DetectionMonitor(Node):
    """Observe detector confirmation and request a goal-scoped Verify transition.

    The detectors establish persistence themselves. Either confirmed source is
    sufficient; visible camera detections do not establish a measured distance.
    Only fresh source images acquired strictly after both action acceptance and
    Exploration entry can trigger. Requests retry at a bounded rate while that
    phase remains active, allowing a new source after a completed site is ignored.
    Observations outside Exploration never issue additional requests. Requests
    carry that goal's UUID so delayed delivery cannot interrupt its successor.
    The executive owns state changes, flight commands, and safe recovery.
    """

    def __init__(self, **kwargs):
        """Subscribe to confirmed detections and the executive's action status."""
        super().__init__('detection_monitor', **kwargs)
        self.config = {
            name: self.declare_parameter(name, default).value
            for name, default in DEFAULTS.items()
        }
        for name, default in DEFAULTS.items():
            value = self.config[name]
            if isinstance(default, str) and not value.strip():
                raise ValueError(f'{name} must not be empty')
            if isinstance(default, float) and (
                not math.isfinite(value) or value < 0.0
                or (name in ('observation_timeout', 'request_retry_interval') and value == 0.0)
            ):
                requirement = (
                    'positive' if name in ('observation_timeout', 'request_retry_interval')
                    else 'nonnegative'
                )
                raise ValueError(f'{name} must be finite and {requirement}')
        self._initialize_tracking()
        detection_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._request_publisher = self.create_publisher(
            MissionStateRequest, self.config['mission_state_request_topic'], detection_qos,
        )
        # Do not replace Node._subscriptions, which also owns /clock.
        self._monitor_subscriptions = [
            self.create_subscription(
                SmokeDetection, self.config['smoke_topic'], self._smoke_callback,
                detection_qos,
            ),
            self.create_subscription(
                FireDetection, self.config['fire_topic'], self._fire_callback,
                detection_qos,
            ),
            self.create_subscription(
                SmokeDetectionArray, self.config['smoke_topic'] + '_candidates',
                lambda message: self._candidates_callback(message, 'smoke'), detection_qos,
            ),
            self.create_subscription(
                FireDetectionArray, self.config['fire_topic'] + '_candidates',
                lambda message: self._candidates_callback(message, 'fire'), detection_qos,
            ),
            self.create_subscription(
                GoalStatusArray, self.config['action_status_topic'], self._status_callback,
                status_qos,
            ),
            self.create_subscription(
                MissionPhase, self.config['mission_phase_topic'], self._phase_callback,
                status_qos,
            ),
        ]
        self.get_logger().info(
            'Detection monitor ready; confirmed smoke or fire during an executing '
            'Exploration phase will request Verify'
        )

    def _initialize_tracking(self):
        self._active_goal_id = None
        self._active_goal_stamp_ns = 0
        self._request_sent = False
        self._next_request_at = 0.0
        self._closed_goals = deque()
        self._closed_goal_ids = set()
        self._phase_goal_id = None
        self._phase_sequence = -1
        self._phase_state = None
        self._phase_stamp_ns = 0
        self._phase_cache = {}
        self._clear_candidate_tracking()

    def _clear_candidate_tracking(self):
        self._candidate_stamps = {'smoke': 0, 'fire': 0}
        self._candidate_observations = {'smoke': None, 'fire': None}

    def _remember_closed_goal(self, goal_id):
        self._phase_cache.pop(goal_id, None)
        if goal_id in self._closed_goal_ids:
            return
        if len(self._closed_goals) >= _CLOSED_GOAL_LIMIT:
            self._closed_goal_ids.remove(self._closed_goals.popleft())
        self._closed_goals.append(goal_id)
        self._closed_goal_ids.add(goal_id)

    def _status_callback(self, message):
        # Action status is retained by the server, so a monitor started during a
        # mission can join it. Accepted, canceling, and finished goals are idle
        # for this monitor. The executive permits only one active mission.
        for status in message.status_list:
            if status.status in {
                GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
            }:
                self._remember_closed_goal(tuple(status.goal_info.goal_id.uuid))
        executing = [
            status for status in message.status_list
            if status.status == GoalStatus.STATUS_EXECUTING
            and tuple(status.goal_info.goal_id.uuid) not in self._closed_goal_ids
        ]
        goal_id = None
        accepted_ns = 0
        if len(executing) == 1:
            info = executing[0].goal_info
            goal_id = tuple(info.goal_id.uuid)
            accepted_ns = info.stamp.sec * 1_000_000_000 + info.stamp.nanosec
        if goal_id == self._active_goal_id:
            return  # Duplicate status must never rearm a triggered goal.
        if self._active_goal_id is not None:
            self._remember_closed_goal(self._active_goal_id)
        self._active_goal_id = goal_id
        self._active_goal_stamp_ns = accepted_ns
        self._request_sent = False
        self._next_request_at = 0.0
        self._clear_candidate_tracking()
        if goal_id != self._phase_goal_id:
            self._phase_goal_id = None
            self._phase_sequence = -1
            self._phase_state = None
            self._phase_stamp_ns = 0
        phase = self._phase_cache.get(goal_id)
        if phase is not None:
            self._apply_phase(phase)
        # Observations are never cached across action goals. A newly executing
        # goal requires a newly received, confirmed, correctly stamped image.

    def _phase_callback(self, message):
        goal_id = tuple(message.goal_id.uuid)
        if goal_id in self._closed_goal_ids:
            return
        if goal_id == self._phase_goal_id and message.sequence <= self._phase_sequence:
            return
        previous = self._phase_cache.get(goal_id)
        if previous is not None and message.sequence <= previous.sequence:
            return
        self._phase_cache[goal_id] = message
        if len(self._phase_cache) > _CLOSED_GOAL_LIMIT:
            del self._phase_cache[next(iter(self._phase_cache))]
        if goal_id == self._active_goal_id:
            self._apply_phase(message)

    def _apply_phase(self, message):
        goal_id = tuple(message.goal_id.uuid)
        self._phase_goal_id = goal_id
        self._phase_sequence = message.sequence
        self._phase_state = message.state
        stamp = message.header.stamp
        self._phase_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if message.state == 'Exploration':
            self._request_sent = False
            self._next_request_at = 0.0
            self._clear_candidate_tracking()

    def _smoke_callback(self, message):
        if (
            message.detected and message.status == SmokeDetection.CONFIRMED
            and message.background_ready
        ):
            self._request_verify(message, 'smoke')
        self._request_current_candidate('smoke')

    def _fire_callback(self, message):
        if message.detected and message.status == FireDetection.CONFIRMED:
            self._request_verify(message, 'fire')
        self._request_current_candidate('fire')

    def _candidates_callback(self, array, source):
        """Observe bounded independent confirmations without replacing primary cues."""
        if source not in ('smoke', 'fire') or len(array.detections) > 16:
            return
        stamp = array.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if (not array.header.frame_id.strip() or source_ns <= 0 or now_ns <= 0
                or not -self.config['future_tolerance'] <= (now_ns - source_ns) / 1e9
                <= self.config['observation_timeout']):
            return
        previous = self._candidate_stamps[source]
        if source_ns < previous or (source_ns == previous and array.detections):
            return
        self._candidate_stamps[source] = source_ns
        confirmed = SmokeDetection.CONFIRMED if source == 'smoke' else FireDetection.CONFIRMED
        selected = next((message for message in array.detections
                         if message.header == array.header and message.detected
                         and message.status == confirmed
                         and (source != 'smoke' or message.background_ready)), None)
        self._candidate_observations[source] = (
            (deepcopy(selected), time.monotonic()) if selected is not None else None)
        self._request_current_candidate(source)

    def _request_current_candidate(self, source):
        """Retain a fresh independent candidate through primary misses."""
        observation = self._candidate_observations[source]
        if observation is not None and (
                0.0 <= time.monotonic() - observation[1] <= self.config['observation_timeout']):
            self._request_verify(observation[0], source)

    def _request_verify(self, message, source):
        if (
            self._active_goal_id is None
            or self._phase_goal_id != self._active_goal_id
            or self._phase_state != 'Exploration'
        ):
            return
        now = time.monotonic()
        if self._request_sent and now < self._next_request_at:
            return
        stamp = message.header.stamp
        image_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0 or image_ns <= 0:
            return  # ROS simulation time and the source image must be initialized.
        if image_ns <= max(self._active_goal_stamp_ns, self._phase_stamp_ns):
            # Queued images from a previous phase or goal, including its entry
            # clock tick, cannot interrupt this Exploration entry.
            return
        age_ns = now_ns - image_ns
        if not (
            -int(self.config['future_tolerance'] * 1_000_000_000)
            <= age_ns <= int(self.config['observation_timeout'] * 1_000_000_000)
        ):
            return
        if self._request_publisher.get_subscription_count() == 0:
            return  # Retry on a later confirmed frame after DDS discovery.
        self._request_publisher.publish(MissionStateRequest(
            goal_id=UUID(uuid=list(self._active_goal_id)), state='Verify',
        ))
        if not self._request_sent:
            self.get_logger().info(
                f'Confirmed {source} detected; requesting Verify from Exploration'
            )
        self._request_sent = True
        self._next_request_at = now + self.config['request_retry_interval']


def main(args=None):
    """Run the passive monitor until ROS shutdown."""
    rclpy.init(args=args)
    node = DetectionMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
