"""Detection and action lifecycle safety without a running ROS graph."""

from copy import deepcopy
from types import MethodType, SimpleNamespace

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time
import kv_executive.detection_monitor as monitor_module
from kv_executive.detection_monitor import DEFAULTS, DetectionMonitor
from kv_executive.msg import MissionPhase
from kv_perception.msg import FireDetection, SmokeDetection
import pytest
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID


def stamp(seconds):
    nanoseconds = round(seconds * 1_000_000_000)
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def goal_status(goal=1, status=GoalStatus.STATUS_EXECUTING, accepted=19.0):
    return GoalStatus(
        goal_info=GoalInfo(goal_id=UUID(uuid=[goal] * 16), stamp=stamp(accepted)),
        status=status,
    )


class Publisher:
    def __init__(self):
        self.messages = []
        self.subscribers = 1

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(deepcopy(message))


class Harness:
    """Exercise real monitor callbacks with deterministic ROS time and outputs."""

    def __init__(self):
        self.config = deepcopy(DEFAULTS)
        self.seconds = 20.0
        self._request_publisher = Publisher()
        self.logs = []
        self._initialize_tracking()

    def __getattr__(self, name):
        return MethodType(getattr(DetectionMonitor, name), self)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=round(self.seconds * 1_000_000_000),
        ))

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def status(self, *statuses):
        self._status_callback(GoalStatusArray(status_list=list(statuses)))

    def phase(self, state='Exploration', goal=1, sequence=1, entered=0.0):
        self._phase_callback(MissionPhase(
            header=Header(stamp=stamp(entered)), goal_id=UUID(uuid=[goal] * 16),
            sequence=sequence, state=state,
        ))

    def detection(self, source='smoke', acquired=None, **overrides):
        values = {
            'header': Header(stamp=stamp(self.seconds if acquired is None else acquired)),
            'detected': True,
        }
        if source == 'smoke':
            values.update(status=SmokeDetection.CONFIRMED, background_ready=True)
            values.update(overrides)
            self._smoke_callback(SmokeDetection(**values))
        else:
            values.update(status=FireDetection.CONFIRMED)
            values.update(overrides)
            self._fire_callback(FireDetection(**values))


@pytest.fixture
def monitor(monkeypatch):
    harness = Harness()
    harness.monotonic_seconds = 0.0
    monkeypatch.setattr(monitor_module.time, 'monotonic', lambda: harness.monotonic_seconds)
    return harness


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_either_confirmed_detector_can_request_verify_alone(monitor, source):
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection(source)

    assert [message.state for message in monitor._request_publisher.messages] == ['Verify']
    assert monitor._request_publisher.messages[0].goal_id == UUID(uuid=[1] * 16)
    assert source in monitor.logs[-1]


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_idle_detection_does_not_carry_into_later_goal(monitor, source):
    monitor.detection(source)
    monitor.status(goal_status())
    monitor.phase()
    assert monitor._request_publisher.messages == []

    monitor.detection(source)
    assert len(monitor._request_publisher.messages) == 1


@pytest.mark.parametrize('status', [
    GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_CANCELING,
    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
])
def test_only_executing_goals_can_request_verify(monitor, status):
    monitor.status(goal_status(status=status))
    monitor.phase()
    monitor.detection('smoke')
    monitor.detection('fire')

    assert monitor._request_publisher.messages == []


@pytest.mark.parametrize('source, fields', [
    ('smoke', {'detected': False}),
    ('smoke', {'status': SmokeDetection.CALIBRATING}),
    ('smoke', {'status': SmokeDetection.SEARCHING}),
    ('smoke', {'status': SmokeDetection.CONFIRMING}),
    ('smoke', {'background_ready': False}),
    ('fire', {'detected': False}),
    ('fire', {'status': FireDetection.SEARCHING}),
    ('fire', {'status': FireDetection.CONFIRMING}),
])
def test_candidates_and_inconsistent_flags_never_interrupt_exploration(monitor, source, fields):
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection(source, **fields)

    assert monitor._request_publisher.messages == []


@pytest.mark.parametrize('source', ['smoke', 'fire'])
@pytest.mark.parametrize('acquired', [0.0, 17.9, 20.250000001, 19.49])
def test_zero_stale_future_and_pre_goal_source_images_are_ignored(monitor, source, acquired):
    monitor.status(goal_status(accepted=19.5))
    monitor.phase()
    monitor.detection(source, acquired=acquired)

    assert monitor._request_publisher.messages == []


@pytest.mark.parametrize('acquired', [18.0, 20.25])
def test_freshness_boundaries_are_inclusive(monitor, acquired):
    monitor.status(goal_status(accepted=17.9))
    monitor.phase()
    monitor.detection('fire', acquired=acquired)

    assert len(monitor._request_publisher.messages) == 1


def test_requests_are_throttled_despite_repeated_and_simultaneous_detections(monitor):
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection('smoke')
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection('fire')
    monitor.detection('smoke')
    assert len(monitor._request_publisher.messages) == 1

    monitor.status(goal_status(status=GoalStatus.STATUS_ABORTED))
    monitor.phase()
    monitor.status(goal_status(goal=2, accepted=20.0))
    monitor.phase(goal=2)
    monitor.seconds = 20.1
    monitor.detection('fire')
    assert len(monitor._request_publisher.messages) == 2
    assert monitor._request_publisher.messages[1].goal_id == UUID(uuid=[2] * 16)


def test_ignored_completed_source_does_not_latch_out_later_new_source(monitor):
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection('fire')
    monitor.monotonic_seconds = monitor.config['request_retry_interval'] - 0.01
    monitor.seconds += 0.1
    monitor.detection('smoke')
    assert len(monitor._request_publisher.messages) == 1
    monitor.monotonic_seconds += 0.01
    monitor.detection('smoke')
    assert len(monitor._request_publisher.messages) == 2
    assert len(monitor.logs) == 1
    monitor.phase('Verify', sequence=2)
    monitor.monotonic_seconds += 10.0
    monitor.detection('fire')
    assert len(monitor._request_publisher.messages) == 2


@pytest.mark.parametrize('terminal', [
    GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
])
def test_terminal_goals_cannot_rearm_from_an_obsolete_executing_status(monitor, terminal):
    monitor.status(goal_status())
    monitor.phase()
    monitor.status(goal_status(status=terminal))
    monitor.phase()
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection()

    assert monitor._request_publisher.messages == []


def test_missing_goal_is_retired_and_next_goal_accepts_only_new_images(monitor):
    monitor.status(goal_status())
    monitor.phase()
    monitor.status()
    monitor.status(goal_status())
    monitor.phase()
    monitor.detection()
    assert monitor._request_publisher.messages == []

    monitor.status(goal_status(goal=2, accepted=20.0))
    monitor.phase(goal=2)
    monitor.detection(acquired=19.999999999)
    assert monitor._request_publisher.messages == []
    monitor.seconds = 20.1
    monitor.detection()
    assert len(monitor._request_publisher.messages) == 1


def test_history_of_finished_goals_does_not_hide_new_executing_goal(monitor):
    monitor.status(
        goal_status(status=GoalStatus.STATUS_ABORTED),
        goal_status(goal=2, accepted=20.0),
    )
    monitor.phase(goal=2)
    monitor.seconds = 20.1
    monitor.detection('fire')

    assert len(monitor._request_publisher.messages) == 1


def test_conflicting_executing_goals_are_ignored(monitor):
    monitor.status(goal_status(), goal_status(goal=2))
    monitor.phase()
    monitor.detection()

    assert monitor._request_publisher.messages == []


def test_wait_for_request_subscriber_without_latching_an_undelivered_request(monitor):
    monitor.status(goal_status())
    monitor.phase()
    monitor._request_publisher.subscribers = 0
    monitor.detection()
    assert monitor._request_publisher.messages == []

    monitor._request_publisher.subscribers = 1
    monitor.detection('fire')
    assert len(monitor._request_publisher.messages) == 1


def test_uninitialized_ros_clock_and_clock_rewind_cannot_trigger_old_goal(monitor):
    monitor.status(goal_status())
    monitor.phase()
    monitor.seconds = 0.0
    monitor.detection(acquired=0.1)
    monitor.seconds = 1.0
    monitor.detection()

    assert monitor._request_publisher.messages == []


def test_zero_acceptance_stamp_still_requires_a_fresh_initialized_source_image(monitor):
    monitor.status(goal_status(accepted=0.0))
    monitor.phase()
    monitor.detection(acquired=0.0)
    assert monitor._request_publisher.messages == []
    monitor.detection()
    assert len(monitor._request_publisher.messages) == 1


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_source_image_on_goal_acceptance_tick_cannot_interrupt_the_goal(monitor, source):
    monitor.status(goal_status(accepted=20.0))
    monitor.phase()
    monitor.detection(source, acquired=20.0)
    assert monitor._request_publisher.messages == []

    monitor.seconds = 20.1
    monitor.detection(source)
    assert len(monitor._request_publisher.messages) == 1


def test_active_goal_requires_a_matching_exploration_phase(monitor):
    monitor.status(goal_status())
    monitor.detection()
    monitor.phase(goal=2)
    monitor.detection()
    assert monitor._request_publisher.messages == []
    monitor.phase()
    monitor.detection()
    assert len(monitor._request_publisher.messages) == 1


def test_phase_published_before_action_status_is_matched_when_status_arrives(monitor):
    monitor.phase(entered=19.5)
    monitor.detection()
    assert monitor._request_publisher.messages == []
    monitor.status(goal_status())
    monitor.detection()
    assert len(monitor._request_publisher.messages) == 1


def test_new_goal_phase_can_arrive_before_previous_goal_terminal_status(monitor):
    monitor.status(goal_status())
    monitor.phase()
    monitor.phase(goal=2, entered=19.5)
    monitor.status(
        goal_status(status=GoalStatus.STATUS_CANCELED),
        goal_status(goal=2, accepted=19.5),
    )
    monitor.detection()
    assert len(monitor._request_publisher.messages) == 1
    assert monitor._request_publisher.messages[0].goal_id == UUID(uuid=[2] * 16)


def test_verify_disables_requests_and_new_exploration_rearms_with_new_images(monitor):
    monitor.status(goal_status())
    monitor.phase(entered=19.1)
    monitor.detection()
    monitor.phase('Verify', sequence=2, entered=20.0)
    monitor.detection('fire')
    assert len(monitor._request_publisher.messages) == 1

    monitor.phase(sequence=3, entered=20.1)
    monitor.seconds = 20.2
    monitor.detection(acquired=20.0)
    monitor.detection(acquired=20.1)
    assert len(monitor._request_publisher.messages) == 1
    monitor.detection('fire')
    assert len(monitor._request_publisher.messages) == 2
    assert all(message.state == 'Verify' for message in monitor._request_publisher.messages)
    assert all(message.goal_id == UUID(uuid=[1] * 16)
               for message in monitor._request_publisher.messages)


def test_verify_cannot_be_overridden_by_old_phase_sequence_or_other_goal(monitor):
    monitor.status(goal_status())
    monitor.phase('Verify', sequence=3)
    monitor.phase(sequence=2)
    monitor.phase(sequence=3)
    monitor.phase(goal=2, sequence=99)
    monitor.detection('fire')
    assert monitor._request_publisher.messages == []


def test_duplicate_or_older_exploration_phase_cannot_rearm_sent_request(monitor):
    monitor.status(goal_status())
    monitor.phase(sequence=3)
    monitor.detection()
    monitor.phase(sequence=3)
    monitor.phase(sequence=2)
    monitor.phase(goal=2, sequence=99)
    monitor.detection()
    assert len(monitor._request_publisher.messages) == 1
