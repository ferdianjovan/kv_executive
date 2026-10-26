"""Deterministic Verify decisions and perception guards without a flight controller."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from kv_executive.mission_geometry import quaternion_from_yaw, yaw_from_quaternion
import kv_executive.verification as verification
from kv_executive.verification import (
    VerificationObservationLost, VerifyController, VerifyStateMachine,
)
from kv_perception.msg import SmokeDetection
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Header
from yasmin import Blackboard


class Harness:
    def __init__(self):
        self.now = 100.0
        self.config = {}
        self.events = []
        self.odometry = Odometry()
        self.odometry.pose.pose.position.z = 6.0
        self.odometry.pose.pose.orientation = quaternion_from_yaw(0.0)
        self.on_motion = lambda: None
        self.on_wait = lambda: None
        self._stop = SimpleNamespace(wait=self.wait)
        self._verify = None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9)))

    def wait(self, duration):
        self.now += duration
        self.on_wait()
        return False

    def _snapshot(self, require_armed=False):
        assert require_armed
        return None, deepcopy(self.odometry)

    def _check_running(self):
        return None

    def _verify_parent_entry(self, name):
        self.events.append(('parent', name))

    def _report(self, message, error=False):
        self.events.append(('status', message))

    def _verify_plan_to(self, target, guard=None, speed=None):
        if guard:
            guard()
        self.events.append(('plan', deepcopy(target)))
        self.on_motion()
        if guard:
            guard()
        self.odometry.pose.pose = deepcopy(target)

    def _verify_rotate(self, yaw, guard=None, hold_duration=None):
        if guard:
            guard()
        self.events.append(('rotate', yaw))
        self.on_motion()
        if guard:
            guard()
        self.odometry.pose.pose.orientation = quaternion_from_yaw(yaw)

    def _verify_hold(self, seconds, guard=None):
        self.events.append(('hold', seconds))
        self.on_motion()
        if guard:
            guard()

    def _verify_record_fire(self, fire, bearing):
        self.events.append(('record', deepcopy(fire), bearing))


@pytest.fixture
def system(monkeypatch):
    executive = Harness()
    monkeypatch.setattr(verification.time, 'monotonic', lambda: executive.now)
    controller = VerifyController(executive)
    executive._verify = controller
    return executive, controller


def header(now, frame='camera'):
    result = Header(frame_id=frame)
    result.stamp.sec = int(now)
    result.stamp.nanosec = int((now % 1.0) * 1e9)
    return result


def smoke(executive, controller, **changes):
    executive.now += 0.01
    message = SmokeDetection(
        header=header(executive.now), detected=True, status=SmokeDetection.CONFIRMED,
        background_ready=True, coverage=0.02, regions=[1], region=1,
        yaw_correction_rad=0.0, vertical_correction_rad=0.0,
    )
    for key, value in changes.items():
        setattr(message, key, value)
    controller.smoke_callback(message)
    return message


def fire(executive, controller, **changes):
    executive.now += 0.01
    message = SimpleNamespace(
        header=header(executive.now, 'thermal'), detected=True, status=2, CONFIRMED=2,
        image_width=100, image_height=100, center_x=50.0, center_y=50.0,
        area_px=100, yaw_correction_rad=0.0, vertical_correction_rad=0.0,
        source_count=2, track_id=1,
    )
    for key, value in changes.items():
        setattr(message, key, value)
    controller.fire_callback(message)
    return message


def stable_fire(executive, controller, **changes):
    fire(executive, controller, **changes)
    for _ in range(2):
        executive.now += 0.75
        result = fire(executive, controller, **changes)
    return result


def test_accept_smoke_from_any_region_and_return_owned_copy(system):
    executive, controller = system
    message = smoke(executive, controller, region=0, regions=[])
    assert controller.fresh_smoke() is not None
    controller.fresh_smoke().coverage = 0.9
    assert controller.fresh_smoke().coverage == message.coverage


@pytest.mark.parametrize('stamp_delta', [-10.0, 3.0, -100.01])
def test_reject_uninitialized_stale_and_future_images(system, stamp_delta):
    executive, controller = system
    smoke(executive, controller, header=header(executive.now + stamp_delta))
    assert controller.fresh_smoke() is None


def test_expire_observation_without_negative_detector_frame(system):
    executive, controller = system
    smoke(executive, controller)
    executive.now += 2.1
    assert controller.fresh_smoke() is None


def test_fire_needs_distinct_source_images_and_elapsed_persistence(system):
    executive, controller = system
    message = fire(executive, controller)
    for _ in range(10):
        controller.fire_callback(message)
    executive.now += 1.6
    assert controller.stable_fire() is None
    fire(executive, controller)
    assert controller.stable_fire() is None
    fire(executive, controller)
    assert controller.stable_fire() is not None


@pytest.mark.parametrize('changes', [
    {'track_id': 2}, {'center_x': 80.0}, {'area_px': 500}, {'detected': False},
])
def test_replaced_or_lost_fire_restarts_persistence(system, changes):
    executive, controller = system
    stable_fire(executive, controller)
    assert controller.stable_fire() is not None
    fire(executive, controller, **changes)
    assert controller.stable_fire() is None


def test_gap_and_camera_change_restart_fire_persistence(system):
    executive, controller = system
    stable_fire(executive, controller)
    executive.now += 2.1
    fire(executive, controller)
    assert controller.stable_fire() is None
    stable_fire(executive, controller)
    fire(executive, controller, header=header(executive.now + 0.01, 'other'))
    assert controller.stable_fire() is None


def test_approach_step_shrinks_with_apparent_size_and_remains_bounded(system):
    executive, controller = system
    small = smoke(executive, controller, coverage=0.0)
    medium = smoke(executive, controller, coverage=0.1)
    large = smoke(executive, controller, coverage=0.5)
    assert controller.approach_distance(small) == 20.0
    assert 5.0 < controller.approach_distance(medium) < 20.0
    assert controller.approach_distance(large) == 5.0


def test_approach_records_reference_and_plans_toward_camera_bearing(system):
    executive, controller = system
    smoke(executive, controller, yaw_correction_rad=math.pi / 2)
    blackboard = Blackboard()
    assert controller.approach_smoke(blackboard) == 'reacquire'
    planned = next(event[1] for event in executive.events if event[0] == 'plan')
    assert planned.position.x == pytest.approx(0.0, abs=1e-5)
    assert 5.0 <= planned.position.y <= 20.0
    assert planned.position.z == 6.0
    assert blackboard['verify_reference_pose'].position.x == 0.0
    assert blackboard['verify_reference_pose'].position.y == 0.0


def test_approach_stops_on_observation_loss_during_planned_motion(system):
    executive, controller = system
    smoke(executive, controller)
    executive.on_motion = lambda: smoke(executive, controller, detected=False)
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == 0.0


def test_all_quadrants_prevent_forward_approach(system):
    executive, controller = system
    smoke(executive, controller, regions=[1, 2, 3, 4])
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert not any(event[0] == 'plan' for event in executive.events)
    assert controller.reacquire(Blackboard()) == 'search'


def test_reacquire_prioritizes_persistent_fire_without_smoke(system):
    executive, controller = system
    stable_fire(executive, controller)
    assert controller.reacquire(Blackboard()) == 'fire'
    assert not executive.events


def test_reacquire_centers_smoke_before_another_approach(system):
    executive, controller = system
    smoke(executive, controller, yaw_correction_rad=math.radians(20))
    executive.on_motion = lambda: smoke(executive, controller)
    executive.on_wait = lambda: smoke(executive, controller)
    assert controller.reacquire(Blackboard()) == 'approach'
    assert next(e[1] for e in executive.events if e[0] == 'rotate') == pytest.approx(
        math.radians(20))


def test_reacquire_restores_reference_altitude_and_bounded_heading_search(system):
    executive, controller = system
    reference = Pose()
    reference.position.z = 8.0
    reference.orientation = quaternion_from_yaw(0.5)
    board = Blackboard()
    board['verify_reference_pose'] = reference
    assert controller.reacquire(board) == 'explore'
    plans = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(plans) == 1
    assert plans[0].position.z == 8.0
    assert plans[0].position.x == plans[0].position.y == 0.0
    assert len([e for e in executive.events if e[0] == 'rotate']) == 4
    assert executive.now < 116.0


def test_centered_fire_reports_count_then_turns_away_and_plans_retreat(system):
    executive, controller = system
    stable_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'explore'
    records = [event for event in executive.events if event[0] == 'record']
    assert records[0][1].source_count == 2
    assert executive.events.index(records[0]) < next(
        index for index, event in enumerate(executive.events) if event[0] == 'plan')
    target = next(e[1] for e in executive.events if e[0] == 'plan')
    assert target.position.x == pytest.approx(-10.0)
    assert target.position.z == 6.0
    assert abs(yaw_from_quaternion(target.orientation)) == pytest.approx(math.pi)


def test_fire_altitude_correction_is_bounded_and_planned(system):
    executive, controller = system
    stable_fire(executive, controller, vertical_correction_rad=0.2)

    def centered():
        fire(executive, controller)

    executive.on_motion = centered
    executive.on_wait = centered
    assert controller.center_fire(Blackboard()) == 'explore'
    targets = [e[1] for e in executive.events if e[0] == 'plan']
    assert targets[0].position.z == 6.5
    assert targets[0].position.x == 0.0


def test_fire_loss_interrupts_centering_and_suppresses_report(system):
    executive, controller = system
    stable_fire(executive, controller, yaw_correction_rad=0.3)
    executive.on_motion = lambda: fire(executive, controller, detected=False)
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert not any(e[0] == 'record' for e in executive.events)


def test_creep_requires_fresh_smoke_and_has_distance_limit(system):
    executive, controller = system
    stable_fire(executive, controller, area_px=10)
    smoke(executive, controller)
    executive.on_motion = lambda: fire(executive, controller, area_px=10)
    executive.on_wait = lambda: fire(executive, controller, area_px=10)
    assert controller.center_fire(Blackboard()) == 'explore'
    targets = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(targets) == 5  # Four short creeps and one retreat.
    assert targets[-2].position.x == pytest.approx(3.0)


def test_all_quadrants_block_fire_creep_and_retreat_before_reacquisition(system):
    executive, controller = system
    stable_fire(executive, controller, area_px=10)
    smoke(executive, controller, regions=[1, 2, 3, 4])
    assert controller.center_fire(Blackboard()) == 'reacquire'
    targets = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(targets) == 1
    assert targets[0].position.x == pytest.approx(-10.0)
    assert not any(e[0] == 'record' for e in executive.events)
    assert controller.stable_fire() is None


def test_level_search_returns_heading_then_retreats_up_and_faces_smoke(system):
    executive, controller = system
    assert controller.search_fire(Blackboard()) == 'approach'
    yaws = [e[1] for e in executive.events if e[0] == 'rotate']
    assert yaws == pytest.approx([math.pi / 3, -math.pi / 3, 0, math.pi, 0])
    target = next(e[1] for e in executive.events if e[0] == 'plan')
    assert target.position.x == pytest.approx(-10.0)
    assert target.position.z == 16.0
    assert target.orientation.x == target.orientation.y == 0.0


def test_persistent_fire_interrupts_search_before_escape(system):
    executive, controller = system
    executive.on_motion = lambda: stable_fire(executive, controller)
    assert controller.search_fire(Blackboard()) == 'fire'
    assert not any(e[0] == 'plan' for e in executive.events)


def test_approach_limit_holds_instead_of_unbounded_smoke_chasing(system):
    executive, controller = system
    smoke(executive, controller)
    board = Blackboard()
    board['verify_approaches'] = 8
    with pytest.raises(RuntimeError, match='approach limit'):
        controller.approach_smoke(board)
    assert not any(e[0] == 'plan' for e in executive.events)


def test_search_retreat_cycles_are_bounded_even_when_smoke_stays_full(system):
    executive, controller = system
    smoke(executive, controller, regions=[1, 2, 3, 4])
    board = Blackboard()
    board['verify_approaches'] = 8
    with pytest.raises(RuntimeError, match='approach limit'):
        controller.approach_smoke(board)
    assert not any(e[0] == 'plan' for e in executive.events)


def test_verify_machine_declares_four_substates(system):
    executive, _ = system
    machine = VerifyStateMachine(executive)
    assert set(machine.get_states()) == {
        'APPROACH_SMOKE', 'REACQUIRE', 'CENTER_FIRE', 'SEARCH_FIRE',
    }


def test_smoke_guard_rejects_missing_or_all_quadrants(system):
    executive, controller = system
    with pytest.raises(VerificationObservationLost):
        controller._smoke_guard()
    smoke(executive, controller, regions=[1, 2, 3, 4])
    with pytest.raises(VerificationObservationLost):
        controller._smoke_guard()


def test_reacquire_waits_for_visible_fire_persistence_before_rotating(system):
    executive, controller = system
    fire(executive, controller)
    executive.on_wait = lambda: fire(executive, controller)
    assert controller.reacquire(Blackboard()) == 'fire'
    assert not any(e[0] in {'rotate', 'plan'} for e in executive.events)


@pytest.mark.parametrize('failure, expected', [
    ('cancel', 'canceled'), ('return', 'return_requested'), ('error', 'aborted'),
])
def test_four_state_machine_propagates_cancel_return_and_failure(system, failure, expected):
    from kv_executive.mission_executive import (
        MissionCanceled, MissionError, MissionStateRequested,
    )
    executive, controller = system
    errors = {
        'cancel': MissionCanceled('operator cancellation'),
        'return': MissionStateRequested('Return'),
        'error': MissionError('planner failed'),
    }

    def fail(blackboard):
        raise errors[failure]

    controller.approach_smoke = fail
    assert VerifyStateMachine(executive)(Blackboard()) == expected


def test_smoke_correction_does_not_reuse_a_cached_angle_when_camera_stalls(system):
    executive, controller = system
    smoke(executive, controller, yaw_correction_rad=math.radians(6.0))
    assert controller.reacquire(Blackboard()) == 'explore'
    assert len([event for event in executive.events if event[0] == 'rotate']) == 1
    assert not any(event[0] == 'plan' for event in executive.events)
    assert executive.now < 103.0


def test_fire_correction_requires_post_motion_frame_before_reporting_or_moving_again(system):
    executive, controller = system
    stable_fire(executive, controller, vertical_correction_rad=0.1)
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert len([event for event in executive.events if event[0] == 'plan']) == 1
    assert not any(event[0] == 'record' for event in executive.events)


def test_waiting_for_post_motion_image_honors_cancellation(system):
    from kv_executive.mission_executive import MissionCanceled
    executive, controller = system
    smoke(executive, controller, yaw_correction_rad=0.2)

    def cancel():
        raise MissionCanceled('operator cancellation')

    executive.on_wait = cancel
    with pytest.raises(MissionCanceled):
        controller.reacquire(Blackboard())
