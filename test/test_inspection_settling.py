"""Check physical settling independently of FCU mode acknowledgements."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Pose, Quaternion
import kv_executive.inspection_settling as settling
from kv_executive.inspection_settling import InspectionSettlingMixin
from nav_msgs.msg import Odometry
import pytest


def rotation(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy,
                      y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy,
                      w=cr * cp * cy + sr * sp * sy)


class Harness(InspectionSettlingMixin):
    def __init__(self):
        self.wall = 100.0
        self.epoch = self.wall
        self.clock_rate = 1.0
        self.clock_warp = lambda elapsed: 0.0
        self.period = 0.05
        self.freeze_odometry = False
        self.mode = 'AUTO.LOITER'
        self.config = {'verify_settle_timeout': 15.0}
        self.pose_at = lambda elapsed: Pose(orientation=rotation())
        self.messages = []
        self.loiter_calls = 0
        self.alignment_calls = 0
        self.map_calls = 0
        self.align = lambda: True
        self.map_wait = lambda context, guard: None
        self.twist = Odometry().twist
        self._stop = SimpleNamespace(wait=self.wait)
        self._verify_wall_observation_context = None

    def source_time(self, elapsed):
        return 10.0 + elapsed * self.clock_rate + self.clock_warp(elapsed)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=round(self.source_time(self.wall - self.epoch) * 1e9)))

    def _snapshot(self, require_armed=False):
        assert require_armed
        elapsed = math.floor((self.wall - self.epoch + 1e-8) / self.period) * self.period
        if self.freeze_odometry:
            elapsed = 0.0
        odometry = Odometry()
        stamp_ns = round(self.source_time(elapsed) * 1e9)
        odometry.header.stamp.sec, odometry.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        odometry.header.frame_id = 'local'
        odometry.pose.pose = self.pose_at(elapsed)
        odometry.twist = deepcopy(self.twist)
        return SimpleNamespace(mode=self.mode), odometry

    def _check_running(self):
        return None

    def _report(self, message):
        self.messages.append(message)

    def wait(self, seconds):
        self.wall += seconds
        return False

    def _loiter(self):
        self.loiter_calls += 1
        self.mode = 'AUTO.LOITER'

    def _verify_align_fire_heading(self, guard=None):
        self.alignment_calls += 1
        if guard:
            guard()
        return self.align()

    def _verify_wait_for_wall_map(self, context, guard=None):
        self.map_calls += 1
        self.map_wait(context, guard)
        context['map_wait_complete'] = True
        return context['map_after_epoch']


@pytest.fixture
def vehicle(monkeypatch):
    harness = Harness()
    monkeypatch.setattr(settling.time, 'monotonic', lambda: harness.wall)
    return harness


def test_loiter_acknowledgement_does_not_bypass_braking(vehicle):
    def pose(elapsed):
        result = Pose(orientation=rotation())
        result.position.x = 0.3 * min(elapsed, 1.0)
        return result

    vehicle.pose_at = pose
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch >= 2.0
    context = vehicle._verify_wall_observation_context
    assert context['pose'].position.x == pytest.approx(0.3)
    assert context['linear_speed'] <= 0.1
    assert context['settled_ns'] >= 12 * 10**9
    assert vehicle.map_calls == 1


def test_angular_oscillation_never_counts_as_a_stationary_dwell(vehicle):
    vehicle.pose_at = lambda elapsed: Pose(orientation=rotation(
        roll=0.12 * math.sin(elapsed * 8.0)))
    assert not vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch >= 15.0
    assert vehicle._verify_wall_observation_context is None
    assert vehicle.map_calls == 0


def test_stable_nonlevel_attitude_is_allowed(vehicle):
    vehicle.pose_at = lambda elapsed: Pose(orientation=rotation(
        roll=math.radians(20), pitch=math.radians(-15), yaw=math.radians(30)))
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch == pytest.approx(1.0, abs=0.06)
    assert vehicle._verify_wall_observation_ready()


def test_default_zero_twist_cannot_hide_pose_derived_motion(vehicle):
    def pose(elapsed):
        result = Pose(orientation=rotation())
        result.position.y = elapsed * 0.2
        return result

    vehicle.pose_at = pose
    assert not vehicle._verify_prepare_wall_observation()
    assert vehicle._verify_wall_observation_context is None


def test_valid_twist_can_veto_apparently_stationary_pose(vehicle):
    vehicle.twist.twist.linear.x = 0.2
    vehicle.twist.covariance[0] = 0.01
    assert not vehicle._verify_prepare_wall_observation()
    assert 'speed=0.200' in vehicle._verify_wall_settle_reason


def test_frozen_source_clock_cannot_prove_settling(vehicle):
    vehicle.clock_rate = 0.0
    assert not vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch == pytest.approx(5.0, abs=0.06)
    assert vehicle._verify_wall_settle_reason == 'source clock stopped progressing'


def test_duplicate_odometry_frames_cannot_complete_a_dwell(vehicle):
    vehicle.freeze_odometry = True
    assert not vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch >= 15.0
    assert vehicle.map_calls == 0


def test_odometry_gap_requires_a_new_continuous_dwell_after_frames_resume(vehicle):
    original = vehicle._snapshot

    def interrupted(require_armed=False):
        elapsed = vehicle.wall - vehicle.epoch
        if 0.25 <= elapsed < 1.6:
            vehicle.freeze_odometry = True
        else:
            vehicle.freeze_odometry = False
        return original(require_armed=require_armed)

    vehicle._snapshot = interrupted
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch >= 2.6


def test_slow_but_progressing_clock_uses_one_wall_second_dwell(vehicle):
    vehicle.clock_rate = 0.1
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch == pytest.approx(1.0, abs=0.06)
    assert vehicle._verify_wall_observation_context['settled_ns'] < 11 * 10**9


def test_clock_rewind_discards_incomplete_dwell(vehicle):
    vehicle.clock_warp = lambda elapsed: -2.0 if elapsed >= 0.3 else 0.0
    assert not vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch < 1.0
    assert vehicle._verify_wall_settle_reason == 'source clock moved backwards'


def test_context_reuse_keeps_its_epoch_and_does_not_repeat_map_wait(vehicle):
    assert vehicle._verify_prepare_wall_observation()
    context = vehicle._verify_wall_observation_context
    vehicle.wait(0.2)
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle._verify_wall_observation_context is context
    assert vehicle.map_calls == 1
    assert vehicle.alignment_calls == 2


def test_report_settling_keeps_physical_dwell_without_waiting_for_a_map(vehicle):
    def never_query(context, guard):
        raise AssertionError('Retained-coordinate reporting must not require a new map')

    vehicle.map_wait = never_query
    assert vehicle._verify_prepare_wall_observation(align_fire=False, wait_for_map=False)
    assert vehicle.wall - vehicle.epoch >= 1.0
    assert vehicle.map_calls == 0
    assert vehicle.alignment_calls == 0
    assert not vehicle._verify_wall_observation_context['map_wait_complete']
    assert vehicle._verify_wall_observation_ready()


def test_normal_ranging_after_report_settling_waits_once_for_its_map(vehicle):
    assert vehicle._verify_prepare_wall_observation(wait_for_map=False)
    context = vehicle._verify_wall_observation_context
    assert vehicle.map_calls == 0
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle._verify_wall_observation_context is context
    assert vehicle.map_calls == 1
    assert context['map_wait_complete']
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.map_calls == 1


def test_skipping_wall_map_wait_does_not_accept_motion_or_a_frozen_clock(vehicle):
    vehicle.clock_rate = 0.0
    assert not vehicle._verify_prepare_wall_observation(wait_for_map=False)
    assert vehicle.map_calls == 0
    assert vehicle._verify_wall_observation_context is None


def test_report_settling_still_obeys_the_original_inspection_deadline(vehicle):
    def original_deadline():
        if vehicle.wall - vehicle.epoch >= 0.3:
            raise RuntimeError('Original inspection deadline expired')

    with pytest.raises(RuntimeError, match='Original inspection deadline'):
        vehicle._verify_prepare_wall_observation(
            wait_for_map=False, guard=original_deadline)
    assert vehicle.map_calls == 0
    assert vehicle._verify_wall_observation_context is None


def test_pending_wall_recheck_reuses_settled_pose_without_turning_again(vehicle):
    assert vehicle._verify_prepare_wall_observation()
    context = vehicle._verify_wall_observation_context
    vehicle.align = lambda: False
    assert vehicle._verify_prepare_wall_observation(align_fire=False)
    assert vehicle._verify_wall_observation_context is context
    assert vehicle.alignment_calls == 1
    assert vehicle.map_calls == 1


@pytest.mark.parametrize('motion', ['translate', 'rotate'])
def test_new_motion_invalidates_a_ready_context_without_waiting(vehicle, motion):
    assert vehicle._verify_prepare_wall_observation()
    anchor = vehicle.wall

    def pose(elapsed):
        result = Pose(orientation=rotation(yaw=math.radians(10) if motion == 'rotate' else 0))
        result.position.x = 0.2 if motion == 'translate' else 0.0
        return result

    vehicle.pose_at = pose
    vehicle.wait(0.05)
    assert not vehicle._verify_wall_observation_ready()
    assert vehicle.wall == pytest.approx(anchor + 0.05)
    assert vehicle._verify_wall_observation_context is None


def test_slow_accumulated_drift_invalidates_old_observation_pose(vehicle):
    assert vehicle._verify_prepare_wall_observation()
    drift_start = vehicle.wall - vehicle.epoch

    def pose(elapsed):
        result = Pose(orientation=rotation())
        result.position.x = max(0.0, elapsed - drift_start) * 0.05
        return result

    vehicle.pose_at = pose
    for _ in range(22):
        vehicle.wait(0.1)
        if not vehicle._verify_wall_observation_ready():
            break
    assert vehicle._verify_wall_observation_context is None
    assert 'drifted' in vehicle._verify_wall_settle_reason


@pytest.mark.parametrize('gap', [1.1, 3.0])
@pytest.mark.parametrize('net_motion', [0.0, 0.02])
def test_fresh_sample_after_observation_gap_requires_new_dwell_and_map(
        vehicle, gap, net_motion):
    map_epochs = []

    def map_after_dwell(context, guard):
        if guard:
            guard()
        vehicle.wait(0.05)
        map_epochs.append(context['settled_ns'])
        context['map_after_epoch'] = True

    vehicle.map_wait = map_after_dwell
    assert vehicle._verify_prepare_wall_observation()
    previous = vehicle._verify_wall_observation_context

    def pose(elapsed):
        result = Pose(orientation=rotation())
        result.position.x = net_motion
        return result

    vehicle.pose_at = pose
    vehicle.wait(gap)
    resumed_at = vehicle.wall
    # The resumed source sample is fresh and net displacement is below the
    # drift threshold. Neither proves continuous stability through the gap.
    assert not vehicle._verify_wall_observation_ready()
    assert vehicle.wall == resumed_at
    assert vehicle._verify_wall_observation_context is None
    assert 'odometry update gap' in vehicle._verify_wall_settle_reason
    assert vehicle._verify_prepare_wall_observation()
    current = vehicle._verify_wall_observation_context
    assert vehicle.wall - resumed_at >= 1.0
    assert current is not previous and current['settled_ns'] > previous['settled_ns']
    assert current['pose'].position.x == pytest.approx(net_motion)
    assert current['map_after_epoch'] and current['map_wait_complete']
    assert vehicle.map_calls == 2
    assert map_epochs == [previous['settled_ns'], current['settled_ns']]


def test_stale_source_odometry_invalidates_ready_context(vehicle):
    assert vehicle._verify_prepare_wall_observation()
    vehicle.freeze_odometry = True
    vehicle.wait(2.1)
    assert not vehicle._verify_wall_observation_ready()
    assert vehicle._verify_wall_observation_context is None


def test_alignment_happens_before_dwell_and_unavailable_heading_never_probes(vehicle):
    def align():
        vehicle.wait(0.4)
        return True

    vehicle.align = align
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.wall - vehicle.epoch >= 1.4
    vehicle._verify_wall_observation_context = None
    vehicle.align = lambda: False
    assert not vehicle._verify_prepare_wall_observation()
    assert vehicle.map_calls == 1


def test_optional_map_wait_timeout_keeps_physically_settled_navigation_available(vehicle):
    def wait_for_map(context, guard):
        for _ in range(10):
            if guard:
                guard()
            vehicle.wait(0.1)
            assert vehicle._verify_wall_observation_ready()

    vehicle.map_wait = wait_for_map
    assert vehicle._verify_prepare_wall_observation()
    context = vehicle._verify_wall_observation_context
    assert context['map_wait_complete'] and not context['map_after_epoch']
    assert vehicle._verify_wall_observation_ready()


def test_prepare_remains_responsive_to_external_deadline_guard(vehicle):
    def guard():
        if vehicle.wall - vehicle.epoch >= 0.3:
            raise RuntimeError('original near-wall deadline expired')

    with pytest.raises(RuntimeError, match='original near-wall deadline'):
        vehicle._verify_prepare_wall_observation(guard=guard)
    assert vehicle._verify_wall_observation_context is None


def test_offboard_mode_is_returned_to_loiter_before_dwell(vehicle):
    vehicle.mode = 'OFFBOARD'
    assert vehicle._verify_prepare_wall_observation()
    assert vehicle.loiter_calls == 1
    assert vehicle._verify_wall_observation_context['sample']['mode'] == 'AUTO.LOITER'
