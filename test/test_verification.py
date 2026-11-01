"""Deterministic Verify decisions and perception guards without a flight controller."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Point, PointStamped
from kv_executive.mission_geometry import quaternion_from_yaw
import kv_executive.verification as verification
from kv_executive.verification import (
    VerifyController, VerifyStateMachine,
)
from kv_perception.msg import SmokeDetection
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Header
from yasmin import Blackboard


class Harness:
    def __init__(self):
        self.now = 100.0
        # Observation tests start at a measured wall standoff. Flight-policy
        # cases explicitly move the wall farther away and enable approach legs.
        self.config = {'verify_fire_approach_max_steps': 0, 'verify_smoke_standoff': 10.0}
        self.wall_x = 10.5
        self.events = []
        self.odometry = Odometry()
        self.odometry.pose.pose.position.z = 6.0
        self.odometry.pose.pose.orientation = quaternion_from_yaw(0.0)
        self.on_motion = lambda: None
        self.on_wait = lambda: None
        self._stop = SimpleNamespace(wait=self.wait)
        self._verify = None
        self._verify_last_geometry_stamp_ns = int(self.now * 1e9)
        self.map_available = True
        self.map_stamp_ns = None
        self.target_geometry = {
            'point': Point(x=20.0),
            'planning_point': PointStamped(
                header=Header(frame_id='map'), point=Point(x=20.0)),
            'range': 20.0,
            'uncertainty': 0.25,
            'method': 'thermal_multiview_bearings',
            'target_valid': True,
            'source_stamp': header(self.now).stamp,
            'ground_z': 0.0,
        }

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

    def _verify_navigation_preflight(self):
        self.events.append(('preflight',))
        return True

    def _verify_fresh_map_stamp(self, timeout=None):
        self.events.append(('map_poll',))
        if not self.map_available:
            return 0
        return int(self.now * 1e9) if self.map_stamp_ns is None else self.map_stamp_ns

    def _report(self, message, error=False):
        self.events.append(('status', message))

    def _verify_plan_to(self, target, guard=None, speed=None, allow_plan_failure=False):
        if guard:
            guard()
        self.events.append(('plan', deepcopy(target)))
        self.on_motion()
        if guard:
            guard()
        self.odometry.pose.pose = deepcopy(target)
        return True

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
        deadline = self.now + seconds
        while self.now < deadline:
            self.wait(min(0.05, deadline - self.now))
            if guard:
                guard()

    def _verify_center_fire_wall(self, camera_frame):
        distance = self.wall_x - self.odometry.pose.pose.position.x
        return {'origin': deepcopy(self.odometry.pose.pose.position), 'axis': (1.0, 0.0, 0.0),
                'mean': distance, 'nearest': distance, 'uncertainty': 0.2,
                'camera_frame': camera_frame}

    def _verify_opportunistic_wall(self, camera_frame, bearing):
        self.events.append(('wall_probe', camera_frame, bearing))
        return None

    def _verify_smoke_navigation_wall(self, geometry):
        return None

    def _verify_smoke_wall_at_standoff(self, wall):
        if wall is None:
            return False
        current = self.odometry.pose.pose.position
        distance = wall['nearest'] - sum(
            (getattr(current, a) - getattr(wall['origin'], a)) * wall['axis'][i]
            for i, a in enumerate('xyz'))
        return verification.smoke_wall_standoff_reached(
            {**wall, 'nearest': distance}, self.config['verify_smoke_standoff'])

    def _verify_record_fire(self, fire, bearing):
        self.events.append(('record', deepcopy(fire), bearing))

    def _verify_target_geometry(self, message, source='smoke'):
        self.events.append(('geometry', source, deepcopy(message)))
        geometry = deepcopy(self.target_geometry)
        geometry['source_stamp'] = deepcopy(message.header.stamp)
        geometry['source'] = source
        if geometry['point'] is not None:
            point = geometry['point']
            current = self.odometry.pose.pose.position
            geometry['range'] = math.dist(
                (point.x, point.y, point.z), (current.x, current.y, current.z))
        return geometry

    def _verify_smoke_boundary_reached(self, geometry):
        return bool(geometry and geometry.get('source') == 'smoke'
                    and geometry.get('plume_candidate')
                    and geometry.get('obstacle_range', math.inf) <= 10.0)

    def _verify_recovery_plan_to(self, target, geometry=None, guard=None,
                                 preserve_requested_altitude=False):
        self.events.append(('recovery_plan', deepcopy(target), preserve_requested_altitude))
        return self._verify_plan_to(target, guard=guard, allow_plan_failure=True)

    def _verify_navigation_viewpoint(self, bearing, index=0, guard=None,
                                     geometry=None, maximum_forward=None, forward_only=False,
                                     vertical_offset=0.0):
        self.events.append(('navigation', bearing, deepcopy(geometry)))
        target = deepcopy(self.odometry.pose.pose)
        forward = 3.0 if maximum_forward is None else min(3.0, maximum_forward)
        target.position.x += forward * math.cos(bearing)
        target.position.y += forward * math.sin(bearing)
        target.position.z += vertical_offset
        target.orientation = quaternion_from_yaw(bearing)
        return self._verify_recovery_plan_to(
            target, geometry=geometry, guard=guard,
            preserve_requested_altitude=vertical_offset != 0.0)

    def _verify_report_inspection(self, geometry, fire_message=None, attempts=1, wall=None):
        self.events.append((
            'inspection_report', deepcopy(geometry), deepcopy(fire_message), attempts,
        ))
        return SimpleNamespace(fire_found=fire_message is not None)

    def _verify_complete_site(self, report, message=None, source=None):
        self.events.append(('completed_site', report))


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


def _qualified_fire(executive, controller, **changes):
    metadata = {'confirmation_frames': 3, 'persistence_sec': 1.5, 'track_id': 1}
    metadata.update(changes)
    return fire(executive, controller, **metadata)


def _approached_board():
    """Set up a later Verify state after a completed initial smoke approach."""
    board = Blackboard()
    board['verify_smoke_approach_completed'] = True
    return board


def _complete_search(controller, blackboard):
    """Advance successful short SEARCH_FIRE legs to the next distinct substate."""
    for _ in range(20):
        outcome = controller.search_fire(blackboard)
        if outcome != 'search':
            return outcome
    pytest.fail('SEARCH_FIRE did not finish after twenty committed approach legs')


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


def test_observation_diagnostics_distinguish_no_messages_from_missing_confirmation(system):
    executive, controller = system

    assert 'no messages received' in controller.observation_diagnostics()

    smoke(executive, controller, status=SmokeDetection.CONFIRMING)
    details = controller.observation_diagnostics()

    assert 'detection is not confirmed' in details
    assert 'received=1' in details and 'accepted=0' in details
    assert f'status={SmokeDetection.CONFIRMING}' in details
    assert 'image_age=' in details and 'receive_gap=' in details
    assert controller.fresh_smoke() is None


def test_observation_diagnostics_explain_stale_source_images_without_accepting_them(system):
    executive, controller = system
    executive.now += 10.0
    smoke(executive, controller, header=header(executive.now - 3.0))

    details = controller.observation_diagnostics()

    assert 'source image is too old' in details
    assert 'received=1' in details and 'accepted=0' in details
    assert 'detected=True' in details
    assert 'image_age=3.' in details and 'receive_gap=0.00s' in details
    assert controller.fresh_smoke() is None


def test_negative_smoke_frame_immediately_clears_guidance_and_explains_its_status(system):
    executive, controller = system
    smoke(executive, controller)
    assert controller.fresh_smoke() is not None

    smoke(executive, controller, detected=False, status=SmokeDetection.CONFIRMING)

    assert controller.fresh_smoke() is None
    details = controller.observation_diagnostics()
    assert 'detected=false' in details
    assert 'received=2' in details and 'accepted=1' in details
    assert f'status={SmokeDetection.CONFIRMING}' in details
    assert 'detected=False' in details


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


def test_confirmed_producer_persistence_does_not_require_a_second_consumer_delay(system):
    executive, controller = system
    started = executive.now
    message = _qualified_fire(executive, controller)

    assert controller.stable_fire() is not None
    assert controller._stamp_ns(controller.stable_fire()) == controller._stamp_ns(message)
    assert executive.now - started < 0.1


@pytest.mark.parametrize('changes', [
    {'persistence_sec': 0.0}, {'persistence_sec': 1.49}, {'persistence_sec': math.nan},
    {'persistence_sec': math.inf}, {'confirmation_frames': 0}, {'confirmation_frames': 2},
    {'track_id': 0}, {'detected': False}, {'status': 1}, {'area_px': 0},
])
def test_incomplete_or_invalid_producer_confirmation_cannot_skip_consumer_checks(system, changes):
    executive, controller = system
    _qualified_fire(executive, controller, **changes)

    assert controller.stable_fire() is None


def test_legacy_fire_negative_still_resets_consumer_persistence(system):
    executive, controller = system
    stable_fire(executive, controller)
    fire(executive, controller, detected=False)
    assert controller.fresh_fire() is None and controller.stable_fire() is None

    fire(executive, controller)

    assert controller.stable_fire() is None


def test_qualified_fire_negative_clears_guidance_without_erasing_recent_state_intent(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    fire(executive, controller, detected=False)

    assert controller.fresh_fire() is None and controller.stable_fire() is None
    assert controller._inspection_fire_event(Blackboard())


def test_expired_qualified_fire_event_cannot_reactivate_from_a_duplicate_source_image(system):
    executive, controller = system
    message = _qualified_fire(executive, controller)
    fire(executive, controller, detected=False)
    executive.now += controller.config['verify_reacquire_observation_timeout'] + 0.1
    controller.fire_callback(message)

    assert controller.stable_fire() is None
    assert not controller._inspection_fire_event(Blackboard())


def test_qualified_fire_with_blank_frame_uses_legacy_confirmation_instead(system):
    executive, controller = system
    _qualified_fire(executive, controller, header=header(executive.now + 0.01, ''))

    assert controller.stable_fire() is None
    assert not controller._inspection_fire_event(Blackboard())


def test_discarding_fire_confirmation_does_not_keep_old_state_intent_after_negative(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    fire(executive, controller, detected=False)
    assert controller._inspection_fire_event(Blackboard())

    controller.discard_fire_confirmation()

    assert not controller._inspection_fire_event(Blackboard())
    assert controller.fresh_fire() is None and controller.stable_fire() is None


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


def feed_smoke(executive, controller, **changes):
    executive.on_wait = lambda: smoke(executive, controller, **changes)
    smoke(executive, controller, **changes)


def feed_fire(executive, controller, **changes):
    executive.on_wait = lambda: _qualified_fire(executive, controller, **changes)
    return _qualified_fire(executive, controller, **changes)


@pytest.mark.parametrize('operation', ['approach_smoke', 'reacquire', 'search_fire'])
@pytest.mark.parametrize('distance', [None, 0.1, 10000.0])
def test_confirmed_visible_fire_never_requires_smoke_leg_or_fire_range(
        system, operation, distance):
    executive, controller = system
    executive.target_geometry.update(point=None if distance is None else Point(x=distance),
                                     target_valid=distance is not None)
    feed_fire(executive, controller)
    board = Blackboard()
    assert getattr(controller, operation)(board) == 'fire'
    assert controller.center_fire(board) == 'explore'
    assert any(e[0] == 'inspection_report' and e[2] is not None for e in executive.events)
    assert not any(e[0] in {'navigation', 'plan'} for e in executive.events)


def test_default_fire_policy_approaches_smoke_wall_without_fire_range(system):
    executive, _ = system
    executive.wall_x = 30.0
    executive.config.pop('verify_fire_approach_max_steps')
    executive.config.pop('verify_smoke_standoff')
    controller = VerifyController(executive)
    executive._verify = controller
    executive.target_geometry.update(point=None, target_valid=False, range=math.nan)
    feed_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'explore'
    assert executive.odometry.pose.pose.position.x == pytest.approx(9.5)
    assert len([e for e in executive.events if e[0] == 'navigation']) == 4
    assert any('standoff reached' in e[1] for e in executive.events if e[0] == 'status')
    assert any(e[0] == 'inspection_report' and e[2].detected for e in executive.events)


def test_fire_coordinates_do_not_control_center_fire_motion(system):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 1
    executive.target_geometry.update(point=Point(x=1.0), target_valid=True, uncertainty=0.25)
    feed_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'explore'
    request = next(e for e in executive.events if e[0] == 'navigation')
    assert 'fire_keepout_radius' not in request[2]
    assert request[2]['target_valid'] is False
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_smoke_wall_at_ten_metres_stops_even_if_fire_range_is_unknown(system):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 3
    executive.odometry.pose.pose.position.x = 9.5
    executive.target_geometry.update(point=None, target_valid=False)
    feed_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'explore'
    assert not any(e[0] == 'navigation' for e in executive.events)


def test_missing_wall_measurement_holds_without_inventing_distance(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    executive._verify_center_fire_wall = lambda frame: None
    feed_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert any(e[0] == 'hold' and e[1] == 5.0 for e in executive.events)
    assert not any(e[0] in {'navigation', 'inspection_report'} for e in executive.events)


def test_blocked_fire_approach_retries_then_holds_without_reporting_success(system):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller)
    before = controller._pose()

    def blocked(*args, **kwargs):
        kwargs['guard']()
        executive.events.append(('blocked_fire_plan',))
        return False

    executive._verify_navigation_viewpoint = blocked
    board = Blackboard()
    assert controller.center_fire(board) == 'reacquire'
    assert controller.center_fire(board) == 'reacquire'
    with pytest.raises(RuntimeError, match='fire approach blocked'):
        controller.center_fire(board)
    assert controller._pose() == before
    assert not any(event[0] == 'plan' for event in executive.events)
    assert not any(e[0] in {'inspection_report', 'completed_site'} for e in executive.events)
    assert any('fire approach blocked' in event[1]
               for event in executive.events if event[0] == 'status')


def test_fire_approach_deadline_holds_without_reporting_or_restarting_a_chase(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller)
    board = Blackboard()
    board['verify_inspection_started'] = (
        executive.now - controller.config['verify_inspection_timeout'] - 1.0)
    with pytest.raises(RuntimeError, match='inspection time budget exhausted'):
        controller.center_fire(board)
    assert not any(event[0] == 'navigation' for event in executive.events)
    assert not any(e[0] in {'inspection_report', 'completed_site'} for e in executive.events)


@pytest.mark.parametrize('interrupt', ['lost_fire', 'cancel'])
def test_fire_leg_completes_through_loss_but_cancellation_still_stops(system, interrupt):
    from kv_executive.mission_executive import MissionCanceled
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller)
    before = controller._pose()

    def stop():
        if interrupt == 'cancel':
            raise MissionCanceled('cancel during fire approach')
        fire(executive, controller, detected=False)
        executive.on_wait = lambda: fire(executive, controller, detected=False)

    executive.on_motion = stop
    if interrupt == 'cancel':
        with pytest.raises(MissionCanceled):
            controller.center_fire(Blackboard())
    else:
        assert controller.center_fire(Blackboard()) == 'reacquire'
    if interrupt == 'cancel':
        assert controller._pose() == before
    else:
        assert controller._pose().position.x == pytest.approx(before.position.x + 3.0)
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_smoke_approach_uses_short_terrain_navigation_and_arrival_barrier(system):
    executive, controller = system
    feed_smoke(executive, controller)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    target = next(e[1] for e in executive.events if e[0] == 'plan')
    assert target.position.x == pytest.approx(3.0)
    assert target.position.y == pytest.approx(0.0)
    assert board['verify_arrival_after_ns'] == controller._now_ns()
    assert board['verify_smoke_approach_completed']
    assert controller.reacquire(board) == 'approach'


@pytest.mark.parametrize('axis', ['horizontal', 'vertical'])
def test_smoke_image_jitter_does_not_request_separate_axis_centering(system, axis):
    executive, controller = system
    correction = {'yaw_correction_rad' if axis == 'horizontal'
                  else 'vertical_correction_rad': 0.2}
    feed_smoke(executive, controller, **correction)
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert not any(e[0] == 'rotate' for e in executive.events)
    assert not any(e[0] == 'recovery_plan' and e[2] is True for e in executive.events)
    assert executive.odometry.pose.pose.position.z == 6.0
    assert any(e[0] == 'navigation' for e in executive.events)


def test_brief_smoke_occlusion_holds_then_resumes_forward_with_high_image_cue(system):
    executive, controller = system
    initial = smoke(executive, controller, yaw_correction_rad=0.1)
    board = Blackboard()
    board['verify_entered'] = True  # The remembered bearing belongs to this Verify episode.
    controller._remember_smoke_bearing(board, initial, {})
    smoke(executive, controller, detected=False)
    started = executive.now

    def return_after_two_seconds():
        if executive.now - started >= 2.0:
            smoke(executive, controller, yaw_correction_rad=0.12, vertical_correction_rad=0.4)

    executive.on_wait = return_after_two_seconds
    assert controller.approach_smoke(board) == 'reacquire'
    assert 2.0 <= executive.now - started < 5.0
    assert any(event[0] == 'hold' for event in executive.events)
    assert not any(event[0] == 'rotate' for event in executive.events)
    assert executive.odometry.pose.pose.position.x > 2.9
    assert executive.odometry.pose.pose.position.z == 6.0
    assert board['verify_smoke_bearing'] == pytest.approx(0.105)


def test_smoke_returns_reset_the_loss_grace_instead_of_accumulating_misses(system):
    executive, controller = system
    initial = smoke(executive, controller)
    controller._remember_smoke_bearing(Blackboard(), initial, {})
    smoke(executive, controller, detected=False)
    started = executive.now

    def return_briefly():
        if executive.now - started >= 2.0:
            smoke(executive, controller)

    executive.on_wait = return_briefly
    returned = controller._smoke_guidance()
    assert returned is not None
    controller._remember_smoke_bearing(Blackboard(), returned, {})
    last_seen = controller._smoke_seen_at
    smoke(executive, controller, detected=False)
    executive.on_wait = lambda: None
    assert controller._smoke_guidance() is None
    assert executive.now - last_seen == pytest.approx(5.0)
    assert executive.now - started >= 7.0
    assert not any(event[0] in {'rotate', 'plan'} for event in executive.events)


def test_only_continuous_five_second_loss_permits_a_reacquisition_turn(system):
    executive, controller = system
    initial = smoke(executive, controller)
    controller._remember_smoke_bearing(Blackboard(), initial, {})
    seen = controller._smoke_seen_at
    smoke(executive, controller, detected=False)
    rotations = []

    def observe_again_during_scan():
        rotations.append(executive.now)
        smoke(executive, controller)

    executive.on_motion = lambda: None
    original_rotate = executive._verify_rotate

    def rotate(*args, **kwargs):
        rotations.append(executive.now)
        executive.on_motion = observe_again_during_scan
        return original_rotate(*args, **kwargs)

    executive._verify_rotate = rotate
    executive.on_wait = lambda: None
    # Once a scan starts, new post-stop images must become available too.

    def refresh_after_scan():
        if rotations:
            smoke(executive, controller)
    executive.on_wait = refresh_after_scan
    assert controller.reacquire(Blackboard()) == 'approach'
    assert rotations and rotations[0] - seen >= 5.0
    assert not any(event[0] == 'plan' for event in executive.events)


def test_replayed_smoke_cannot_extend_memory_or_authorize_motion(system):
    executive, controller = system
    old = smoke(executive, controller)
    controller._remember_smoke_bearing(Blackboard(), old, {})
    seen = controller._smoke_seen_at
    smoke(executive, controller, detected=False)
    executive.on_wait = lambda: controller.smoke_callback(old)
    assert controller._smoke_guidance() is None
    assert executive.now - seen == pytest.approx(5.0)
    assert not any(event[0] in {'plan', 'rotate'} for event in executive.events)


def test_one_new_fresh_glimpse_between_negative_frames_can_resume_a_forward_leg(system):
    executive, controller = system
    initial = smoke(executive, controller)
    controller._remember_smoke_bearing(Blackboard(), initial, {})
    smoke(executive, controller, detected=False)
    started = executive.now
    glimpses = []

    def flash_then_occlude():
        if not glimpses and executive.now - started >= 1.0:
            glimpses.append(smoke(executive, controller, vertical_correction_rad=0.4))
            smoke(executive, controller, detected=False)

    executive.on_wait = flash_then_occlude
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert glimpses and executive.now - started >= 1.0
    assert executive.odometry.pose.pose.position.x > 2.9
    assert executive.odometry.pose.pose.position.z == 6.0
    assert controller.fresh_smoke() is None
    assert not any(event[0] == 'rotate' for event in executive.events)


def test_slow_geometry_query_cannot_turn_an_expired_smoke_image_into_a_new_leg(system):
    executive, controller = system
    feed_smoke(executive, controller)
    original = executive._verify_target_geometry

    def delayed(message, source='smoke'):
        executive.now += 2.1
        return original(message, source)

    executive._verify_target_geometry = delayed
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert not any(event[0] in {'plan', 'navigation', 'rotate'} for event in executive.events)


def test_smoke_bearing_memory_smooths_wraparound_once_per_source_image(system):
    executive, controller = system
    board = Blackboard()
    initial = smoke(executive, controller)
    controller._remember_smoke_bearing(board, initial, {'navigation_bearing': math.radians(179)})
    latest = smoke(executive, controller)
    bearing, _ = controller._remember_smoke_bearing(
        board, latest, {'navigation_bearing': math.radians(-179)})
    assert math.degrees(bearing) == pytest.approx(179.5)
    repeated, _ = controller._remember_smoke_bearing(
        board, latest, {'navigation_bearing': math.radians(-179)})
    assert repeated == bearing
    controller.reset()
    assert controller._smoke_seen_at is None and controller._smoke_bearing is None


def test_new_verify_episode_does_not_steer_toward_the_previous_site(system):
    executive, controller = system
    initial = smoke(executive, controller)
    controller._remember_smoke_bearing(Blackboard(), initial, {'navigation_bearing': 0.0})
    # Exploration has turned toward a different site in the same camera frame.
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(math.pi)
    smoke(executive, controller)
    board = Blackboard()

    assert controller.approach_smoke(board) == 'reacquire'

    bearing = next(event[1] for event in executive.events if event[0] == 'navigation')
    assert bearing == pytest.approx(math.pi)
    assert executive.odometry.pose.pose.position.x == pytest.approx(-3.0)


def test_short_forward_steps_are_not_mislabeled_as_revisited_positions(system):
    executive, controller = system
    board = Blackboard()
    for _ in range(3):
        before = controller._pose()
        executive.odometry.pose.pose.position.x += 0.4
        assert controller._navigation_progress(board, before, 0.0, True)
        assert board['verify_navigation_stalls'] == 0


@pytest.mark.parametrize('vertical', [-0.25, 0.25])
def test_center_fire_corrects_yaw_and_bounded_height_at_standoff(system, vertical):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 5
    feed_fire(executive, controller, yaw_correction_rad=0.3,
              vertical_correction_rad=vertical)
    assert controller.center_fire(Blackboard()) == 'explore'
    corrections = [e[0] for e in executive.events if e[0] in {'rotate', 'recovery_plan'}]
    assert corrections == ['rotate', 'recovery_plan', 'recovery_plan', 'recovery_plan']
    assert executive.odometry.pose.pose.position.z == pytest.approx(
        6.0 + math.copysign(1.5, vertical))
    assert any(e[0] == 'inspection_report' and e[2] is not None for e in executive.events)


@pytest.mark.parametrize('failure', ['rejected', 'start', 'escape'])
def test_blocked_vertical_centering_still_reports_current_visible_fire(system, failure):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 5
    feed_fire(executive, controller, yaw_correction_rad=0.3, vertical_correction_rad=-0.3)

    def blocked(*args, **kwargs):
        executive.events.append(('blocked_vertical',))
        if failure == 'rejected':
            return False
        error = (verification.VerificationStartUnavailable if failure == 'start'
                 else verification.VerificationRecoveryUnavailable)
        raise error('test blocked clearance')
    executive._verify_recovery_plan_to = blocked
    outcome = controller.center_fire(Blackboard())
    assert outcome == ('explore' if failure == 'rejected' else 'fire')
    assert any(e[0] == 'rotate' for e in executive.events)
    assert any(e[0] == 'blocked_vertical' for e in executive.events)
    assert any(e[0] == 'inspection_report' for e in executive.events) == (failure == 'rejected')


def test_fire_centering_cannot_report_a_cached_pre_motion_frame(system):
    executive, controller = system
    _qualified_fire(executive, controller, yaw_correction_rad=0.3)
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_fire_lost_during_vertical_correction_requires_new_confirmation(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 5
    feed_fire(executive, controller, vertical_correction_rad=-0.3)

    def lose():
        fire(executive, controller, detected=False)
        executive.on_wait = lambda: fire(executive, controller, detected=False)
    executive.on_motion = lose
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_fire_recovery_does_not_publish_a_negative_when_smoke_remains(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    board = Blackboard()
    controller._route_fire(board)
    fire(executive, controller, detected=False)
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    assert controller.center_fire(board) == 'reacquire'
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_fire_during_smoke_leg_allows_arrival_then_requires_post_stop_confirmation(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.on_motion = lambda: _qualified_fire(executive, controller)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_smoke_approach_completed']
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert controller.had_fire_confirmation()
    assert controller._reacquire_fire(board) is None
    assert not any(event[0] == 'inspection_report' for event in executive.events)
    feed_fire(executive, controller)
    assert controller.reacquire(board) == 'fire'


def test_broad_smoke_during_leg_allows_arrival_without_starting_inspection(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.on_motion = lambda: smoke(executive, controller, coverage=0.3,
                                        regions=[1, 2, 3, 4])
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_smoke_approach_completed']
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert verification._bb_get(board, 'verify_inspection_started', None) is None
    assert not any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('coverage', [
    {'regions': [1, 2, 3, 4]}, {'regions': [1], 'quadrant_coverage': [0.2] * 4}])
def test_broad_smoke_can_start_and_repeat_smoke_legs_without_inspection_timer(system, coverage):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.8, **coverage)
    board = Blackboard()
    for _ in range(2):
        assert controller.approach_smoke(board) == 'reacquire'
        assert controller.reacquire(board) == 'approach'
    assert executive.odometry.pose.pose.position.x == pytest.approx(6.0)
    assert verification._bb_get(board, 'verify_inspection_started', None) is None
    assert not any(e[0] in {'inspection_report', 'completed_site'} for e in executive.events)


def test_broad_smoke_in_search_does_not_start_timed_inspection(system):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.8, regions=[1, 2, 3, 4])
    board = Blackboard()
    assert controller.search_fire(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert verification._bb_get(board, 'verify_inspection_started', None) is None
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)


def test_measured_boundary_starts_shared_budget_even_if_later_measurement_disappears(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.target_geometry.update(plume_candidate=True, obstacle_range=8.0)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    started = board['verify_inspection_started']
    executive.target_geometry['plume_candidate'] = False
    assert controller.reacquire(board) == 'search'
    assert controller.search_fire(board) == 'reacquire'
    assert board['verify_inspection_started'] == started


@pytest.mark.parametrize('failure', ['cancel', 'return', 'unsafe_corridor'])
def test_committed_smoke_leg_still_stops_for_mission_or_navigation_failures(system, failure):
    from kv_executive.mission_executive import MissionCanceled, MissionStateRequested
    executive, controller = system
    feed_smoke(executive, controller)
    board = Blackboard()
    errors = {
        'cancel': MissionCanceled('operator canceled'),
        'return': MissionStateRequested('Return'),
        'unsafe_corridor': verification.VerificationObservationLost('corridor unsafe'),
    }

    def interrupt():
        _qualified_fire(executive, controller)
        smoke(executive, controller, coverage=0.8, regions=[1, 2, 3, 4])
        raise errors[failure]

    executive.on_motion = interrupt
    if failure == 'unsafe_corridor':
        assert controller.approach_smoke(board) == 'reacquire'
    else:
        with pytest.raises(type(errors[failure])):
            controller.approach_smoke(board)
    assert executive.odometry.pose.pose.position.x == 0.0
    assert not verification._bb_get(board, 'verify_smoke_approach_completed', False)
    assert not any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('steps', [0, 1])
def test_fire_step_limit_cannot_report_distant_fire(system, steps):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = steps
    feed_fire(executive, controller)
    with pytest.raises(RuntimeError, match='fire approach step limit reached'):
        controller.center_fire(Blackboard())
    assert executive.odometry.pose.pose.position.x == pytest.approx(steps * 3.0)
    assert not any(e[0] in {'inspection_report', 'completed_site'} for e in executive.events)


@pytest.mark.parametrize('delay', ['query', 'hold'])
def test_wall_wait_cannot_start_another_leg_after_inspection_deadline(system, delay):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config.update(verify_fire_approach_max_steps=3, verify_inspection_timeout=2.0)
    feed_fire(executive, controller)
    query = executive._verify_center_fire_wall
    calls = []

    def delayed_wall(frame):
        calls.append(frame)
        if len(calls) == 1:
            if delay == 'hold':
                return None
            executive.now += 3.0
            _qualified_fire(executive, controller)
        return query(frame)

    executive._verify_center_fire_wall = delayed_wall
    with pytest.raises(RuntimeError, match='inspection time budget exhausted'):
        controller.center_fire(Blackboard())
    assert not any(e[0] in {'navigation', 'inspection_report'} for e in executive.events)


def test_search_uses_only_smoke_geometry_for_translation(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.target_geometry.update(point=Point(x=10000.0), range=10000.0,
                                     target_valid=True)
    assert controller.search_fire(Blackboard()) == 'reacquire'
    navigation = next(e for e in executive.events if e[0] == 'navigation')
    assert navigation[2]['source'] == 'smoke'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_observed_smoke_boundary_prevents_another_forward_leg(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.target_geometry.update(plume_candidate=True, obstacle_range=8.0)
    board = Blackboard()
    assert controller.search_fire(board) == 'inspect'
    assert not any(e[0] == 'navigation' for e in executive.events)


def test_broad_smoke_with_unknown_boundary_cannot_publish_negative(system):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    board = Blackboard()
    assert controller.center_fire(board) == 'search'
    assert verification._bb_get(board, 'verify_inspection_started', None) is None
    assert not any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('initial_bearing', [None, 0.0])
def test_three_completed_boundary_windows_can_report_no_fire(system, initial_bearing):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    executive.target_geometry.update(plume_candidate=True, obstacle_range=8.0)
    board = Blackboard()
    board['verify_smoke_bearing'] = initial_bearing
    assert controller.center_fire(board) == 'explore'
    reports = [e for e in executive.events if e[0] == 'inspection_report']
    assert len(reports) == 1 and reports[0][2] is None and reports[0][3] == 3
    assert len([e for e in executive.events if e[0] == 'recovery_plan']) == 2


def test_blocked_boundary_viewpoint_never_counts_as_completed_inspection(system):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    executive.target_geometry.update(plume_candidate=True, obstacle_range=8.0)
    executive._verify_recovery_plan_to = lambda *a, **k: False
    board = Blackboard()
    assert controller.center_fire(board) == 'reacquire'
    assert board['verify_inspection_attempts'] == 1
    assert not any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('error_type', [
    verification.VerificationStartUnavailable, verification.VerificationRecoveryUnavailable])
def test_map_refresh_then_explicit_hold_for_blocked_source(system, error_type):
    executive, controller = system
    feed_smoke(executive, controller)

    def unavailable(*a, **k):
        raise error_type('no safe clearance route')
    executive._verify_navigation_viewpoint = unavailable
    board = Blackboard()
    assert controller.search_fire(board) == 'search'
    executive.map_available = False
    with pytest.raises(RuntimeError, match='Verify blocked'):
        controller.search_fire(board)
    assert not verification._bb_get(board, 'verify_exploration_required', False)


@pytest.mark.parametrize('error_type', [
    verification.VerificationMapUnavailable, verification.VerificationRecoveryUnavailable])
def test_fire_found_during_map_refresh_does_not_get_abandoned(system, error_type):
    executive, controller = system
    feed_smoke(executive, controller)

    def unavailable(*a, **k):
        raise error_type('map or clearance unavailable')
    executive._verify_navigation_viewpoint = unavailable
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    board = Blackboard()
    assert controller.search_fire(board) in {'search', 'fire'}
    feed_fire(executive, controller)
    assert controller.search_fire(board) == 'fire'
    assert controller.center_fire(board) == 'explore'
    assert any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('confirmed_fire', [False, True])
def test_map_recovery_waits_for_a_new_source_stamp_without_recentering(system, confirmed_fire):
    executive, controller = system
    if confirmed_fire:
        _qualified_fire(executive, controller)
        assert controller.stable_fire() is not None
    baseline = executive._verify_last_geometry_stamp_ns
    stamps = iter([baseline, 0, baseline, baseline + 500_000_000])

    def poll(timeout=None):
        executive.events.append(('map_poll',))
        assert 0.0 < timeout <= 0.5
        return next(stamps)

    executive._verify_fresh_map_stamp = poll
    board = Blackboard()
    error = verification.VerificationMapUnavailable('source age exceeds limit', baseline)
    expected = 'fire' if confirmed_fire else 'approach'
    assert controller._navigation_start_unavailable(board, error, resume='approach') == expected
    assert len([event for event in executive.events if event[0] == 'map_poll']) == 4
    assert board['verify_navigation_refresh_stamp_ns'] == baseline + 500_000_000
    assert not any(event[0] in {'rotate', 'plan', 'navigation', 'recovery_plan'}
                   for event in executive.events)


@pytest.mark.parametrize('available', [False, True])
@pytest.mark.parametrize('fire_timing', ['none', 'before', 'during'])
def test_missing_or_replayed_maps_exhaust_one_bounded_recovery_budget(
        system, available, fire_timing):
    executive, controller = system
    if fire_timing == 'before':
        _qualified_fire(executive, controller)
        assert controller.stable_fire() is not None
    elif fire_timing == 'during':
        def detect_fire_once():
            _qualified_fire(executive, controller)
            executive.on_wait = lambda: None
        executive.on_wait = detect_fire_once
    controller.config['verify_navigation_map_timeout'] = 1.0
    executive.map_available = available
    executive.map_stamp_ns = executive._verify_last_geometry_stamp_ns
    started = executive.now
    with pytest.raises(RuntimeError, match='recovery deadline exceeded'):
        controller._navigation_start_unavailable(
            Blackboard(), verification.VerificationMapUnavailable('stale source'))
    assert executive.now == pytest.approx(started + 1.0)
    if fire_timing != 'none':
        assert controller.stable_fire() is not None
    assert not any(event[0] in {'rotate', 'plan', 'navigation'} for event in executive.events)


@pytest.mark.parametrize('error_type', [
    verification.VerificationStartUnavailable, verification.VerificationRecoveryUnavailable])
def test_new_maps_do_not_restart_clearance_recovery_at_same_position(system, error_type):
    executive, controller = system
    controller.config['verify_navigation_map_timeout'] = 1.0
    board = Blackboard()
    error = error_type('current clearance unavailable')
    assert controller._navigation_start_unavailable(board, error) == 'reacquire'
    deadline = board['verify_navigation_refresh_deadline']
    executive.now = deadline - 0.1
    with pytest.raises(RuntimeError, match='recovery deadline exceeded'):
        controller._navigation_start_unavailable(board, error)
    assert board['verify_navigation_refresh_deadline'] == deadline


def test_separate_map_outages_do_not_spend_budget_during_healthy_planning(system):
    executive, controller = system
    controller.config['verify_navigation_map_timeout'] = 1.0
    board = Blackboard()
    executive.now += 0.5
    for _ in range(3):
        # The prior map failed during motion. A new map has already arrived
        # while the aircraft switched to LOITER, before recovery starts.
        failed_stamp = int((executive.now - 0.5) * 1e9)
        executive.map_stamp_ns = int(executive.now * 1e9)
        started = executive.now
        assert controller._navigation_start_unavailable(
            board, verification.VerificationMapUnavailable('stale source', failed_stamp),
            resume='approach') == 'approach'
        assert executive.now - started < 1.0
        # Successful planning and mode handoff can outlast the outage timeout
        # without any completed translation, as in the reported flight.
        executive.now += 2.0
    assert len([event for event in executive.events if event[0] == 'map_poll']) == 3
    assert verification._bb_get(board, 'verify_navigation_refresh_deadline', None) is None


def test_map_outage_checks_recovered_data_despite_expired_clearance_deadline(system):
    executive, controller = system
    board = Blackboard()
    executive.now += 0.5
    board['verify_navigation_refresh_deadline'] = executive.now - 1.0
    failed_stamp = int((executive.now - 0.5) * 1e9)
    executive.map_stamp_ns = int(executive.now * 1e9)

    assert controller._navigation_start_unavailable(
        board, verification.VerificationMapUnavailable('old source', failed_stamp),
        resume='approach') == 'approach'

    assert any(event[0] == 'map_poll' for event in executive.events)
    # Map recovery must not forgive a separate physical-clearance failure.
    assert board['verify_navigation_refresh_deadline'] < executive.now


def test_missing_map_after_a_recovered_outage_still_has_a_bounded_wait(system):
    executive, controller = system
    controller.config['verify_navigation_map_timeout'] = 1.0
    board = Blackboard()
    error = verification.VerificationMapUnavailable('stale source')
    assert controller._navigation_start_unavailable(board, error) == 'reacquire'
    executive.now += 2.0
    executive.map_available = False
    started = executive.now

    with pytest.raises(RuntimeError, match='recovery deadline exceeded'):
        controller._navigation_start_unavailable(board, error)

    assert executive.now == pytest.approx(started + 1.0)


def test_map_recovery_accepts_new_epoch_after_clock_reset_during_wait(system):
    executive, controller = system
    ros_time = [100_000_000_000]
    controller._now_ns = lambda: ros_time[0]
    polls = []

    def poll(timeout=None):
        polls.append(True)
        if len(polls) == 1:
            ros_time[0] = 10_000_000_000
            return 0
        return ros_time[0]

    executive._verify_fresh_map_stamp = poll
    board = Blackboard()
    assert controller._navigation_start_unavailable(
        board, verification.VerificationMapUnavailable('clock reset')) == 'reacquire'
    assert len(polls) == 2
    assert board['verify_navigation_refresh_stamp_ns'] == 10_000_000_000


@pytest.mark.parametrize('failure,expected', [
    ('cancel', 'canceled'), ('return', 'return_requested')])
def test_outer_state_recovery_preserves_mission_interrupts(system, failure, expected):
    from kv_executive.mission_executive import MissionCanceled, MissionStateRequested
    executive, controller = system
    executive.map_available = False

    def unavailable(board):
        raise verification.VerificationMapUnavailable('stale source')

    def interrupt():
        raise (MissionCanceled('cancel during map wait') if failure == 'cancel'
               else MissionStateRequested('Return'))

    controller.approach_smoke = unavailable
    executive.on_wait = interrupt
    assert VerifyStateMachine(executive)(Blackboard()) == expected


def test_lateral_ping_pong_is_not_forward_smoke_progress(system):
    executive, controller = system
    board = Blackboard()
    start = controller._pose()
    executive.odometry.pose.pose.position.y = 1.0
    assert controller._navigation_progress(board, start, 0.0, True)
    assert board['verify_navigation_stalls'] == 1
    before = controller._pose()
    executive.odometry.pose.pose.position.y = 0.0
    assert not controller._navigation_progress(board, before, 0.0, True)
    assert not controller._navigation_progress(board, controller._pose(), 0.0, True)


@pytest.mark.parametrize('operation', [
    'approach_smoke', 'reacquire', 'search_fire', 'center_fire'])
@pytest.mark.parametrize('failure', ['cancel', 'return', 'abort'])
def test_every_state_preserves_mission_interrupts(system, operation, failure):
    from kv_executive.mission_executive import MissionCanceled, MissionError, MissionStateRequested
    executive, controller = system
    feed_smoke(executive, controller)
    errors = {'cancel': MissionCanceled('cancel'), 'return': MissionStateRequested('Return'),
              'abort': MissionError('abort')}

    def stop():
        raise errors[failure]
    executive._check_running = stop
    with pytest.raises(type(errors[failure])):
        getattr(controller, operation)(Blackboard())


@pytest.mark.parametrize('operation', ['approach_smoke', 'search_fire', 'center_fire'])
def test_waiting_for_new_image_preserves_cancellation(system, operation):
    from kv_executive.mission_executive import MissionCanceled
    executive, controller = system
    if operation == 'center_fire':
        _qualified_fire(executive, controller, yaw_correction_rad=0.3)
    else:
        smoke(executive, controller, yaw_correction_rad=0.3)
        if operation == 'approach_smoke':
            controller._remember_smoke_bearing(
                Blackboard(), controller.fresh_smoke(), {})
            smoke(executive, controller, detected=False)

    def stop():
        raise MissionCanceled('cancel while waiting')
    executive.on_wait = stop
    with pytest.raises(MissionCanceled):
        getattr(controller, operation)(Blackboard())


@pytest.mark.parametrize('failure, expected', [
    ('cancel', 'canceled'), ('return', 'return_requested'), ('error', 'aborted')])
def test_four_state_machine_propagates_cancel_return_and_failure(system, failure, expected):
    from kv_executive.mission_executive import MissionCanceled, MissionError, MissionStateRequested
    executive, controller = system
    errors = {'cancel': MissionCanceled('cancel'), 'return': MissionStateRequested('Return'),
              'error': MissionError('error')}

    def fail(blackboard):
        raise errors[failure]
    controller.approach_smoke = fail
    assert VerifyStateMachine(executive)(Blackboard()) == expected


def test_verify_machine_declares_four_substates(system):
    executive, _ = system
    assert set(VerifyStateMachine(executive).get_states()) == {
        'APPROACH_SMOKE', 'REACQUIRE', 'CENTER_FIRE', 'SEARCH_FIRE'}


def test_approach_cycle_limit_stops_smoke_chasing(system):
    executive, controller = system
    feed_smoke(executive, controller)
    board = Blackboard()
    board['verify_approaches'] = controller.config['verify_max_approaches']
    with pytest.raises(RuntimeError, match='approach limit'):
        controller.approach_smoke(board)


def test_map_retries_do_not_spend_completed_smoke_leg_budget(system):
    executive, controller = system
    controller.config['verify_max_approaches'] = 1
    board = Blackboard()
    navigate = executive._verify_navigation_viewpoint

    def unavailable(*args, **kwargs):
        raise verification.VerificationMapUnavailable('temporary map outage')

    executive._verify_navigation_viewpoint = unavailable
    for _ in range(2):
        feed_smoke(executive, controller)
        assert controller.approach_smoke(board) == 'approach'
        assert verification._bb_get(board, 'verify_approaches', 0) == 0

    executive._verify_navigation_viewpoint = navigate
    feed_smoke(executive, controller)
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_approaches'] == 1
    before = deepcopy(executive.odometry.pose.pose)
    feed_smoke(executive, controller)
    with pytest.raises(RuntimeError, match='approach limit'):
        controller.approach_smoke(board)
    assert executive.odometry.pose.pose == before


def test_smoke_loss_waits_do_not_spend_completed_leg_budget(system):
    executive, controller = system
    controller.config['verify_max_approaches'] = 1
    board = Blackboard()
    smoke(executive, controller, detected=False)

    for _ in range(2):
        assert controller.approach_smoke(board) == 'reacquire'
        assert verification._bb_get(board, 'verify_approaches', 0) == 0
    assert not any(event[0] == 'navigation' for event in executive.events)


def test_noop_successful_planner_cannot_claim_a_completed_smoke_leg(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive._verify_navigation_viewpoint = lambda *a, **k: True
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert not verification._bb_get(board, 'verify_smoke_approach_completed', False)
    assert verification._bb_get(board, 'verify_arrival_after_ns', 0) == 0


def test_failed_horizontal_attempt_retains_visible_fire_without_vertical_dancing(system):
    executive, controller = system
    feed_fire(executive, controller, yaw_correction_rad=0.3, vertical_correction_rad=0.3)

    def blocked(*a, **k):
        executive.events.append(('blocked_horizontal',))
        raise verification.VerificationStartUnavailable('yaw correction interrupted')

    executive._verify_rotate = blocked
    assert controller.center_fire(Blackboard()) == 'explore'
    assert any(e[0] == 'blocked_horizontal' for e in executive.events)
    assert not any(e[0] == 'recovery_plan' for e in executive.events)
    assert any(e[0] == 'inspection_report' for e in executive.events)


def test_lost_confirmed_fire_with_smoke_eventually_holds_without_false_negative(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    board = Blackboard()
    controller._route_fire(board)
    fire(executive, controller, detected=False)
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    for _ in range(controller.config['verify_search_recovery_max'] - 1):
        assert controller.center_fire(board) == 'reacquire'
    assert controller.center_fire(board) == 'reacquire'
    assert verification._bb_get(board, 'verify_search_failures', 0) == 0
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_short_smoke_leg_can_finish_through_brief_detector_gap(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.on_motion = lambda: smoke(executive, controller, detected=False)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_smoke_approach_completed']
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_actual_new_viewpoint_resets_blocked_recovery_counter(system):
    executive, controller = system
    feed_smoke(executive, controller)
    board = Blackboard()
    board['verify_search_failures'] = 2
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_search_failures'] == 0


def test_center_fire_climb_longer_than_five_seconds_finishes_before_observation_wait(system):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 1
    feed_fire(executive, controller)

    def climb(bearing, **kwargs):
        kwargs['guard']()
        fire(executive, controller, detected=False)
        executive.now += 8.0
        kwargs['guard']()
        executive.odometry.pose.pose.position.x += 3.0
        executive.odometry.pose.pose.position.z += 3.0
        return True
    executive._verify_navigation_viewpoint = climb
    executive.on_wait = lambda: fire(executive, controller, detected=False)
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert executive.odometry.pose.pose.position.z == pytest.approx(9.0)
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_center_fire_track_id_change_does_not_cancel_committed_leg(system):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 1
    feed_fire(executive, controller)
    executive.on_motion = lambda: _qualified_fire(executive, controller, track_id=99)
    executive.on_wait = lambda: _qualified_fire(executive, controller, track_id=99)
    assert controller.center_fire(Blackboard()) == 'explore'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_center_fire_retreats_if_smoke_wall_is_already_too_close(system):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 1
    executive.odometry.pose.pose.position.x = 12.0
    feed_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'explore'
    assert executive.odometry.pose.pose.position.x < 12.0
    assert not any(e[0] == 'navigation' for e in executive.events)


def test_vertical_preference_uses_hysteresis_fresh_images_and_track_identity(system):
    executive, controller = system
    board = Blackboard()

    def preference(degrees, track_id=1):
        observation = _qualified_fire(
            executive, controller, vertical_correction_rad=math.radians(degrees),
            track_id=track_id)
        return controller._fire_vertical_preference(board, observation)

    assert preference(-6.0) == 0.0  # Inside the activation band.
    assert preference(-8.0) == -0.5
    assert preference(-6.0) == -0.5  # Keep the same direction until the release band.
    assert controller._fire_vertical_preference(board, controller.stable_fire()) == 0.0
    assert preference(-4.0) == 0.0
    assert preference(6.0) == 0.0
    assert preference(8.0) == 0.5
    board['verify_vertical_attempts'] = 3
    assert preference(6.0, track_id=2) == 0.0
    assert board['verify_vertical_attempts'] == 0
    observation = controller.stable_fire()
    executive.now += controller.config['verify_observation_timeout'] + 0.1
    assert controller._fire_vertical_preference(board, observation) == 0.0


@pytest.mark.parametrize('vertical', [-0.2, 0.2])
def test_fire_approach_combines_forward_motion_with_bounded_altitude(system, vertical):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 2
    _qualified_fire(executive, controller, vertical_correction_rad=vertical)
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'explore'
    goals = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(goals) == 1
    assert goals[0].position.x == pytest.approx(3.0)
    assert goals[0].position.z == pytest.approx(6.0 + math.copysign(0.5, vertical))


def test_fire_near_vertical_edge_corrects_height_before_advancing(system):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    board = Blackboard()
    assert controller.center_fire(board) == 'explore'
    goals = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(goals) == 2
    assert goals[0].position == Point(x=0.0, z=5.5)
    assert goals[1].position == Point(x=3.0, z=5.5)
    assert board['verify_fire_approach_steps'] == 2
    assert verification._bb_get(board, 'verify_navigation_stalls', 0) == 0


def test_lower_fire_during_smoke_leg_changes_only_a_subsequent_goal(system):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_smoke(executive, controller)
    executive.on_motion = lambda: _qualified_fire(
        executive, controller, vertical_correction_rad=-0.2)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=6.0)
    feed_fire(executive, controller, vertical_correction_rad=-0.2)
    assert controller.reacquire(board) == 'fire'
    executive.on_motion = lambda: None
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    assert controller.center_fire(board) == 'explore'
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=5.5)


def test_vertical_correction_remeasures_wall_before_success(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller, vertical_correction_rad=-0.2)
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    executive.on_motion = lambda: setattr(executive, 'wall_x', 13.5)
    assert controller.center_fire(Blackboard()) == 'explore'
    goals = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(goals) == 2
    assert goals[0].position == Point(z=5.5)
    assert goals[1].position == Point(x=3.0, z=5.5)
    events = [event[0] for event in executive.events]
    assert events.index('inspection_report') > events.index('navigation')


def test_blocked_edge_correction_does_not_advance_toward_distant_fire(system):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller, vertical_correction_rad=-0.4)
    executive._verify_recovery_plan_to = lambda *a, **k: False
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(e[0] in {'navigation', 'inspection_report'} for e in executive.events)


def test_unavailable_wall_probes_allow_repeated_far_approach_without_inspection_clock(system):
    executive, controller = system
    executive.target_geometry.update(point=None, target_valid=False)
    board = Blackboard()
    for _ in range(4):
        smoke(executive, controller)
        assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == 12.0
    assert len([event for event in executive.events if event[0] == 'wall_probe']) == 4
    assert verification._bb_get(board, 'verify_inspection_started', None) is None


@pytest.mark.parametrize('angle', [-0.4, 0.4])
@pytest.mark.parametrize('operation', ['approach_smoke', 'reacquire', 'search_fire'])
def test_fire_seen_during_leg_recovers_height_once_in_either_direction(system, angle, operation):
    executive, controller = system
    board = Blackboard()
    smoke(executive, controller)
    executive.on_motion = lambda: _qualified_fire(
        executive, controller, vertical_correction_rad=angle)
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=6.0)
    executive.on_motion = lambda: None
    fire(executive, controller, detected=False)
    assert getattr(controller, operation)(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(
        x=3.0, z=6.0 + math.copysign(0.5, angle))
    assert not controller._recover_fire_height(board)
    assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 2
    assert not any(event[0] == 'inspection_report' for event in executive.events)


@pytest.mark.parametrize('invalid', ['wall_age', 'image_age', 'distance', 'yaw', 'track'])
def test_expired_or_displaced_fire_hint_never_commands_height(system, invalid):
    executive, controller = system
    board = Blackboard()
    _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    controller._capture_height_hint(board)
    fire(executive, controller, detected=False)
    if invalid == 'wall_age':
        image, pose, received = board['verify_height_hint']
        board['verify_height_hint'] = (image, pose, received - 16.0)
    elif invalid == 'image_age':
        executive.now += 6.0
    elif invalid == 'distance':
        executive.odometry.pose.pose.position.x = 4.0
    elif invalid == 'yaw':
        executive.odometry.pose.pose.orientation = quaternion_from_yaw(math.radians(20.0))
    else:
        fire(executive, controller, track_id=2)
    assert not controller._recover_fire_height(board)
    assert not any(event[0] in {'plan', 'navigation', 'recovery_plan'}
                   for event in executive.events)


@pytest.mark.parametrize('angle', [-0.4, 0.4])
def test_visible_edge_height_correction_precedes_wall_query(system, angle):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller, vertical_correction_rad=angle)

    def wall_query(frame):
        pytest.fail('Edge height correction must precede wall ranging')

    executive._verify_center_fire_wall = wall_query
    # Lose the target during the correction; the goal must still complete.
    executive.on_motion = lambda: fire(executive, controller, detected=False)
    assert controller.center_fire(Blackboard()) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(z=6.0 + math.copysign(0.5, angle))


def test_measured_arrival_survives_missing_probes_without_forward_motion_or_report(system):
    executive, controller = system
    board = Blackboard()
    feed_smoke(executive, controller)
    executive._verify_opportunistic_wall = lambda frame, bearing: {
        **executive._verify_center_fire_wall(frame), 'bearing': bearing}
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_smoke_boundary_reached']
    started = board['verify_inspection_started']
    executive._verify_opportunistic_wall = lambda *args: None
    assert controller.approach_smoke(board) == 'reacquire'
    assert controller.reacquire(board) == 'search'
    assert controller.search_fire(board) == 'inspect'
    assert controller.center_fire(board) == 'reacquire'
    assert board['verify_inspection_started'] == started
    assert not any(event[0] in {'navigation', 'plan', 'inspection_report'}
                   for event in executive.events)


def test_one_quadrant_smoke_at_measured_wall_completes_three_attempt_negative_inspection(system):
    executive, controller = system
    board = Blackboard()
    feed_smoke(executive, controller, regions=[1], coverage=0.02)
    executive._verify_opportunistic_wall = lambda frame, bearing: {
        **executive._verify_center_fire_wall(frame), 'bearing': bearing}
    assert controller.approach_smoke(board) == 'reacquire'
    assert controller.reacquire(board) == 'search'
    assert controller.search_fire(board) == 'inspect'
    assert controller.center_fire(board) == 'explore'
    report = next(event for event in executive.events if event[0] == 'inspection_report')
    assert report[2] is None and report[3] == 3


def test_reacquisition_cannot_restart_expired_inspection_clock(system):
    executive, controller = system
    board = Blackboard()
    board['verify_inspection_started'] = executive.now - controller.config[
        'verify_inspection_timeout'] - 1.0
    smoke(executive, controller)
    with pytest.raises(RuntimeError, match='inspection time budget exhausted'):
        controller.reacquire(board)
    assert not any(event[0] in {'plan', 'navigation'} for event in executive.events)


def test_legacy_smoke_boundary_is_retained_after_its_range_disappears(system):
    executive, controller = system
    board = Blackboard()
    feed_smoke(executive, controller)
    wall = executive._verify_center_fire_wall('camera')
    executive._verify_smoke_navigation_wall = lambda geometry: wall
    assert controller.approach_smoke(board) == 'reacquire'
    executive._verify_smoke_navigation_wall = lambda geometry: None
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_smoke_boundary_reached']
    assert not any(event[0] in {'navigation', 'plan'} for event in executive.events)


def test_height_image_already_used_for_visible_correction_is_not_reused_as_memory(system):
    executive, controller = system
    board = Blackboard()
    image = _qualified_fire(executive, controller, vertical_correction_rad=0.4)
    controller._capture_height_hint(board)
    assert controller._fire_vertical_preference(board, image) == 0.5
    fire(executive, controller, detected=False)
    assert not controller._recover_fire_height(board)
    assert not any(event[0] == 'recovery_plan' for event in executive.events)


def test_arrival_at_smoke_leg_limit_still_enters_inspection(system):
    executive, controller = system
    board = Blackboard()
    board['verify_approaches'] = controller.config['verify_max_approaches']
    feed_smoke(executive, controller)
    executive._verify_opportunistic_wall = lambda frame, bearing: {
        **executive._verify_center_fire_wall(frame), 'bearing': bearing}
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_smoke_boundary_reached']
    assert not any(event[0] == 'navigation' for event in executive.events)


@pytest.mark.parametrize('interrupt', ['lost_fire', 'cancel', 'return'])
def test_vertical_leg_finishes_through_fire_loss_but_obeys_vehicle_interrupts(system, interrupt):
    from kv_executive.mission_executive import MissionCanceled, MissionStateRequested
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller, vertical_correction_rad=-0.2)

    def stop():
        if interrupt == 'cancel':
            raise MissionCanceled('cancel during vertical correction')
        if interrupt == 'return':
            raise MissionStateRequested('Return')
        fire(executive, controller, detected=False)
        executive.on_wait = lambda: fire(executive, controller, detected=False)

    executive.on_motion = stop
    if interrupt == 'lost_fire':
        assert controller.center_fire(Blackboard()) == 'reacquire'
        assert executive.odometry.pose.pose.position == Point(z=5.5)
    else:
        with pytest.raises(MissionCanceled if interrupt == 'cancel' else MissionStateRequested):
            controller.center_fire(Blackboard())
        assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(e[0] == 'inspection_report' for e in executive.events)
