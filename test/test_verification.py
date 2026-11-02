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
        self.config = {'verify_fire_approach_max_steps': 0, 'verify_smoke_standoff': 10.0,
                       'verify_smoke_inspection_max_distance': 10.6}
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

    def _verify_prepare_wall_observation(self, guard=None, *, align_fire=True):
        if guard:
            guard()
        return True

    def _verify_wall_observation_ready(self):
        return True

    def _verify_image_bearing(self, message):
        return verification.yaw_from_quaternion(
            self.odometry.pose.pose.orientation) + message.yaw_correction_rad

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
                'camera_frame': camera_frame, 'map_stamp_ns': int(self.now * 1e9)}

    def _verify_opportunistic_wall(self, camera_frame, bearing):
        self.events.append(('wall_probe', camera_frame, bearing))
        if self._verify.stable_fire() is not None:
            wall = self._verify_center_fire_wall(camera_frame)
            return None if wall is None else {**wall, 'bearing': bearing}
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
            {**wall, 'nearest': distance}, self.config['verify_smoke_standoff'],
            max_distance=self.config['verify_smoke_inspection_max_distance'])

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
                                     vertical_offset=0.0, allow_oblique=False):
        self.events.append(('navigation', bearing, deepcopy(geometry)))
        target = deepcopy(self.odometry.pose.pose)
        forward = 3.0 if maximum_forward is None else min(3.0, maximum_forward)
        for wall in (geometry or {}).get('smoke_navigation_walls', []):
            advance = sum((getattr(target.position, a) - getattr(wall['origin'], a))
                          * wall['axis'][i] for i, a in enumerate('xyz'))
            remaining = (wall['nearest'] - wall['uncertainty']
                         - self.config['verify_smoke_standoff'] - 0.3 - advance)
            projection = math.cos(bearing) * wall['axis'][0] + math.sin(bearing) * wall['axis'][1]
            if projection > 0.0:
                forward = min(forward, max(0.0, remaining / projection))
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


def _confirmed_wall_board(executive, controller):
    """Enter a unit-tested inspection with two independent nearby measurements."""
    board = Blackboard()
    for _ in range(2):
        executive.now += 0.01
        wall = executive._verify_center_fire_wall('camera')
        if wall is not None:
            controller._record_wall_observation(board, {**wall, 'bearing': 0.0})
    return board


def _enable_wall_probes(executive):
    executive._verify_opportunistic_wall = lambda frame, bearing: {
        **executive._verify_center_fire_wall(frame), 'bearing': bearing}


def _run_verify_controller(executive, controller, board=None, start='approach'):
    """Drive real controller state operations through the new confirmation phase."""
    board = Blackboard() if board is None else board
    state = start
    for _ in range(80):
        operation = {'approach': controller.approach_smoke, 'reacquire': controller.reacquire,
                     'search': controller.search_fire, 'inspect': controller.center_fire,
                     'fire': controller.center_fire}[state]
        state = operation(board)
        if state == 'explore':
            return state
    pytest.fail('Verification did not finish within eighty bounded operations')


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


def test_unconfirmed_fire_diagnostics_retain_raw_detector_progress_without_guidance(system):
    executive, controller = system
    fire(executive, controller, detected=False, status=1,
         confirmation_frames=4, persistence_sec=1.2, track_id=0)

    details = controller.observation_diagnostics('fire')

    assert 'status=1, detected=False' in details
    assert 'accepted=0' in details
    assert 'detector_frames=4' in details
    assert 'detector_persistence=1.20s' in details
    assert 'detector_track=0' in details
    assert 'ready=False, detector_ready=False' in details
    assert controller.fresh_fire() is None and controller.stable_fire() is None


def test_fire_diagnostics_replace_old_confirmation_progress_after_a_candidate_reset(system):
    executive, controller = system
    _qualified_fire(executive, controller, confirmation_frames=8, persistence_sec=2.0)
    assert controller.stable_fire() is not None
    fire(executive, controller, detected=False, status=1,
         confirmation_frames=2, persistence_sec=0.4, track_id=0)

    details = controller.observation_diagnostics('fire')

    assert 'accepted=1' in details
    assert 'detector_frames=2' in details
    assert 'detector_persistence=0.40s' in details
    assert 'ready=False, detector_ready=False' in details
    assert controller.stable_fire() is None


def test_stale_fire_diagnostics_show_raw_progress_without_accepting_the_image(system):
    executive, controller = system
    executive.now += 4.0
    fire(executive, controller, header=header(executive.now - 3.0, 'thermal'),
         confirmation_frames=7, persistence_sec=2.1, track_id=9)

    details = controller.observation_diagnostics('fire')

    assert 'source image is too old' in details
    assert 'accepted=0' in details
    assert 'detector_frames=7' in details
    assert 'detector_persistence=2.10s' in details
    assert 'detector_track=9' in details
    assert 'ready=False, detector_ready=False' in details
    assert controller.stable_fire() is None


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
    board = _confirmed_wall_board(executive, controller)
    assert getattr(controller, operation)(board) == 'fire'
    assert controller.center_fire(board) == 'explore'
    assert any(e[0] == 'inspection_report' and e[2] is not None for e in executive.events)
    assert not any(e[0] in {'navigation', 'plan'} for e in executive.events)


def test_default_fire_policy_approaches_smoke_wall_without_fire_range(system):
    executive, _ = system
    executive.wall_x = 30.0
    executive.config.pop('verify_fire_approach_max_steps')
    executive.config.pop('verify_smoke_standoff')
    executive.config.pop('verify_smoke_inspection_max_distance')
    controller = VerifyController(executive)
    executive._verify = controller
    executive.config['verify_smoke_standoff'] = 15.0
    executive.config['verify_smoke_inspection_max_distance'] = 20.0
    executive.target_geometry.update(point=None, target_valid=False, range=math.nan)
    feed_fire(executive, controller)
    _enable_wall_probes(executive)
    assert _run_verify_controller(executive, controller) == 'explore'
    assert executive.odometry.pose.pose.position.x == pytest.approx(12.0)
    assert len([e for e in executive.events if e[0] == 'navigation']) == 4
    assert any(e[0] == 'inspection_report' and e[2].detected for e in executive.events)


def test_fire_coordinates_do_not_control_center_fire_motion(system):
    executive, controller = system
    executive.wall_x = 13.5
    executive.target_geometry.update(point=Point(x=1.0), target_valid=True, uncertainty=0.25)
    feed_fire(executive, controller)
    assert _run_verify_controller(executive, controller) == 'explore'
    request = next(e for e in executive.events if e[0] == 'navigation')
    assert 'fire_keepout_radius' not in request[2]
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_smoke_wall_at_ten_metres_stops_even_if_fire_range_is_unknown(system):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 3
    executive.odometry.pose.pose.position.x = 9.5
    executive.target_geometry.update(point=None, target_valid=False)
    feed_fire(executive, controller)
    assert controller.center_fire(_confirmed_wall_board(executive, controller)) == 'explore'
    assert not any(e[0] == 'navigation' for e in executive.events)


def test_missing_wall_measurement_holds_without_inventing_distance(system):
    executive, controller = system
    executive._verify_center_fire_wall = lambda frame: None
    feed_fire(executive, controller)
    assert controller.center_fire(Blackboard()) == 'approach'
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_blocked_fire_approach_retries_then_holds_without_reporting_success(system):
    executive, controller = system
    executive.wall_x = 20.0
    feed_fire(executive, controller)
    before = controller._pose()

    def blocked(*args, **kwargs):
        kwargs['guard']()
        executive.events.append(('blocked_fire_plan',))
        return False
    executive._verify_navigation_viewpoint = blocked
    board = Blackboard()
    controller.approach_smoke(board)
    controller.approach_smoke(board)
    with pytest.raises(RuntimeError, match='Verify blocked'):
        controller.approach_smoke(board)
    assert controller._pose() == before
    assert not any(e[0] in {'inspection_report', 'completed_site'} for e in executive.events)


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
            controller.approach_smoke(Blackboard())
    else:
        assert controller.approach_smoke(Blackboard()) == 'reacquire'
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
    assert controller.center_fire(_confirmed_wall_board(executive, controller)) == 'explore'
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
    outcome = controller.center_fire(_confirmed_wall_board(executive, controller))
    assert outcome == ('explore' if failure == 'rejected' else 'fire')
    if failure == 'rejected':
        assert any(e[0] == 'rotate' for e in executive.events)
    assert any(e[0] == 'blocked_vertical' for e in executive.events)
    assert any(e[0] == 'inspection_report' for e in executive.events) == (failure == 'rejected')


def test_fire_centering_cannot_report_a_cached_pre_motion_frame(system):
    executive, controller = system
    _qualified_fire(executive, controller, yaw_correction_rad=0.3)
    assert controller.center_fire(_confirmed_wall_board(executive, controller)) == 'reacquire'
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_fire_lost_during_vertical_correction_requires_new_confirmation(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 5
    feed_fire(executive, controller, vertical_correction_rad=-0.3)

    def lose():
        fire(executive, controller, detected=False)
        executive.on_wait = lambda: fire(executive, controller, detected=False)
    executive.on_motion = lose
    assert controller.center_fire(_confirmed_wall_board(executive, controller)) == 'reacquire'
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_fire_recovery_does_not_publish_a_negative_when_smoke_remains(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    board = _confirmed_wall_board(executive, controller)
    controller._route_fire(board)
    fire(executive, controller, detected=False)
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    _enable_wall_probes(executive)
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
    assert controller.reacquire(board) == 'approach'


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
    assert controller.search_fire(board) == 'approach'
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert verification._bb_get(board, 'verify_inspection_started', None) is None


def test_measured_boundary_starts_shared_budget_even_if_later_measurement_disappears(system):
    executive, controller = system
    feed_smoke(executive, controller)
    _enable_wall_probes(executive)
    board = _confirmed_wall_board(executive, controller)
    started = board['verify_inspection_started']
    executive._verify_opportunistic_wall = lambda *args: None
    assert controller.search_fire(board) == 'reacquire'
    assert controller.center_fire(board) == 'reacquire'
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
    board = Blackboard()
    assert controller.center_fire(board) == 'approach'
    assert not any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('delay', ['query', 'hold'])
def test_wall_wait_cannot_start_another_leg_after_inspection_deadline(system, delay):
    executive, controller = system
    controller.config.update(verify_fire_approach_max_steps=3, verify_inspection_timeout=2.0)
    feed_fire(executive, controller)
    board = _confirmed_wall_board(executive, controller)
    query = executive._verify_center_fire_wall

    def delayed_wall(frame):
        if delay == 'hold':
            return None
        executive.now += 3.0
        _qualified_fire(executive, controller)
        return query(frame)
    executive._verify_center_fire_wall = delayed_wall
    with pytest.raises(RuntimeError, match='inspection time budget exhausted'):
        for _ in range(12):
            controller.center_fire(board)
    assert not any(e[0] in {'navigation', 'inspection_report'} for e in executive.events)


def test_search_uses_only_smoke_geometry_for_translation(system):
    executive, controller = system
    feed_smoke(executive, controller)
    executive.target_geometry.update(point=Point(x=10000.0), range=10000.0, target_valid=True)
    board = Blackboard()
    assert controller.search_fire(board) == 'approach'
    assert controller.approach_smoke(board) == 'reacquire'
    navigation = next(e for e in executive.events if e[0] == 'navigation')
    assert navigation[2]['source'] == 'smoke'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_observed_smoke_boundary_prevents_another_forward_leg(system):
    executive, controller = system
    feed_smoke(executive, controller)
    _enable_wall_probes(executive)
    board = _confirmed_wall_board(executive, controller)
    assert controller.search_fire(board) == 'inspect'
    assert not any(e[0] == 'navigation' for e in executive.events)


def test_broad_smoke_with_unknown_boundary_cannot_publish_negative(system):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    board = Blackboard()
    assert controller.center_fire(board) == 'approach'
    assert verification._bb_get(board, 'verify_inspection_started', None) is None
    assert not any(e[0] == 'inspection_report' for e in executive.events)


@pytest.mark.parametrize('initial_bearing', [None, 0.0])
def test_three_completed_boundary_windows_can_report_no_fire(system, initial_bearing):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    _enable_wall_probes(executive)
    board = _confirmed_wall_board(executive, controller)
    board['verify_smoke_bearing'] = initial_bearing
    assert controller.center_fire(board) == 'explore'
    reports = [e for e in executive.events if e[0] == 'inspection_report']
    assert len(reports) == 1 and reports[0][2] is None and reports[0][3] == 3
    assert len([e for e in executive.events if e[0] == 'recovery_plan']) == 2


def test_blocked_boundary_viewpoint_never_counts_as_completed_inspection(system):
    executive, controller = system
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    _enable_wall_probes(executive)
    board = _confirmed_wall_board(executive, controller)
    executive._verify_recovery_plan_to = lambda *a, **k: False
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
    assert controller.approach_smoke(board) == 'approach'
    executive.map_available = False
    with pytest.raises(RuntimeError, match='Verify blocked'):
        controller.approach_smoke(board)
    assert not verification._bb_get(board, 'verify_exploration_required', False)


@pytest.mark.parametrize('error_type', [
    verification.VerificationMapUnavailable, verification.VerificationRecoveryUnavailable])
def test_fire_found_during_map_refresh_does_not_get_abandoned(system, error_type):
    executive, controller = system
    feed_smoke(executive, controller)
    navigate = executive._verify_navigation_viewpoint

    def unavailable(*a, **k):
        raise error_type('map or clearance unavailable')
    executive._verify_navigation_viewpoint = unavailable
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'approach'
    executive._verify_navigation_viewpoint = navigate
    feed_fire(executive, controller)
    assert _run_verify_controller(executive, controller, board) == 'explore'
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
    expected = 'approach'  # Fresh fire alone does not establish near-wall arrival.
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
        if operation in {'approach_smoke', 'search_fire'}:
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
    assert controller.center_fire(_confirmed_wall_board(executive, controller)) == 'explore'
    assert any(e[0] == 'blocked_horizontal' for e in executive.events)
    assert not any(e[0] == 'recovery_plan' for e in executive.events)
    assert any(e[0] == 'inspection_report' for e in executive.events)


def test_lost_confirmed_fire_with_smoke_eventually_holds_without_false_negative(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    board = _confirmed_wall_board(executive, controller)
    controller._route_fire(board)
    fire(executive, controller, detected=False)
    feed_smoke(executive, controller, coverage=0.3, regions=[1, 2, 3, 4])
    _enable_wall_probes(executive)
    for index in range(controller.config['verify_search_recovery_max'] - 1):
        assert controller.center_fire(board) == 'reacquire'
        assert board['verify_fire_loss_failures'] == index + 1
        assert verification._bb_get(board, 'verify_search_failures', 0) == 0
    with pytest.raises(RuntimeError, match='Verify blocked'):
        controller.center_fire(board)
    assert board['verify_fire_loss_failures'] == controller.config['verify_search_recovery_max']
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
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
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
    assert _run_verify_controller(executive, controller) == 'explore'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)


def test_center_fire_retreats_if_smoke_wall_is_already_too_close(system):
    executive, controller = system
    executive.wall_x = 20.0
    controller.config['verify_fire_approach_max_steps'] = 1
    executive.odometry.pose.pose.position.x = 12.0
    feed_fire(executive, controller)
    assert _run_verify_controller(executive, controller, start='inspect') == 'explore'
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
    assert _run_verify_controller(executive, controller) == 'explore'
    goals = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(goals) == 2
    assert goals[0].position.x == pytest.approx(0.4 / math.tan(abs(vertical)))
    assert goals[0].position.z == pytest.approx(6.0 + math.copysign(0.5, vertical))
    assert goals[-1].position.x == pytest.approx(3.0)


def test_fire_near_vertical_edge_advances_and_changes_height_in_one_leg(system):
    executive, controller = system
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    board = Blackboard()
    assert _run_verify_controller(executive, controller, board) == 'explore'
    goals = [e[1] for e in executive.events if e[0] == 'plan']
    assert len(goals) == 2
    assert goals[0].position.x == pytest.approx(0.4 / math.tan(0.4))
    assert goals[0].position.z == pytest.approx(5.5)
    assert goals[1].position.x == pytest.approx(3.0)
    assert goals[1].position.z == pytest.approx(5.5)
    assert board['verify_fire_approach_steps'] == 1
    assert board['verify_approaches'] == 2
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

    def after_selected_height():
        if executive.events[-1][0] == 'plan':
            executive.on_wait = lambda: _qualified_fire(executive, controller)

    executive.on_motion = after_selected_height
    assert _run_verify_controller(executive, controller, board, start='reacquire') == 'explore'
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=5.5)


def test_vertical_correction_remeasures_wall_before_success(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller, vertical_correction_rad=-0.2)
    executive.on_wait = lambda: _qualified_fire(executive, controller)
    executive.on_motion = lambda: setattr(executive, 'wall_x', 13.5)
    board = _confirmed_wall_board(executive, controller)
    assert _run_verify_controller(executive, controller, board, start='inspect') == 'explore'
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
    assert controller.center_fire(Blackboard()) == 'approach'
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
    controller.config['verify_fire_approach_max_steps'] = 3
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


@pytest.mark.parametrize('invalid', ['clock_frozen', 'image_age', 'distance', 'yaw', 'track'])
def test_expired_or_displaced_fire_hint_never_commands_height(system, invalid, monkeypatch):
    executive, controller = system
    board = Blackboard()
    _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    controller._capture_height_hint(board)
    fire(executive, controller, detected=False)
    if invalid == 'clock_frozen':
        executive.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(executive.now * 1e9)),
            ros_time_is_active=True)
        controller._height_clock_progress()
        original_now = executive.now
        monkeypatch.setattr(verification.time, 'monotonic', lambda: original_now + 16.0)
    elif invalid == 'image_age':
        executive.now += 16.0
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
def test_visible_edge_fire_probes_wall_before_combined_movement(system, angle):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    executive.wall_x = 50.0
    _qualified_fire(executive, controller, vertical_correction_rad=angle)
    executive.on_motion = lambda: fire(executive, controller, detected=False)
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    events = [event[0] for event in executive.events]
    assert events.index('wall_probe') < events.index('navigation')
    assert executive.odometry.pose.pose.position.x > 0.3
    assert executive.odometry.pose.pose.position.z == pytest.approx(
        6.0 + math.copysign(0.5, angle))


def test_measured_arrival_survives_missing_probes_without_forward_motion_or_report(system):
    executive, controller = system
    feed_smoke(executive, controller)
    board = _confirmed_wall_board(executive, controller)
    started = board['verify_inspection_started']
    executive._verify_opportunistic_wall = lambda *args: None
    assert controller.approach_smoke(board) == 'reacquire'
    assert controller.reacquire(board) == 'approach'
    assert controller.search_fire(board) == 'reacquire'
    assert controller.center_fire(board) == 'reacquire'
    assert board['verify_inspection_started'] == started
    assert not any(e[0] in {'navigation', 'plan', 'inspection_report'} for e in executive.events)


def test_one_quadrant_smoke_at_measured_wall_completes_three_attempt_negative_inspection(system):
    executive, controller = system
    feed_smoke(executive, controller, regions=[1], coverage=0.02)
    _enable_wall_probes(executive)
    assert _run_verify_controller(executive, controller) == 'explore'
    report = next(e for e in executive.events if e[0] == 'inspection_report')
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


def test_legacy_smoke_ray_cannot_bypass_aperture_confirmation(system):
    executive, controller = system
    board = Blackboard()
    feed_smoke(executive, controller)
    executive.target_geometry.update(plume_candidate=True, obstacle_range=8.0)
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == pytest.approx(3.0)
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)
    assert verification._bb_get(board, 'verify_inspection_started', None) is None


def test_height_image_already_used_for_visible_correction_is_not_reused_as_memory(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    board = Blackboard()
    image = _qualified_fire(executive, controller, vertical_correction_rad=0.4)
    controller._capture_height_hint(board)
    assert controller._fire_vertical_preference(board, image) == 0.5
    controller._move_visible_fire_height(board, 0.5)
    fire(executive, controller, detected=False)
    assert not controller._recover_fire_height(board)
    assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 1


def test_arrival_at_smoke_leg_limit_still_enters_inspection(system):
    executive, controller = system
    feed_smoke(executive, controller)
    _enable_wall_probes(executive)
    board = _confirmed_wall_board(executive, controller)
    board['verify_approaches'] = controller.config['verify_max_approaches']
    assert controller.approach_smoke(board) == 'search'
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
        assert controller.center_fire(_confirmed_wall_board(executive, controller)) == 'reacquire'
        assert executive.odometry.pose.pose.position == Point(z=5.5)
    else:
        with pytest.raises(MissionCanceled if interrupt == 'cancel' else MissionStateRequested):
            controller.center_fire(_confirmed_wall_board(executive, controller))
        assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def _slow_height_hint(system, monkeypatch, wall_seconds, angle=-0.2):
    """Finish one 3 m leg in six source seconds with an independently slow clock."""
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 8
    wall = [executive.now]
    monkeypatch.setattr(verification.time, 'monotonic', lambda: wall[0])
    executive.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(executive.now * 1e9)),
        ros_time_is_active=True)
    board = Blackboard()
    _qualified_fire(executive, controller, vertical_correction_rad=angle)
    controller._movement_guard(board)
    fire(executive, controller, detected=False)
    source_start, wall_start = executive.now, wall[0]
    for second in range(1, wall_seconds + 1):
        wall[0] = wall_start + second
        executive.now = source_start + 6.0 * second / wall_seconds
        executive.odometry.pose.pose.position.x = 3.0 * second / wall_seconds
        controller._movement_guard(board)
    monkeypatch.setattr(controller, '_inspection_wait_fire', lambda _: None)
    monkeypatch.setattr(controller, '_smoke_guidance', lambda **kwargs: controller.fresh_smoke())
    return executive, controller, board, wall


@pytest.mark.parametrize('wall_seconds', [30, 60, 150])
@pytest.mark.parametrize('angle', [-0.2, 0.2])
@pytest.mark.parametrize('operation', ['approach_smoke', 'reacquire', 'search_fire'])
def test_slow_leg_preserves_moderate_height_guidance_until_arrival(
        system, monkeypatch, wall_seconds, angle, operation):
    executive, controller, board, _ = _slow_height_hint(
        system, monkeypatch, wall_seconds, angle)
    assert controller.stable_fire() is None
    assert getattr(controller, operation)(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(
        x=3.0, z=6.0 + math.copysign(0.5, angle))
    assert not controller._recover_fire_height(board)
    assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 1
    assert not any(event[0] in {'navigation', 'inspection_report'} for event in executive.events)


@pytest.mark.parametrize('planner_result', [False, True])
@pytest.mark.parametrize('previous_reason', ['', 'terrain_floor'])
def test_slow_height_refusal_or_no_progress_reacquires_without_blind_forward_flight(
        system, monkeypatch, planner_result, previous_reason):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 150)
    executive._verify_height_rejection = previous_reason
    executive._verify_recovery_plan_to = lambda *args, **kwargs: planner_result
    assert controller.reacquire(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=6.0)
    assert board['verify_height_only_attempted']
    assert not controller._recover_fire_height(board)
    assert not any(event[0] in {'navigation', 'inspection_report'} for event in executive.events)


@pytest.mark.parametrize('wall_seconds', [30, 150])
def test_slow_floor_blocked_memory_reacquires_then_reconsiders_descent_after_forward_leg(
        system, monkeypatch, wall_seconds):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, wall_seconds)
    executive.wall_x = 50.0
    recovery = executive._verify_recovery_plan_to
    blocked = []
    cliff_reached = [False]

    def terrain_checked_move(target, **kwargs):
        if target.position.z < executive.odometry.pose.pose.position.z and not cliff_reached[0]:
            blocked.append(deepcopy(target.position))
            executive._verify_height_rejection = 'terrain_floor'
            return False
        return recovery(target, **kwargs)

    executive._verify_recovery_plan_to = terrain_checked_move
    smoke(executive, controller)
    assert controller.reacquire(board) == 'approach'
    assert not blocked
    assert controller.approach_smoke(board) == 'reacquire'
    started = board['verify_inspection_started']
    assert executive.odometry.pose.pose.position == Point(x=6.0, z=6.0)
    assert len(blocked) == 2  # Combined first, then the one height-only attempt.
    assert not board['verify_height_only_attempted']
    assert not board['verify_descent_deferred']
    assert verification._bb_get(board, 'verify_search_failures', 0) == 0

    cliff_reached[0] = True
    _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x > 6.3
    assert executive.odometry.pose.pose.position.z == pytest.approx(5.5)
    assert board['verify_inspection_started'] == started
    assert len(blocked) == 2
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_floor_blocked_memory_does_not_authorize_translation_without_fresh_source(
        system, monkeypatch):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 60)

    def reject(*args, **kwargs):
        executive._verify_height_rejection = 'terrain_floor'
        return False

    executive._verify_recovery_plan_to = reject
    monkeypatch.setattr(controller, '_smoke_guidance', lambda **kwargs: None)
    assert controller.reacquire(board) == 'reacquire'
    assert board['verify_descent_deferred']
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=6.0)
    assert not any(event[0] in {'navigation', 'plan', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('hover_drift', [0.0, 0.02, -0.02, 0.2])
def test_preflight_floor_veto_remains_recoverable_through_hover_drift(
        system, monkeypatch, hover_drift):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 60)

    def reject_before_flight(*args, **kwargs):
        executive.odometry.pose.pose.position.z += hover_drift
        executive._verify_height_rejection = 'terrain_floor'
        return False

    executive._verify_recovery_plan_to = reject_before_flight
    assert controller.reacquire(board) == 'reacquire'
    assert board['verify_descent_deferred']
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=6.0 + hover_drift)
    assert not controller._recover_fire_height(board)
    assert not any(event[0] in {'navigation', 'plan', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('reason', ['terrain_unavailable', ''])
def test_remembered_height_candidate_refusal_uses_bounded_reacquisition(
        system, monkeypatch, reason):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 60)

    def reject(*args, **kwargs):
        executive._verify_height_rejection = reason
        return False

    executive._verify_recovery_plan_to = reject
    assert controller.reacquire(board) == 'reacquire'
    assert board['verify_height_only_attempted']
    assert not verification._bb_get(board, 'verify_descent_deferred', False)
    assert not any(event[0] in {'navigation', 'plan', 'inspection_report'}
                   for event in executive.events)


def test_remembered_descent_map_outage_is_not_a_skipped_height_preference(system, monkeypatch):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 60)

    def no_map(*args, **kwargs):
        raise verification.VerificationMapUnavailable('map source expired')

    executive._verify_recovery_plan_to = no_map
    with pytest.raises(verification.VerificationMapUnavailable, match='map source expired'):
        controller._recover_fire_height(board)
    assert not verification._bb_get(board, 'verify_descent_deferred', False)


@pytest.mark.parametrize('forward_up', [0.0, 0.4])
def test_visible_edge_floor_veto_uses_checked_forward_leg_without_blocked_recovery(
        system, forward_up):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 8
    executive.wall_x = 50.0
    recovery = executive._verify_recovery_plan_to
    blocked = []

    def terrain_checked_move(target, **kwargs):
        if target.position.z < executive.odometry.pose.pose.position.z:
            blocked.append(deepcopy(target.position))
            executive._verify_height_rejection = 'terrain_floor'
            return False
        target.position.z += forward_up
        return recovery(target, **kwargs)

    executive._verify_recovery_plan_to = terrain_checked_move
    _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=6.0 + forward_up)
    assert len(blocked) == 2
    assert not board['verify_descent_deferred']
    assert verification._bb_get(board, 'verify_search_failures', 0) == 0
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_failed_forward_leg_does_not_reset_the_one_height_only_attempt(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 8
    executive.wall_x = 50.0
    attempts = []

    def terrain_checked_move(target, **kwargs):
        attempts.append(deepcopy(target.position))
        if target.position.z < executive.odometry.pose.pose.position.z:
            executive._verify_height_rejection = 'terrain_floor'
        return False

    executive._verify_recovery_plan_to = terrain_checked_move
    board = Blackboard()
    for _ in range(2):
        _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
        assert controller.approach_smoke(board) == 'reacquire'
        assert board['verify_height_only_attempted']
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    pure_height = [target for target in attempts if target.z < 6.0 and abs(target.x) < 1e-6]
    assert len(pure_height) == 1


def test_deferred_height_attempt_does_not_disable_combined_guidance_in_either_direction(system):
    executive, controller = system
    board = Blackboard()
    board['verify_descent_deferred'] = True
    board['verify_height_only_attempted'] = True
    lower = _qualified_fire(executive, controller, vertical_correction_rad=-0.4)
    assert controller._fire_vertical_preference(board, lower) == -0.5
    upper = _qualified_fire(executive, controller, vertical_correction_rad=0.4)
    assert controller._fire_vertical_preference(board, upper) == 0.5
    assert board['verify_height_only_attempted']


def test_floor_blocked_height_at_wall_never_authorizes_forward_fallback(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 8
    board = _confirmed_wall_board(executive, controller)
    feed_fire(executive, controller, vertical_correction_rad=-0.4)

    def reject(*args, **kwargs):
        executive._verify_height_rejection = 'terrain_floor'
        return False

    executive._verify_recovery_plan_to = reject
    assert controller.center_fire(board) == 'explore'
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(event[0] == 'navigation' for event in executive.events)
    assert any(event[0] == 'inspection_report' and event[2] is not None
               for event in executive.events)


def test_slow_leg_does_not_preserve_height_guidance_through_a_frozen_clock(system, monkeypatch):
    executive, controller, board, wall = _slow_height_hint(system, monkeypatch, 60)
    wall[0] += 6.0
    assert not controller._recover_fire_height(board)
    assert not any(event[0] == 'recovery_plan' for event in executive.events)
    assert any('source clock unavailable or frozen' in event[1]
               for event in executive.events if event[0] == 'status')


def test_slow_height_hint_is_rejected_after_source_clock_rewind(system, monkeypatch):
    executive, controller, board, wall = _slow_height_hint(system, monkeypatch, 60)
    executive.now -= 1.0
    wall[0] += 1.0
    assert not controller._recover_fire_height(board)
    assert any('source clock rewound' in event[1]
               for event in executive.events if event[0] == 'status')


def test_temporary_yaw_rejection_does_not_consume_pending_height_hint(system, monkeypatch):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 30)
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(math.radians(20.0))
    assert not controller._recover_fire_height(board)
    assert verification._bb_get(board, 'verify_height_hint_consumed', 0) == 0
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(0.0)
    assert controller._recover_fire_height(board)
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=5.5)


def test_unused_height_hint_cannot_cross_a_second_forward_leg(system, monkeypatch):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 30)
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(math.radians(20.0))
    assert not controller._recover_fire_height(board)
    controller._begin_forward_leg(board)
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(0.0)
    assert not controller._recover_fire_height(board)
    assert not any(event[0] == 'recovery_plan' for event in executive.events)


def test_height_preference_does_not_consume_an_unselected_recovery(system, monkeypatch):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 30)
    fire_image = board['verify_height_hint'][0]
    # Simulate an earlier decision that computed height preference but never
    # selected a path. Only an actual selected correction consumes the image.
    board['verify_vertical_stamp'] = controller._stamp_ns(fire_image)
    assert controller._recover_fire_height(board)
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=5.5)


def test_center_fire_discards_unused_hint_before_selecting_another_forward_leg(system):
    executive, controller = system
    board = Blackboard()
    executive.wall_x = 13.5
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller)
    controller._capture_height_hint(board)
    executive.on_motion = lambda: fire(executive, controller, detected=False)
    assert controller.center_fire(board) == 'approach'
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_height_hint'] is None
    assert not controller._recover_fire_height(board)
    assert any(event[0] == 'navigation' for event in executive.events)


def _qualified_wall(executive, distance, stamp=None):
    return {'origin': deepcopy(executive.odometry.pose.pose.position),
            'axis': (1.0, 0.0, 0.0), 'mean': distance + 0.1, 'nearest': distance,
            'uncertainty': 0.1, 'camera_frame': 'camera', 'bearing': 0.0,
            'map_stamp_ns': int(executive.now * 1e9) if stamp is None else stamp}


@pytest.mark.parametrize('operation', ['approach_smoke', 'search_fire', 'center_fire'])
def test_unsettled_preparation_cannot_probe_translate_or_report_and_has_a_bound(system, operation):
    executive, controller = system
    smoke(executive, controller)
    board = Blackboard()
    executive._verify_wall_settle_reason = 'still braking'
    executive._verify_prepare_wall_observation = lambda **kwargs: False
    for _ in range(2):
        assert getattr(controller, operation)(board) == 'reacquire'
    with pytest.raises(RuntimeError, match='stationary observation preparation exhausted'):
        getattr(controller, operation)(board)
    assert board['verify_settle_failures'] == 3
    blocked = {'geometry', 'wall_probe', 'plan', 'navigation', 'inspection_report'}
    assert not any(event[0] in blocked
                   for event in executive.events)


def test_preparation_rechecks_post_settle_image_before_geometry_or_navigation(system):
    executive, controller = system
    old = smoke(executive, controller)
    board = Blackboard()
    settled = []

    def prepare(guard=None, *, align_fire=True):
        executive.now += 0.4
        if guard:
            guard()
        settled.append(int(executive.now * 1e9))
        executive._verify_wall_observation_context = {'settled_ns': settled[-1]}
        return True

    executive._verify_prepare_wall_observation = prepare
    executive.on_wait = lambda: smoke(executive, controller)
    assert controller.approach_smoke(board) == 'reacquire'
    images = [event[2] for event in executive.events if event[0] == 'geometry']
    assert images and images[0].header.stamp != old.header.stamp
    assert controller._stamp_ns(images[0]) > settled[0]
    assert any(event[0] == 'navigation' for event in executive.events)


def test_pre_settle_camera_frame_cannot_start_a_new_approach_leg(system):
    executive, controller = system
    smoke(executive, controller)
    board = Blackboard()

    def prepare(guard=None, *, align_fire=True):
        executive._verify_wall_observation_context = {'settled_ns': int(executive.now * 1e9)}
        return True

    executive._verify_prepare_wall_observation = prepare
    assert controller.approach_smoke(board) == 'reacquire'
    assert not any(event[0] in {'geometry', 'wall_probe', 'navigation', 'inspection_report'}
                   for event in executive.events)


def test_unready_physical_context_cannot_count_a_wall_vote_or_reuse_arrival(system):
    executive, controller = system
    board = Blackboard()
    executive._verify_wall_observation_ready = lambda: False
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    assert board['verify_wall_latest'] is None
    assert verification._bb_get(board, 'verify_wall_near_count', 0) == 0
    executive._verify_wall_observation_ready = lambda: True
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 102))
    controller._record_wall_observation(board, _qualified_wall(executive, 10.3, 103))
    assert controller._at_known_wall(board)
    executive._verify_wall_observation_ready = lambda: False
    assert not controller._at_known_wall(board)
    assert controller._wall_action(board) == 'reacquire'
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def _query_loses_settling(system, *, available=True, after_record=False):
    executive, controller = system
    ready = [True]
    original_prepare = executive._verify_prepare_wall_observation
    original_record = controller._record_wall_observation

    def prepare(**kwargs):
        ready[0] = True
        return original_prepare(**kwargs)

    def probe(*args):
        executive.events.append(('wall_probe',))
        ready[0] = after_record
        return _qualified_wall(executive, 10.4) if available else None

    def record(board, wall):
        original_record(board, wall)
        ready[0] = False

    executive._verify_prepare_wall_observation = prepare
    executive._verify_wall_observation_ready = lambda: ready[0]
    executive._verify_opportunistic_wall = probe
    executive._verify_center_fire_wall = probe
    if after_record:
        controller._record_wall_observation = record
    return ready


@pytest.mark.parametrize('operation', ['approach_smoke', 'search_fire', 'center_fire'])
@pytest.mark.parametrize('available', [False, True])
@pytest.mark.parametrize('after_record', [False, True])
def test_query_time_settling_loss_holds_before_any_movement_or_report(
        system, operation, available, after_record):
    executive, controller = system
    smoke(executive, controller)
    board = Blackboard()
    _query_loses_settling(system, available=available, after_record=after_record)
    assert getattr(controller, operation)(board) == 'reacquire'
    assert board['verify_settle_failures'] == 1
    assert board['verify_wall_latest'] is None
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(event[0] in {'plan', 'navigation', 'recovery_plan', 'inspection_report'}
                   for event in executive.events)
    assert any(event[0] == 'hold' for event in executive.events)
    if not (available and after_record):
        assert verification._bb_get(board, 'verify_wall_recheck_deadline', None) is None


def test_successful_preparation_cannot_hide_repeated_post_query_settling_losses(system):
    executive, controller = system
    board = Blackboard()
    _query_loses_settling(system)
    for expected in (1, 2):
        smoke(executive, controller)
        assert controller.approach_smoke(board) == 'reacquire'
        assert board['verify_settle_failures'] == expected
    smoke(executive, controller)
    with pytest.raises(RuntimeError, match='stationary observation preparation exhausted'):
        controller.approach_smoke(board)
    assert board['verify_settle_failures'] == 3
    assert not any(event[0] in {'plan', 'navigation', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('available', [False, True])
def test_unsettled_query_preserves_history_and_both_original_deadlines(system, available):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 9.9, 101))
    board['verify_inspection_started'] = executive.now - 7.0
    history = {key: deepcopy(board[key]) for key in (
        'verify_wall_last_stamp_ns', 'verify_wall_near_count', 'verify_wall_close_count',
        'verify_wall_recheck_deadline', 'verify_inspection_started', 'verify_smoke_walls')}
    executive._verify_wall_observation_ready = lambda: False
    controller._record_wall_observation(
        board, _qualified_wall(executive, 10.3, 102) if available else None)
    assert controller._wall_action(board) == 'reacquire'
    assert board['verify_wall_latest'] is None
    assert all(board[key] == value for key, value in history.items())
    assert not any(event[0] in {'plan', 'recovery_plan', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('distance', [None, 10.4, 50.0])
def test_only_a_physically_valid_wall_decision_resets_settling_failure_budget(system, distance):
    executive, controller = system
    board = Blackboard()
    board['verify_settle_failures'] = 2
    assert controller._prepare_wall_observation(board)
    assert board['verify_settle_failures'] == 2
    controller._record_wall_observation(
        board, None if distance is None else _qualified_wall(executive, distance))
    assert controller._wall_action(board) == ('reacquire' if distance == 10.4 else 'approach')
    assert board['verify_settle_failures'] == 0


@pytest.mark.parametrize('vertical', [-0.2, 0.2])
def test_unsettled_wall_decision_blocks_center_fire_height_fallback(system, vertical):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller, vertical_correction_rad=vertical)
    _query_loses_settling(system)
    board = Blackboard()
    assert controller.center_fire(board) == 'reacquire'
    assert board['verify_settle_failures'] == 1
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(event[0] in {'recovery_plan', 'plan', 'navigation', 'inspection_report'}
                   for event in executive.events)


def test_inner_fire_approach_missing_result_uses_settling_hold_before_sensor_recheck(system):
    executive, controller = system
    feed_fire(executive, controller)
    _query_loses_settling(system, available=False)
    board = Blackboard()
    assert controller._approach_visible_fire(board, controller.stable_fire()) is None
    assert board['verify_settle_failures'] == 1
    assert verification._bb_get(board, 'verify_wall_recheck_deadline', None) is None
    assert not any(event[0] in {'recovery_plan', 'plan', 'navigation', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('clock', ['near', 'inspection'])
def test_settling_loss_hold_enforces_the_original_deadlines(system, clock):
    executive, controller = system
    board = Blackboard()
    if clock == 'near':
        board['verify_wall_recheck_deadline'] = executive.now + 0.1
        message = 'near-wall revalidation exhausted'
    else:
        board['verify_inspection_started'] = (
            executive.now - controller.config['verify_inspection_timeout'] + 0.1)
        message = 'inspection time budget exhausted'
    executive._verify_wall_observation_ready = lambda: False
    with pytest.raises(RuntimeError, match=message):
        controller._wall_action(board)
    assert executive.now == pytest.approx(100.1)


def test_preparation_cannot_reset_near_deadline_or_main_inspection_clock(system):
    executive, controller = system
    board = Blackboard()
    board['verify_inspection_started'] = executive.now - 5.0
    started = board['verify_inspection_started']
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    deadline = board['verify_wall_recheck_deadline']

    def expire(guard=None, *, align_fire=True):
        executive.now = deadline
        guard()
        return True

    executive._verify_prepare_wall_observation = expire
    with pytest.raises(RuntimeError, match='near-wall revalidation exhausted'):
        controller.approach_smoke(board)
    assert board['verify_wall_recheck_deadline'] == deadline
    assert board['verify_inspection_started'] == started
    assert not any(event[0] in {'geometry', 'wall_probe', 'navigation', 'inspection_report'}
                   for event in executive.events)


def test_first_fresh_fire_starts_main_clock_before_heading_and_settling_wait(system):
    executive, controller = system
    feed_fire(executive, controller)
    board = Blackboard()
    before = executive.now

    def unavailable(guard=None, *, align_fire=True):
        executive.now += 2.0
        guard()
        return False

    executive._verify_prepare_wall_observation = unavailable
    assert controller.approach_smoke(board) == 'reacquire'
    assert board['verify_inspection_started'] == before
    assert executive.now - board['verify_inspection_started'] >= 2.0


def test_fire_arriving_during_stationary_preparation_starts_clock_without_renewing_it(system):
    executive, controller = system
    board = Blackboard()
    first_seen = []

    def prepare(guard=None, *, align_fire=True):
        executive.now += 0.4
        feed_fire(executive, controller)
        first_seen.append(executive.now)
        guard()
        for _ in range(3):
            executive.now += 0.2
            guard()
        return True

    executive._verify_prepare_wall_observation = prepare
    assert controller._prepare_wall_observation(board)
    assert board['verify_inspection_started'] == first_seen[0]
    assert executive.now - board['verify_inspection_started'] >= 0.6 - 1e-6


@pytest.mark.parametrize('phase', ['pending_vote', 'post_retreat'])
def test_missing_sources_during_wall_recheck_never_scan_or_move_height(system, phase):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    _qualified_fire(executive, controller, vertical_correction_rad=-0.2)
    board = Blackboard()
    controller._capture_height_hint(board)
    fire(executive, controller, detected=False)
    started = executive.now - 2.0
    board['verify_inspection_started'] = started
    if phase == 'pending_vote':
        controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
        deadline = board['verify_wall_recheck_deadline']
    else:
        board['verify_smoke_boundary_reached'] = True
        board['verify_wall_recheck_after_settle'] = True
        deadline = executive.now + 1.0 + controller.config['verify_wall_recheck_timeout']

        def settle(guard=None, *, align_fire=True):
            assert not align_fire
            executive.now += 1.0
            guard()
            return True

        executive._verify_prepare_wall_observation = settle
    assert controller.reacquire(board) == 'reacquire'
    assert board['verify_wall_recheck_deadline'] == pytest.approx(deadline)
    assert board['verify_inspection_started'] == started
    assert not any(event[0] in {'rotate', 'recovery_plan', 'navigation', 'inspection_report'}
                   for event in executive.events)
    assert verification._bb_get(board, 'verify_height_hint_consumed', 0) == 0
    executive.now = deadline
    with pytest.raises(RuntimeError, match='near-wall revalidation exhausted'):
        controller.reacquire(board)


def test_pre_arrival_image_disappearing_during_wall_recheck_cannot_trigger_scan(system):
    executive, controller = system
    image = smoke(executive, controller)
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    deadline = board['verify_wall_recheck_deadline']
    board['verify_arrival_after_ns'] = controller._stamp_ns(image) + 1

    def no_guidance_wait(**kwargs):
        raise AssertionError('Pending wall recheck must not enter source grace or yaw scans')

    controller._smoke_guidance = no_guidance_wait
    executive.on_wait = lambda: smoke(executive, controller, detected=False)
    assert controller.reacquire(board) == 'reacquire'
    assert board['verify_wall_recheck_deadline'] == deadline
    assert board['verify_wall_near_count'] == 1
    assert not any(event[0] in {'rotate', 'recovery_plan', 'navigation', 'inspection_report'}
                   for event in executive.events)


def test_pending_wall_vote_keeps_heading_and_its_original_deadline_during_preparation(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    deadline = board['verify_wall_recheck_deadline']
    align = []

    def prepare(guard=None, *, align_fire=True):
        align.append(align_fire)
        guard()
        return True

    executive._verify_prepare_wall_observation = prepare
    assert controller._prepare_wall_observation(board)
    assert align == [False]
    assert board['verify_wall_recheck_deadline'] == deadline
    assert board['verify_wall_near_count'] == 1


def test_inspection_guard_polls_settling_without_resetting_either_deadline(system):
    executive, controller = system
    board = Blackboard()
    board['verify_inspection_started'] = executive.now - 3.0
    board['verify_wall_recheck_deadline'] = executive.now + 20.0
    started, deadline = board['verify_inspection_started'], board['verify_wall_recheck_deadline']
    executive._verify_wall_observation_context = {'settled_ns': 0}
    polls = []

    def lost_settling():
        polls.append(executive.now)
        executive._verify_wall_observation_context = None
        return False

    executive._verify_wall_observation_ready = lost_settling
    controller._inspection_guard(board)
    assert polls == [executive.now]
    assert executive._verify_wall_observation_context is None
    assert board['verify_inspection_started'] == started
    assert board['verify_wall_recheck_deadline'] == deadline


def test_braking_after_confirmed_arrival_does_not_start_a_sensor_recheck_clock(system):
    executive, controller = system
    board = _confirmed_wall_board(executive, controller)
    started = board['verify_inspection_started']
    executive._verify_prepare_wall_observation = lambda **kwargs: False
    assert not controller._prepare_wall_observation(board)
    assert board['verify_wall_latest'] is None
    assert board['verify_wall_near_count'] == 2
    assert board['verify_wall_recheck_deadline'] is None
    assert board['verify_inspection_started'] == started


def test_fire_navigation_bearing_uses_transformed_image_geometry(system):
    executive, controller = system
    message = _qualified_fire(executive, controller, yaw_correction_rad=0.7)
    bearing, geometry = controller._approach_bearing(
        Blackboard(), message, 'fire', {'navigation_bearing': -0.3})
    assert bearing == geometry['navigation_bearing'] == -0.3
    executive._verify_image_bearing = lambda image: None
    bearing, _ = controller._approach_bearing(Blackboard(), message, 'fire', {})
    assert bearing == pytest.approx(0.0)


def test_one_distant_probe_followed_by_missing_returns_never_latches_arrival(system):
    executive, controller = system
    controller.config['verify_smoke_standoff'] = executive.config['verify_smoke_standoff'] = 15.0
    wall = _qualified_wall(executive, 35.0)
    replies = iter([wall, None, None, None, None])
    executive._verify_opportunistic_wall = lambda *args: next(replies)
    board = Blackboard()
    for _ in range(5):
        smoke(executive, controller)
        assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == 15.0
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)
    assert verification._bb_get(board, 'verify_inspection_started', None) is None
    assert board['verify_smoke_walls'] == []


def test_two_new_near_observations_confirm_without_static_wall_agreement(system):
    executive, controller = system
    board = Blackboard()
    first = _qualified_wall(executive, 10.55, 101)
    controller._record_wall_observation(board, first)
    assert controller._wall_action(board) == 'reacquire'
    assert board['verify_wall_near_count'] == 1
    controller._record_wall_observation(board, _qualified_wall(executive, 10.15, 102))
    assert controller._wall_action(board) == 'ready'
    assert board['verify_wall_near_count'] == 2
    assert board['verify_smoke_boundary_reached']


def test_duplicate_map_observation_cannot_confirm_near_arrival(system):
    executive, controller = system
    board = Blackboard()
    first = _qualified_wall(executive, 10.4, 101)
    controller._record_wall_observation(board, first)
    for _ in range(4):
        controller._record_wall_observation(board, first)
        assert controller._wall_action(board) == 'reacquire'
    assert board['verify_wall_near_count'] == 1
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)


@pytest.mark.parametrize('evidence', [
    ('flicker', 'flicker'), ('persistent', 'persistent'),
    ('flicker', 'persistent'), ('persistent', 'flicker'),
])
def test_missing_probes_preserve_pending_near_vote_without_live_range(system, evidence):
    executive, controller = system
    board = Blackboard()
    first = {**_qualified_wall(executive, 10.4, 101), 'evidence': evidence[0]}
    controller._record_wall_observation(board, first)
    deadline = board['verify_wall_recheck_deadline']
    for _ in range(3):
        executive.now += 2.0
        controller._record_wall_observation(board, None)
        assert board['verify_wall_latest'] is None
        assert board['verify_wall_near_count'] == 1
        assert board['verify_wall_recheck_deadline'] == deadline
        assert board['verify_smoke_walls'] == [first]
        assert not controller._at_known_wall(board)
        assert controller._wall_action(board) == 'reacquire'
    controller._record_wall_observation(
        board, {**_qualified_wall(executive, 10.3, 102), 'evidence': evidence[1]})
    assert board['verify_wall_near_count'] == 2
    assert controller._wall_action(board) == 'ready'
    assert not any(event[0] in {'navigation', 'recovery_plan', 'inspection_report'}
                   for event in executive.events)


def test_missing_probes_preserve_close_vote_until_a_second_new_close_map(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0, 101))
    deadline = board['verify_wall_recheck_deadline']
    executive.now += 4.0
    controller._record_wall_observation(board, None)
    assert board['verify_wall_latest'] is None
    assert board['verify_wall_close_count'] == 1
    assert board['verify_wall_recheck_deadline'] == deadline
    assert controller._wall_action(board) == 'reacquire'
    assert not any(event[0] == 'recovery_plan' for event in executive.events)
    controller._record_wall_observation(board, _qualified_wall(executive, 8.2, 102))
    assert board['verify_wall_close_count'] == 2
    assert controller._wall_action(board) == 'reacquire'
    assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 1


@pytest.mark.parametrize('stamp', [100, 101])
@pytest.mark.parametrize('distance', [10.4, 8.0])
def test_replayed_or_reordered_map_after_gap_does_not_refresh_or_confirm(system, stamp, distance):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, distance, 101))
    deadline = board['verify_wall_recheck_deadline']
    controller._record_wall_observation(board, None)
    executive.now += 1.0
    controller._record_wall_observation(board, _qualified_wall(executive, distance - 0.1, stamp))
    assert board['verify_wall_latest'] is None
    assert board['verify_wall_last_stamp_ns'] == 101
    assert board['verify_wall_near_count'] == 1
    assert board['verify_wall_close_count'] == (1 if distance < 10.0 else 0)
    assert board['verify_wall_recheck_deadline'] == deadline
    assert controller._wall_action(board) == 'reacquire'
    assert not controller._at_known_wall(board)
    assert not any(event[0] == 'recovery_plan' for event in executive.events)
    controller._record_wall_observation(board, _qualified_wall(executive, distance - 0.1, 102))
    if distance < 10.0:
        assert controller._wall_action(board) == 'reacquire'
        assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 1
    else:
        assert controller._wall_action(board) == 'ready'


@pytest.mark.parametrize('kind', ['near', 'close', 'far', 'replayed', 'missing'])
def test_expired_wall_window_rejects_reappearing_evidence_before_counting(system, kind):
    executive, controller = system
    board = Blackboard()
    board['verify_inspection_started'] = executive.now - 5.0
    started = board['verify_inspection_started']
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    deadline = board['verify_wall_recheck_deadline']
    controller._record_wall_observation(board, None)
    executive.now = deadline
    distance = {'near': 10.3, 'close': 8.0, 'far': 18.0, 'replayed': 10.3}
    wall = (None if kind == 'missing' else _qualified_wall(
        executive, distance[kind], 101 if kind == 'replayed' else 102))
    with pytest.raises(RuntimeError, match='near-wall revalidation exhausted'):
        controller._record_wall_observation(board, wall)
    assert board['verify_wall_latest'] is None
    assert board['verify_wall_near_count'] == 1
    assert board['verify_wall_last_stamp_ns'] == 101
    assert board['verify_wall_recheck_deadline'] == deadline
    assert board['verify_inspection_started'] == started
    assert not any(event[0] in {'navigation', 'recovery_plan', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('first_distance', [10.4, 8.0])
def test_fresh_far_map_after_gap_clears_pending_vote_and_keeps_main_clock(system, first_distance):
    executive, controller = system
    board = Blackboard()
    board['verify_inspection_started'] = executive.now - 5.0
    started = board['verify_inspection_started']
    controller._record_wall_observation(board, _qualified_wall(executive, first_distance, 101))
    controller._record_wall_observation(board, None)
    executive.now += 2.0
    controller._record_wall_observation(board, _qualified_wall(executive, 18.0, 102))
    assert controller._wall_action(board) == 'approach'
    assert board['verify_wall_near_count'] == board['verify_wall_close_count'] == 0
    assert board['verify_wall_recheck_deadline'] is None
    assert board['verify_inspection_started'] == started
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)
    controller._record_wall_observation(board, _qualified_wall(executive, 10.3, 101))
    assert controller._wall_action(board) == 'approach'


@pytest.mark.parametrize('confirmed', [False, True])
def test_band_votes_then_close_votes_share_bounded_window_across_gaps(system, confirmed):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    deadline = board['verify_wall_recheck_deadline']
    if confirmed:
        controller._record_wall_observation(board, _qualified_wall(executive, 10.3, 102))
        assert board['verify_smoke_boundary_reached']
        deadline = None
    executive.now += 2.0
    expected_deadline = executive.now + 20.0 if deadline is None else deadline
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0, 103))
    assert board['verify_wall_near_count'] == 2
    assert board['verify_wall_close_count'] == 1
    assert board['verify_wall_recheck_deadline'] == expected_deadline
    assert controller._wall_action(board) == 'reacquire'
    assert not any(event[0] == 'recovery_plan' for event in executive.events)
    executive.now += 2.0
    controller._record_wall_observation(board, None)
    assert board['verify_wall_near_count'] == (0 if confirmed else 2)
    assert board['verify_wall_close_count'] == 1
    assert board['verify_wall_recheck_deadline'] == expected_deadline
    controller._record_wall_observation(board, _qualified_wall(executive, 8.2, 104))
    assert controller._wall_action(board) == 'reacquire'
    assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 1


def test_confirmed_band_then_close_gap_needs_new_near_votes_for_arrival(system):
    executive, controller = system
    board = _confirmed_wall_board(executive, controller)
    started = board['verify_inspection_started']
    executive.now += 1.0
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0))
    deadline = board['verify_wall_recheck_deadline']
    controller._record_wall_observation(board, None)
    assert board['verify_wall_near_count'] == 0
    assert board['verify_wall_close_count'] == 1
    assert board['verify_wall_latest'] is None
    executive.now += 1.0
    controller._record_wall_observation(board, _qualified_wall(executive, 10.3))
    assert board['verify_wall_near_count'] == 1
    assert board['verify_wall_close_count'] == 0
    assert board['verify_wall_recheck_deadline'] == deadline
    assert controller._wall_action(board) == 'reacquire'
    assert not controller._at_known_wall(board)
    assert not any(event[0] in {'recovery_plan', 'navigation', 'inspection_report'}
                   for event in executive.events)
    controller._record_wall_observation(board, None)
    executive.now += 1.0
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4))
    assert controller._wall_action(board) == 'ready'
    assert board['verify_inspection_started'] == started


def test_fresh_far_observation_releases_arrival_without_restarting_inspection_clock(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 10.3, 102))
    started = board['verify_inspection_started']
    controller._record_wall_observation(board, _qualified_wall(executive, 18.0, 103))
    assert controller._wall_action(board) == 'approach'
    assert not board['verify_smoke_boundary_reached']
    assert board['verify_wall_near_count'] == 0
    assert board['verify_inspection_started'] == started
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    assert controller._wall_action(board) == 'approach'
    assert board['verify_wall_near_count'] == 0


def test_first_close_observation_holds_without_retreat_or_inspection(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0, 101))
    assert controller._wall_action(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == 0.0
    assert board['verify_wall_near_count'] == 1
    assert board['verify_wall_close_count'] == 1
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)
    assert not any(event[0] in {'recovery_plan', 'navigation', 'inspection_report'}
                   for event in executive.events)


@pytest.mark.parametrize('angle', [-0.2, 0.2])
def test_pending_wall_reconfirmation_preserves_height_hint_without_translation(system, angle):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    board = Blackboard()
    _qualified_fire(executive, controller, vertical_correction_rad=angle)
    controller._capture_height_hint(board)
    fire(executive, controller, detected=False)
    hint = deepcopy(board['verify_height_hint'])
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    controller._record_wall_observation(board, None)
    assert not controller._recover_fire_height(board)
    assert board['verify_height_hint'] == hint
    assert verification._bb_get(board, 'verify_height_hint_consumed', 0) == 0
    assert not verification._bb_get(board, 'verify_height_only_attempted', False)
    assert not any(event[0] == 'recovery_plan' for event in executive.events)
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    controller._record_wall_observation(board, _qualified_wall(executive, 18.0, 102))
    assert controller._recover_fire_height(board)
    assert executive.odometry.pose.pose.position.z == pytest.approx(
        6.0 + math.copysign(0.5, angle))


def test_two_new_close_observations_authorize_one_checked_retreat_then_reconfirm(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 8.1, 102))
    started = board['verify_inspection_started']
    assert controller._wall_action(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x < 0.0
    assert board['verify_wall_latest'] is None
    assert board['verify_wall_near_count'] == 0
    assert board['verify_wall_close_count'] == 0
    assert board['verify_wall_recheck_deadline'] is None
    assert board['verify_wall_recheck_after_settle']
    assert board['verify_inspection_started'] == started
    assert controller._prepare_wall_observation(board)
    assert board['verify_wall_recheck_deadline'] > executive.now
    assert controller._wall_action(board) == 'reacquire'
    assert len([event for event in executive.events if event[0] == 'recovery_plan']) == 1


def test_confirmed_band_does_not_authorize_retreat_from_one_new_close_reading(system):
    executive, controller = system
    board = _confirmed_wall_board(executive, controller)
    executive.now += 0.1
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0))
    assert controller._wall_action(board) == 'reacquire'
    assert board['verify_wall_close_count'] == 1
    assert executive.odometry.pose.pose.position.x == 0.0
    assert not any(event[0] == 'recovery_plan' for event in executive.events)


def test_required_retreat_does_not_consume_stationary_reconfirmation_budget(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 8.1, 102))
    started = board['verify_inspection_started']
    recover = executive._verify_recovery_plan_to

    def slow_recovery(*args, **kwargs):
        executive.now += 34.0
        return recover(*args, **kwargs)

    executive._verify_recovery_plan_to = slow_recovery
    assert controller._wall_action(board) == 'reacquire'
    assert board['verify_wall_recheck_deadline'] is None
    assert board['verify_wall_recheck_after_settle']
    executive.now += 10.0  # Braking/settling is separate from stationary sensor rechecking.
    assert controller._prepare_wall_observation(board)
    assert board['verify_wall_recheck_deadline'] == pytest.approx(executive.now + 20.0)
    assert board['verify_inspection_started'] == started
    controller._record_wall_observation(board, None)
    assert controller._wall_action(board) == 'reacquire'
    assert executive.now - started > 34.0


def test_unavailable_near_wall_revalidation_has_one_bounded_budget(system):
    executive, controller = system
    board = Blackboard()
    controller.config['verify_wall_recheck_timeout'] = 1.0
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    with pytest.raises(RuntimeError, match='near-wall revalidation exhausted'):
        for _ in range(8):
            controller._record_wall_observation(board, None)
            controller._wall_action(board)
    assert executive.now < 102.0
    assert not any(event[0] in {'navigation', 'inspection_report'} for event in executive.events)


def test_missing_smoke_at_confirmed_wall_reacquires_instead_of_search_center_spin(system):
    executive, controller = system
    board = Blackboard()
    controller.config['verify_smoke_loss_grace'] = 0.1
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 10.3, 102))
    started = executive.now
    assert controller.search_fire(board) == 'reacquire'
    assert controller.center_fire(board) == 'reacquire'
    assert executive.now >= started + 0.5
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_visible_fire_without_wall_range_keeps_approach_and_height_guidance(system):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    executive._verify_opportunistic_wall = lambda *args: None
    board = Blackboard()
    feed_fire(executive, controller, vertical_correction_rad=-0.2)
    assert controller.approach_smoke(board) == 'reacquire'
    target = next(e[1] for e in executive.events if e[0] == 'plan')
    assert target.position.x == pytest.approx(0.4 / math.tan(0.2))
    assert target.position.z == pytest.approx(5.5)
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)
    assert not any(e[0] == 'inspection_report' for e in executive.events)


def test_confirmed_wall_needs_two_new_near_readings_after_a_missing_probe(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 10.3, 102))
    assert controller._wall_action(board) == 'ready'
    controller._record_wall_observation(board, None)
    assert board['verify_smoke_boundary_reached']
    deadline = board['verify_wall_recheck_deadline']
    controller._record_wall_observation(board, _qualified_wall(executive, 10.25, 103))
    assert controller._wall_action(board) == 'reacquire'
    controller._record_wall_observation(board, None)
    assert board['verify_wall_latest'] is None
    assert board['verify_wall_near_count'] == 1
    assert board['verify_wall_recheck_deadline'] == deadline
    controller._record_wall_observation(board, _qualified_wall(executive, 10.4, 104))
    assert controller._wall_action(board) == 'ready'


def test_repeated_map_close_candidate_holds_without_confirming_or_retreating(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 18.0, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 8.0, 101))
    assert controller._wall_action(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x == 0.0
    assert board['verify_wall_near_count'] == 0
    assert board['verify_wall_close_count'] == 0
    assert not verification._bb_get(board, 'verify_smoke_boundary_reached', False)


@pytest.mark.parametrize('vertical', [-0.2, 0.2])
def test_pending_near_wall_recheck_does_not_block_safe_fire_height_correction(system, vertical):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller, vertical_correction_rad=vertical)
    board = _confirmed_wall_board(executive, controller)
    executive._verify_opportunistic_wall = lambda *args: None

    def post_motion_fire():
        if executive.events[-1][0] == 'plan':
            executive.on_wait = lambda: _qualified_fire(executive, controller)

    executive.on_motion = post_motion_fire
    assert controller.center_fire(board) == 'reacquire'
    assert executive.odometry.pose.pose.position == Point(
        z=6.0 + math.copysign(0.5, vertical))
    assert board['verify_vertical_attempts'] == 1
    assert not any(event[0] in {'navigation', 'inspection_report'} for event in executive.events)


@pytest.mark.parametrize('phase', ['approach', 'pending_near'])
def test_exhausted_fire_leg_budget_cannot_start_fresh_edge_height_correction(system, phase):
    executive, controller = system
    controller.config['verify_fire_approach_max_steps'] = 3
    feed_fire(executive, controller, vertical_correction_rad=-0.4)
    board = (_confirmed_wall_board(executive, controller)
             if phase == 'pending_near' else Blackboard())
    board['verify_fire_approach_steps'] = 3
    executive._verify_opportunistic_wall = lambda *args: None

    with pytest.raises(RuntimeError, match='step limit.*inspection incomplete; holding'):
        if phase == 'pending_near':
            controller.center_fire(board)
        else:
            controller.approach_smoke(board)
    assert not any(event[0] in {'recovery_plan', 'navigation', 'inspection_report'}
                   for event in executive.events)
    assert executive.odometry.pose.pose.position == Point(z=6.0)


@pytest.mark.parametrize('direction', [-1.0, 1.0])
@pytest.mark.parametrize('available', ['combined', 'height', 'level'])
def test_symmetric_approach_tries_combined_then_one_height_then_forward(
        system, direction, available):
    executive, controller = system
    executive.wall_x = 50.0
    controller.config['verify_fire_approach_max_steps'] = 8
    original = executive._verify_recovery_plan_to
    attempts = []

    def plan(target, **kwargs):
        current = executive.odometry.pose.pose.position
        horizontal = math.hypot(target.position.x - current.x, target.position.y - current.y)
        height = target.position.z - current.z
        kind = 'combined' if horizontal > 0.1 and abs(height) > 0.1 else (
            'height' if abs(height) > 0.1 else 'level')
        attempts.append(kind)
        if kind == 'combined' and available != 'combined':
            return False
        if kind == 'height' and available == 'level':
            return False
        return original(target, **kwargs)

    executive._verify_recovery_plan_to = plan
    executive.on_wait = lambda: _qualified_fire(
        executive, controller, vertical_correction_rad=direction * 0.3)
    _qualified_fire(executive, controller, vertical_correction_rad=direction * 0.3)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert attempts == {'combined': ['combined'], 'height': ['combined', 'height'],
                        'level': ['combined', 'height', 'level']}[available]
    events = [event[0] for event in executive.events]
    assert events.index('wall_probe') < events.index('navigation')
    assert not any(event[0] == 'inspection_report' for event in executive.events)
    if available == 'height':
        assert executive.odometry.pose.pose.position == Point(z=6.0 + direction * 0.5)
        assert board['verify_height_only_attempted']
        _qualified_fire(executive, controller, vertical_correction_rad=direction * 0.3)
        assert controller.approach_smoke(board) == 'reacquire'
        assert attempts == ['combined', 'height', 'combined', 'level']
        assert not board['verify_height_only_attempted']
    else:
        assert executive.odometry.pose.pose.position.x > 0.3
        assert executive.odometry.pose.pose.position.z == pytest.approx(
            6.0 + direction * 0.5 if available == 'combined' else 6.0)


def test_forward_fallback_rechecks_fresh_guidance_after_failed_height_attempts(system):
    executive, controller = system
    executive.wall_x = 50.0
    controller.config['verify_fire_approach_max_steps'] = 8
    attempts = []

    def reject(target, **kwargs):
        attempts.append(deepcopy(target.position))
        if abs(target.position.x) < 0.1:
            fire(executive, controller, detected=False)
            smoke(executive, controller, detected=False)
        return False

    executive._verify_recovery_plan_to = reject
    _qualified_fire(executive, controller, vertical_correction_rad=-0.3)
    assert controller.approach_smoke(Blackboard()) == 'reacquire'
    assert len(attempts) == 2  # No new translation is selected from the old fire bearing.
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_forward_fallback_rechecks_a_new_near_wall_after_failed_height_attempts(system):
    executive, controller = system
    executive.wall_x = 50.0
    controller.config['verify_fire_approach_max_steps'] = 8
    attempts = []

    def reject(target, **kwargs):
        attempts.append(deepcopy(target.position))
        if abs(target.position.x) < 0.1:
            executive.wall_x = 10.4
            _qualified_fire(executive, controller, vertical_correction_rad=-0.3)
        return False

    executive._verify_recovery_plan_to = reject
    _qualified_fire(executive, controller, vertical_correction_rad=-0.3)
    board = Blackboard()
    assert controller.approach_smoke(board) == 'reacquire'
    assert len(attempts) == 2
    assert len([event for event in executive.events if event[0] == 'wall_probe']) == 2
    assert board['verify_wall_near_count'] == 1
    assert executive.odometry.pose.pose.position == Point(z=6.0)


def test_memory_waits_for_post_arrival_smoke_before_selecting_combined_leg(system, monkeypatch):
    executive, controller, board, _ = _slow_height_hint(system, monkeypatch, 60, angle=0.3)
    executive.wall_x = 50.0
    smoke(executive, controller)
    board['verify_arrival_after_ns'] = int(executive.now * 1e9) + 1
    monkeypatch.setattr(controller, '_smoke_guidance',
                        VerifyController._smoke_guidance.__get__(controller))
    executive.on_wait = lambda: smoke(executive, controller)
    assert controller.reacquire(board) == 'approach'
    assert not any(event[0] == 'recovery_plan' for event in executive.events)
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x > 3.3
    assert executive.odometry.pose.pose.position.z == pytest.approx(6.5)
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_default_inspection_budget_runs_past_five_minutes_and_expires_at_ten(
        system, monkeypatch):
    executive, controller = system
    wall_time = [1000.0]
    monkeypatch.setattr(verification.time, 'monotonic', lambda: wall_time[0])
    board = Blackboard()
    controller._inspection_begin(board)
    assert controller.config['verify_inspection_timeout'] == 600.0

    for elapsed in (300.0, 599.9):
        wall_time[0] = 1000.0 + elapsed
        controller._check_inspection_deadline(board)
        controller._inspection_begin(board)
        assert board['verify_inspection_started'] == 1000.0

    wall_time[0] = 1600.0
    with pytest.raises(RuntimeError, match='inspection time budget exhausted'):
        controller._check_inspection_deadline(board)
    assert not any(event[0] == 'inspection_report' for event in executive.events)


@pytest.mark.parametrize('distance', [15.0, 17.5, 20.0])
def test_inspection_band_accepts_two_fresh_qualified_ranges_without_advancing(system, distance):
    executive, controller = system
    for config in (executive.config, controller.config):
        config.update(verify_smoke_standoff=15.0, verify_smoke_inspection_max_distance=20.0)
    board = Blackboard()
    controller._record_wall_observation(
        board, _qualified_wall(executive, distance + 0.1, 101))
    assert controller._wall_action(board) == 'reacquire'
    controller._record_wall_observation(
        board, _qualified_wall(executive, distance + 0.1, 102))
    assert controller._wall_action(board) == 'ready'
    assert board['verify_wall_close_count'] == 0
    assert executive.odometry.pose.pose.position == Point(z=6.0)
    assert not any(event[0] in {'navigation', 'recovery_plan'} for event in executive.events)


def test_fresh_range_beyond_twenty_metres_releases_band_without_resetting_clock(system):
    executive, controller = system
    for config in (executive.config, controller.config):
        config.update(verify_smoke_standoff=15.0, verify_smoke_inspection_max_distance=20.0)
    board = Blackboard()
    controller._record_wall_observation(board, _qualified_wall(executive, 18.1, 101))
    controller._record_wall_observation(board, _qualified_wall(executive, 18.1, 102))
    started = board['verify_inspection_started']
    controller._record_wall_observation(board, _qualified_wall(executive, 20.2, 103))
    assert controller._wall_action(board) == 'approach'
    assert not board['verify_smoke_boundary_reached']
    assert board['verify_wall_close_count'] == 0
    assert board['verify_inspection_started'] == started
