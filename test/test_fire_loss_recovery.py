"""Keep fire-loss recovery independent of movement failures and inspection safety."""

from copy import deepcopy

from geometry_msgs.msg import Point
import kv_executive.verification as verification
import pytest
from test_verification import (
    _confirmed_wall_board, _enable_wall_probes, _qualified_fire,
    feed_smoke, fire, header, smoke, system as system_fixture,
)
from yasmin import Blackboard


system = system_fixture


def _pending_fire_loss(system, count=2):
    executive, controller = system
    board = Blackboard()
    board['verify_search_failures'] = 2
    for _ in range(count):
        assert controller._search_blocked(board, 'fire missing', fire_loss=True) == 'reacquire'
    assert board['verify_fire_loss_failures'] == count
    assert board['verify_search_failures'] == 2
    return board


def test_recovered_fire_resets_only_fire_loss_and_next_loss_starts_at_one(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    _qualified_fire(executive, controller)
    assert controller._inspection_fire(board) is not None
    assert board['verify_fire_loss_failures'] == 0
    assert board['verify_search_failures'] == 2
    fire(executive, controller, detected=False)
    assert controller._search_blocked(board, 'fire lost again', fire_loss=True) == 'reacquire'
    assert board['verify_fire_loss_failures'] == 1
    assert board['verify_search_failures'] == 2


@pytest.mark.parametrize('operation', ['center_fire', 'reacquire'])
def test_real_missing_fire_state_uses_only_fire_loss_budget(system, operation):
    executive, controller = system
    _qualified_fire(executive, controller)
    board = _confirmed_wall_board(executive, controller)
    controller._route_fire(board)
    board['verify_search_failures'] = 2
    fire(executive, controller, detected=False)
    if operation == 'center_fire':
        _enable_wall_probes(executive)
        feed_smoke(executive, controller)
        executive.on_wait = lambda: smoke(executive, controller)
    assert getattr(controller, operation)(board) == 'reacquire'
    assert board['verify_fire_loss_failures'] == 1
    assert board['verify_search_failures'] == 2
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_uninterrupted_fire_loss_still_exhausts_three_attempts(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    with pytest.raises(RuntimeError, match='Verify blocked: fire missing'):
        controller._search_blocked(board, 'fire missing', fire_loss=True)
    assert board['verify_fire_loss_failures'] == 3
    assert board['verify_search_failures'] == 2
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_third_movement_failure_still_aborts_despite_valid_recovered_fire(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    _qualified_fire(executive, controller)
    assert controller._inspection_fire(board) is not None
    with pytest.raises(RuntimeError, match='Verify blocked: terrain path blocked'):
        controller._search_blocked(board, 'terrain path blocked')
    assert board['verify_search_failures'] == 3
    assert board['verify_fire_loss_failures'] == 0


def test_continuously_visible_fire_cannot_reset_repeated_blocked_planning(system):
    executive, controller = system
    executive.wall_x = 30.0
    controller.config['verify_fire_approach_max_steps'] = 8
    executive._verify_navigation_viewpoint = lambda *args, **kwargs: False
    board = Blackboard()
    for expected in (1, 2):
        _qualified_fire(executive, controller)
        assert controller.approach_smoke(board) == 'reacquire'
        assert board['verify_search_failures'] == expected
    _qualified_fire(executive, controller)
    with pytest.raises(RuntimeError, match='Verify blocked'):
        controller.approach_smoke(board)
    assert board['verify_search_failures'] == 3
    assert verification._bb_get(board, 'verify_fire_loss_failures', 0) == 0
    assert executive.odometry.pose.pose.position == Point(z=6.0)


@pytest.mark.parametrize('invalid', [
    'stale', 'unconfirmed', 'not_persistent', 'pre_loss', 'pre_arrival',
    'pre_inspection', 'pre_smoke_arrival', 'pre_settle', 'unsettled',
])
def test_unsuitable_fire_cannot_clear_fire_loss_budget(system, invalid):
    executive, controller = system
    board = _pending_fire_loss(system)
    changes = {'status': 1} if invalid == 'unconfirmed' else {}
    if invalid == 'not_persistent':
        fire(executive, controller)
    else:
        _qualified_fire(executive, controller, **changes)
    stamp = controller._stamp_ns(controller.fresh_fire()) if controller.fresh_fire() else 0
    if invalid == 'stale':
        executive.now += controller.config['verify_observation_timeout'] + 0.1
    elif invalid == 'pre_loss':
        board['verify_fire_loss_after_ns'] = stamp
    elif invalid == 'pre_arrival':
        board['verify_arrival_after_ns'] = stamp
    elif invalid == 'pre_inspection':
        board['verify_inspection_after_ns'] = stamp
    elif invalid == 'pre_smoke_arrival':
        board['verify_smoke_approach_after_ns'] = stamp
    elif invalid == 'pre_settle':
        executive._verify_wall_observation_context = {'settled_ns': stamp}
    elif invalid == 'unsettled':
        executive._verify_wall_observation_ready = lambda: False
    controller._inspection_fire(board)
    assert board['verify_fire_loss_failures'] == 2
    assert board['verify_search_failures'] == 2
    assert not any('fire-loss recovery count reset' in event[1]
                   for event in executive.events if event[0] == 'status')


def test_cached_pre_loss_future_frame_cannot_replenish_retries(system):
    executive, controller = system
    controller.config['verify_navigation_map_wait'] = 0.05
    image = _qualified_fire(executive, controller)
    image.header = header(executive.now + 0.2, 'thermal')
    controller.fire_callback(image)
    board = Blackboard()
    executive._verify_wall_observation_ready = lambda: False
    assert controller._search_blocked(board, 'unusable viewpoint', fire_loss=True) == 'reacquire'
    assert board['verify_fire_loss_after_ns'] >= controller._stamp_ns(image)
    executive._verify_wall_observation_ready = lambda: True
    controller.fire_callback(image)  # A repeated source image cannot prove recovery.
    controller._inspection_fire(board)
    assert board['verify_fire_loss_failures'] == 1
    assert controller.stable_fire() is not None  # It is fresh, but predates the loss boundary.
    executive.now = controller._stamp_ns(image) / 1e9 + 0.01
    _qualified_fire(executive, controller)
    assert controller._inspection_fire(board) is not None
    assert board['verify_fire_loss_failures'] == 0


def test_reading_same_recovery_image_resets_once_without_touching_movement_budget(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    _qualified_fire(executive, controller)
    for _ in range(4):
        assert controller._inspection_fire(board) is not None
    resets = [event for event in executive.events
              if event[0] == 'status' and 'fire-loss recovery count reset' in event[1]]
    assert len(resets) == 1
    assert board['verify_search_failures'] == 2


def test_remembered_fire_routing_is_not_a_recovery(system):
    executive, controller = system
    _qualified_fire(executive, controller)
    fire(executive, controller, detected=False)
    board = _pending_fire_loss(system)
    assert controller._route_fire(board) == 'fire'
    assert board['verify_fire_loss_failures'] == 2
    assert board['verify_search_failures'] == 2


@pytest.mark.parametrize('reader', ['inspection', 'approach', 'reacquire', 'observation'])
def test_qualified_recovery_does_not_require_a_wall_measurement(system, reader):
    executive, controller = system
    board = _pending_fire_loss(system)
    executive._verify_center_fire_wall = lambda frame: None
    executive._verify_opportunistic_wall = lambda *args: None
    _qualified_fire(executive, controller)
    if reader == 'inspection':
        assert controller._inspection_fire(board) is not None
    elif reader == 'approach':
        assert controller._approach_observation(board)[1] == 'fire'
    elif reader == 'reacquire':
        assert controller._reacquire_fire(board) is not None
    else:
        assert controller._inspection_observation(board)[1] == 'fire'
    assert board['verify_fire_loss_failures'] == 0
    assert board['verify_search_failures'] == 2
    assert not any(event[0] == 'inspection_report' for event in executive.events)


def test_successful_movement_without_fire_does_not_clear_fire_loss(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    smoke(executive, controller)
    assert controller.approach_smoke(board) == 'reacquire'
    assert executive.odometry.pose.pose.position.x > 0.0
    assert board['verify_search_failures'] == 0
    assert board['verify_fire_loss_failures'] == 2


def test_fire_recovery_cannot_complete_inspection_without_current_wall_evidence(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    executive._verify_opportunistic_wall = lambda *args: None
    executive._verify_center_fire_wall = lambda frame: None
    _qualified_fire(executive, controller)
    assert controller.center_fire(board) == 'approach'
    assert board['verify_fire_loss_failures'] == 0
    assert board['verify_search_failures'] == 2
    assert not any(event[0] in {'inspection_report', 'record', 'completed_site'}
                   for event in executive.events)


def test_recovery_preserves_wall_map_settling_and_inspection_budgets(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    board['verify_inspection_started'] = executive.now - 30.0
    board['verify_wall_recheck_deadline'] = executive.now + 12.0
    board['verify_wall_near_count'] = 1
    board['verify_wall_last_stamp_ns'] = 77
    board['verify_navigation_refresh_deadline'] = executive.now + 8.0
    board['verify_navigation_start_refreshes'] = 2
    board['verify_settle_failures'] = 2
    board['verify_navigation_stalls'] = 1
    keys = (
        'verify_inspection_started', 'verify_wall_recheck_deadline', 'verify_wall_near_count',
        'verify_wall_last_stamp_ns', 'verify_navigation_refresh_deadline',
        'verify_navigation_start_refreshes', 'verify_settle_failures', 'verify_navigation_stalls',
    )
    original = {key: deepcopy(board[key]) for key in keys}
    _qualified_fire(executive, controller)
    assert controller._inspection_fire(board) is not None
    assert all(board[key] == value for key, value in original.items())
    executive.now = board['verify_wall_recheck_deadline']
    with pytest.raises(RuntimeError, match='near-wall revalidation exhausted'):
        controller._check_wall_recheck_deadline(board)


def test_repeated_genuine_fire_recoveries_do_not_restart_the_inspection_clock(system):
    executive, controller = system
    board = Blackboard()
    controller._inspection_begin(board)
    started = board['verify_inspection_started']
    for _ in range(4):
        fire(executive, controller, detected=False)
        assert controller._search_blocked(board, 'fire missing', fire_loss=True) == 'reacquire'
        _qualified_fire(executive, controller)
        assert controller._inspection_fire(board) is not None
        assert board['verify_fire_loss_failures'] == 0
        assert board['verify_inspection_started'] == started
    executive.now = started + controller.config['verify_inspection_timeout']
    with pytest.raises(RuntimeError, match='inspection time budget exhausted'):
        controller._check_inspection_deadline(board)


def test_fire_arriving_at_last_retry_recovers_before_exhaustion_decision(system):
    executive, controller = system
    board = _pending_fire_loss(system)
    _qualified_fire(executive, controller)
    assert controller._search_blocked(
        board, 'snapshot missed new fire', fire_loss=True) == 'reacquire'
    assert board['verify_fire_loss_failures'] == 0
    assert board['verify_search_failures'] == 2
