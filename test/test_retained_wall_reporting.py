"""Retain confirmed map hits without another wall probe for report completion."""

from copy import deepcopy
import math

from geometry_msgs.msg import Point, PointStamped
import pytest
from std_msgs.msg import Header
from test_verification import (
    _qualified_wall, feed_fire, smoke, system as system_fixture,
)
from yasmin import Blackboard


system = system_fixture


def wall_hit(executive, distance=10.4, stamp=None):
    stamp = round(executive.now * 1e9) if stamp is None else stamp
    wall = _qualified_wall(executive, distance, stamp)
    point = PointStamped(header=Header(frame_id='map'),
                         point=Point(x=distance, y=1.0, z=6.0))
    point.header.stamp.sec, point.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    wall.update(nearest_point_valid=True, nearest_point=point)
    return wall


def confirmed(system):
    executive, controller = system
    board = Blackboard()
    controller.config['verify_planning_frame'] = 'map'
    stamp = round(executive.now * 1e9)
    controller._record_wall_observation(board, wall_hit(executive, stamp=stamp - 1))
    controller._record_wall_observation(board, wall_hit(executive, stamp=stamp))
    original_prepare = executive._verify_prepare_wall_observation

    def prepare(guard=None, *, align_fire=True, wait_for_map=True):
        executive.events.append(('prepare', wait_for_map))
        return original_prepare(guard=guard, align_fire=align_fire)

    executive._verify_prepare_wall_observation = prepare
    return board


def no_wall_rpc(*args, **kwargs):
    raise AssertionError('Confirmed report preparation must not range the wall again')


def test_two_distinct_near_maps_capture_an_immutable_actual_hit(system):
    executive, controller = system
    board = Blackboard()
    first = wall_hit(executive, stamp=99_000_000_000)
    second = wall_hit(executive, stamp=100_000_000_000)
    controller._record_wall_observation(board, first)
    assert controller._report_wall(board) is None
    controller._record_wall_observation(board, second)
    retained = controller._report_wall(board)
    assert retained['confirmed']
    assert retained['confirmation_stamps'] == (99_000_000_000, 100_000_000_000)
    assert retained['nearest_point'] == second['nearest_point']
    assert executive._verify_retained_report_wall == retained
    second['nearest_point'].point.x = 999.0
    assert retained['nearest_point'].point.x == 10.4


def test_moving_drone_does_not_move_retained_smoke_or_add_old_range_to_new_pose(system):
    executive, controller = system
    board = confirmed(system)
    retained = deepcopy(controller._report_wall(board))
    executive.odometry.pose.pose.position = Point(x=5.0, y=2.0, z=8.0)
    executive._verify_opportunistic_wall = no_wall_rpc
    assert controller._probe_smoke_wall(board, object(), 0.0) == retained
    assert controller._wall_action(board) == 'ready'
    assert controller._report_wall(board)['nearest_point'].point == Point(x=10.4, y=1.0, z=6.0)


def test_missing_ranges_preserve_confirmed_point_and_do_not_start_a_new_wall_timer(system):
    executive, controller = system
    board = confirmed(system)
    retained = deepcopy(controller._report_wall(board))
    for _ in range(4):
        controller._record_wall_observation(board, None)
        assert controller._wall_action(board) == 'ready'
    assert controller._report_wall(board) == retained
    assert board['verify_wall_near_count'] == 2
    assert board['verify_wall_recheck_deadline'] is None


def test_unready_context_keeps_point_but_still_uses_bounded_physical_hold(system):
    executive, controller = system
    board = confirmed(system)
    retained = deepcopy(controller._report_wall(board))
    executive._verify_wall_observation_ready = lambda: False
    for count in (1, 2):
        assert controller._wall_action(board) == 'reacquire'
        assert board['verify_settle_failures'] == count
    with pytest.raises(RuntimeError, match='stationary observation preparation exhausted'):
        controller._wall_action(board)
    assert controller._report_wall(board) == retained
    assert not any(event[0] in {'navigation', 'plan', 'inspection_report'}
                   for event in executive.events)


def test_replayed_or_reordered_maps_cannot_manufacture_report_confirmation(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, wall_hit(executive, stamp=100))
    controller._record_wall_observation(board, wall_hit(executive, stamp=100))
    controller._record_wall_observation(board, wall_hit(executive, stamp=99))
    assert controller._report_wall(board) is None
    assert board['verify_wall_near_count'] == 1


@pytest.mark.parametrize('invalid', ['missing', 'frame', 'nan', 'stamp'])
def test_coordinate_is_never_fabricated_from_legacy_range_statistics(system, invalid):
    executive, controller = system
    board = Blackboard()
    wall = wall_hit(executive, stamp=100)
    if invalid == 'missing':
        wall['nearest_point_valid'] = False
    elif invalid == 'frame':
        wall['nearest_point'].header.frame_id = 'unknown_frame'
    elif invalid == 'nan':
        wall['nearest_point'].point.z = math.nan
    else:
        wall['nearest_point'].header.stamp.sec = 0
        wall['nearest_point'].header.stamp.nanosec = 0
    controller._record_wall_observation(board, wall_hit(executive, stamp=99))
    controller._record_wall_observation(board, wall)
    assert controller._report_wall(board) is None


@pytest.mark.parametrize('distance', [9.5, 30.0])
def test_a_new_explicit_close_or_far_measurement_releases_historical_arrival(system, distance):
    executive, controller = system
    board = confirmed(system)
    controller._record_wall_observation(
        board, wall_hit(executive, distance, round(executive.now * 1e9) + 1))
    assert controller._report_wall(board) is None
    assert executive._verify_retained_report_wall is None
    if distance > controller.config['verify_smoke_inspection_max_distance']:
        assert controller._wall_action(board) == 'approach'
    else:
        assert controller._wall_action(board) == 'reacquire'
        assert not any(event[0] == 'recovery_plan' for event in executive.events)


def test_confirmed_close_wall_does_not_cache_arrival_before_a_safe_retreat(system):
    executive, controller = system
    board = Blackboard()
    controller._record_wall_observation(board, wall_hit(executive, 9.5, 99))
    controller._record_wall_observation(board, wall_hit(executive, 9.4, 100))
    assert controller._report_wall(board) is None
    assert board['verify_wall_close_count'] == 2


def test_report_preparation_skips_optional_map_wait_and_preserves_inspection_clock(system):
    executive, controller = system
    board = confirmed(system)
    started = board['verify_inspection_started']
    assert controller._prepare_wall_observation(board)
    assert ('prepare', False) in executive.events
    assert board['verify_inspection_started'] == started


def test_positive_completion_uses_historical_arrival_with_fresh_fire_without_wall_rpc(system):
    executive, controller = system
    board = confirmed(system)
    executive._verify_center_fire_wall = executive._verify_opportunistic_wall = no_wall_rpc
    feed_fire(executive, controller)
    assert controller._approach_visible_fire(board, controller.stable_fire()) is not None
    assert not any(event[0] in {'navigation', 'plan', 'recovery_plan'}
                   for event in executive.events)


def test_negative_inspection_keeps_three_views_without_repeating_wall_probes(system):
    executive, controller = system
    board = confirmed(system)
    executive._verify_center_fire_wall = executive._verify_opportunistic_wall = no_wall_rpc
    executive.on_wait = lambda: smoke(executive, controller)
    smoke(executive, controller)
    assert controller.center_fire(board) == 'depart'
    report = next(event for event in executive.events if event[0] == 'inspection_report')
    assert report[-1] == 3
    assert report[2] is None
