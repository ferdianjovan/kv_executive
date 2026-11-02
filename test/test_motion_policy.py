"""Whole-route limits distinguish ordinary pursuit from measured safety recovery."""

from copy import deepcopy
import math

from geometry_msgs.msg import Point, Pose, PoseStamped
from kv_executive.mission_geometry import quaternion_from_yaw
from nav_msgs.msg import Path
import pytest

from test_inspection_navigation import Harness


@pytest.fixture
def executive():
    node = Harness()
    node.config['verify_inspection_altitude'] = 3.0
    node.odometry.pose.pose.position = Point(z=6.0)
    return node


def route(*points):
    return Path(poses=[PoseStamped(pose=Pose(
        position=Point(x=float(x), y=float(y), z=float(z)),
        orientation=quaternion_from_yaw(0.0))) for x, y, z in points])


@pytest.mark.parametrize('points', [
    [(0, 0, 6), (-1, 0, 6), (3, 0, 6)],
    [(0, 0, 6), (2, 0, 6), (1, 0, 6), (3, 0, 6)],
    [(0, 0, 6), (0, 0, 16), (3, 0, 6)],
    [(0, 0, 6), (0, 0, 4), (3, 0, 6)],
    [(0, 0, 6), (1, 8, 6), (3, 0, 6)],
])
def test_positive_forward_endpoint_does_not_hide_unwanted_intermediate_motion(executive, points):
    assert not executive._verify_path_motion_policy(route(*points), Point(z=6.0), 0.0, {})
    assert any('rejected' in event[1] and 'ordinary approach' in event[1]
               for event in executive.events if event[0] == 'status')


@pytest.mark.parametrize('height', [-0.5, 0.0, 0.5])
@pytest.mark.parametrize('side', [-1.5, 1.5])
def test_bounded_side_and_height_routes_remain_available(executive, height, side):
    assert executive._verify_path_motion_policy(
        route((0, 0, 6), (1, side / 2, 6 + height / 2), (3, side, 6 + height)),
        Point(z=6.0), 0.0, {})


def test_actual_backtracking_cannot_reset_at_each_inflight_check(executive):
    state = {}
    assert executive._verify_path_motion_policy(
        route((2, 0, 6), (3, 0, 6)), Point(z=6.0), 0.0, state)
    assert not executive._verify_path_motion_policy(
        route((1, 0, 6), (3, 0, 6)), Point(z=6.0), 0.0, state)


def test_small_tracking_deviations_are_not_deliberate_retreats(executive):
    assert executive._verify_path_motion_policy(
        route((0, 0, 6), (-0.1, 0, 6), (3, 0, 6)), Point(z=6.0), 0.0, {})


def test_motion_direction_is_relative_to_the_selected_source_bearing(executive):
    assert executive._verify_path_motion_policy(
        route((0, 0, 6), (0, 3, 6)), Point(z=6.0), math.pi / 2, {})
    assert not executive._verify_path_motion_policy(
        route((0, 0, 6), (0, -1, 6), (0, 3, 6)), Point(z=6.0), math.pi / 2, {})


def test_optional_high_endpoint_is_rejected_before_requesting_a_native_plan(executive):
    executive.ground_z = 2.0
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = 3.0
    target.position.z += 10.5
    assert not executive._verify_recovery_plan_to(target)
    assert not any(event[0] == 'inspection_plan' for event in executive.events)


def test_fresh_below_floor_clearance_allows_a_bounded_safety_escape(executive):
    executive.ground_z = 4.5
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = -1.0
    target.position.z = 10.0
    assert executive._verify_recovery_plan_to(target)
    assert any('current terrain clearance' in event[1] and 'below' in event[1]
               for event in executive.events if event[0] == 'status')


def test_unproven_recovery_label_cannot_authorize_an_ordinary_backward_leg(executive):
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = -2.0
    assert not executive._verify_recovery_plan_to(
        target, geometry={'safety_recovery_reason': 'confirmed_smoke_wall_inside_standoff'})
    assert not any(event[0] == 'inspection_plan' for event in executive.events)


def test_confirmed_close_wall_allows_bounded_retreat_with_full_corridor_checks(executive):
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = -3.0
    wall = {'origin': Point(z=6.0), 'axis': (1.0, 0.0, 0.0),
            'nearest': 8.0, 'uncertainty': 0.1}
    geometry = {'smoke_navigation_walls': [wall], 'confirmed_wall_retreat': True,
                'safety_recovery_reason': 'confirmed_smoke_wall_inside_standoff'}
    assert executive._verify_recovery_plan_to(
        target, geometry=geometry, preserve_requested_altitude=True)
    assert executive.odometry.pose.pose.position == Point(x=-3.0, z=6.0)
    assert any(event[0] == 'inspection_plan' for event in executive.events)
    assert any('reason=confirmed smoke wall' in event[1]
               for event in executive.events if event[0] == 'status')


def test_safety_escape_still_requires_full_collision_checks(executive):
    executive.ground_z = 4.5
    executive.corridor_safe = False
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = -1.0
    target.position.z = 10.0
    assert not executive._verify_recovery_plan_to(target)
    assert executive.odometry.pose.pose.position == Point(z=6.0)


def test_stale_or_displaced_clearance_proof_is_not_reused(executive):
    executive._verify_current_terrain = {
        'position': Point(x=100.0, z=6.0),
        'stamp_ns': int(executive.now * 1e9), 'terrain': (5.0, 0.1),
    }
    executive.ground_z = 2.0
    assert not executive._verify_current_terrain_hazard(executive.odometry.pose.pose)
