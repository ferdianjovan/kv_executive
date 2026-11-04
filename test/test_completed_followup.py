"""Unresolved aligned smoke receives bounded safe views with measured lateral separation."""

from copy import deepcopy
import math

from geometry_msgs.msg import Quaternion, Transform, Vector3
from kv_executive.mission_executive import MissionCanceled, MissionError
from kv_executive.mission_geometry import transform_pose
from kv_executive.mission_navigation import _inverse
import pytest
from test_cluster_motion import region
from test_inspection_departure import Harness


class FollowupHarness(Harness):
    def __init__(self):
        super().__init__()
        self.current.position.x = 0.0
        self.regions = [region(x=85.0, radius=25.0)]
        self.task = {'key': (1, 0), 'site_id': 1, 'origin': (0.0, 0.0, 6.0),
                     'direction': (1.0, 0.0, 0.0), 'attempts': 0, 'observations': []}
        self.results = []

    def _completed_followup_task(self):
        return deepcopy(self.task)

    def _completed_followup_result(self, key, success, observation_origin=None):
        self.results.append((key, success, observation_origin))
        return success


def test_aligned_far_smoke_gets_a_settled_six_metre_lateral_view_without_ranging():
    node = FollowupHarness()
    assert node._verify_completed_followup()
    assert len(node.moves) == 2 and len(node.settles) == 1
    assert node.current.position.x == pytest.approx(0.0)
    assert node.current.position.y == pytest.approx(6.0)
    assert all(start.z == end.z for start, end in node.moves)
    assert node.turns[-1] == pytest.approx(0.0)
    assert node.settles[0]['wait_for_map'] is False
    assert node.results[0][:2] == ((1, 0), True)
    assert node.results[0][2] == pytest.approx((0.0, 6.0, 6.0))


def test_a_blocked_side_tries_the_opposite_side_in_the_same_bounded_attempt():
    node = FollowupHarness()
    node.allow = lambda target, current: target.position.y < current.position.y
    assert node._verify_completed_followup()
    assert node.current.position.y == pytest.approx(-6.0)
    assert len(node.results) == 1 and node.results[0][1] is True


def test_second_view_does_not_return_to_the_original_observer():
    node = FollowupHarness()
    node.current.position.y = 6.0
    node.task['attempts'] = 1
    node.task['observations'] = [(0.0, 6.0, 6.0)]
    assert node._verify_completed_followup()
    assert len(node.moves) == 4
    assert node.current.position.y == pytest.approx(-6.0)
    assert math.dist(node.task['observations'][0], node.results[0][2]) == pytest.approx(12.0)


def test_no_available_route_consumes_one_attempt_without_aborting_the_report():
    node = FollowupHarness()
    tried = []
    node.allow = lambda *args: tried.append(args) or False
    assert node._verify_completed_followup() is False
    assert len(tried) == 4 and not node.moves
    assert node.results == [((1, 0), False, None)]
    assert node.report.published


@pytest.mark.parametrize('return_value', [True, False, None])
def test_planner_return_without_actual_motion_cannot_claim_a_new_view(return_value):
    node = FollowupHarness()
    node._verify_departure_leg = lambda *_args: return_value
    assert node._verify_completed_followup() is False
    assert node.results == [((1, 0), False, None)]
    assert not node.turns and not node.settles


def test_short_partial_progress_does_not_satisfy_the_baseline():
    node = FollowupHarness()

    def partial(*_args):
        node.current.position.y += 0.5
        return False

    node._verify_departure_leg = partial
    assert node._verify_completed_followup() is False
    assert node.current.position.y == pytest.approx(2.0)
    assert not node.results[0][1]


def test_followup_has_a_caller_supplied_leg_budget():
    node = FollowupHarness()
    assert node._verify_completed_followup(maximum_legs=1) is False
    assert len(node.moves) == 1
    assert not node.results[0][1]


@pytest.mark.parametrize('failure', ['cancel', 'epoch'])
def test_interruptions_propagate_and_consume_only_one_attempt(failure):
    node = FollowupHarness()
    if failure == 'epoch':
        node.on_move = lambda: node.regions.clear()
        error, message = MissionError, 'changed epoch'
    else:
        def stop():
            if node.moves:
                raise MissionCanceled('operator canceled')
        node.on_check = stop
        error, message = MissionCanceled, 'operator canceled'
    with pytest.raises(error, match=message):
        node._verify_completed_followup()
    assert len(node.moves) == 1
    assert node.results == [((1, 0), False, None)]


def test_drift_during_settling_cannot_create_a_false_success():
    node = FollowupHarness()
    node.on_settle = lambda: setattr(node.current.position, 'y', 4.0)
    assert node._verify_completed_followup() is False
    assert not node.results[0][1]


def test_a_view_already_used_is_not_a_new_view():
    node = FollowupHarness()
    node.task['observations'] = [(0.0, 6.0, 6.0)]
    assert node._verify_completed_followup() is False
    assert not node.results[0][1]


def test_forward_drift_does_not_make_the_same_lateral_baseline_a_distinct_view():
    node = FollowupHarness()
    node.task['observations'] = [(8.0, 6.0, 6.0)]
    assert node._verify_completed_followup() is False
    assert node.current.position.y == pytest.approx(6.0)
    assert node.results == [((1, 0), False, None)]


def test_rejected_registry_grant_does_not_claim_success_or_consume_twice():
    node = FollowupHarness()

    def rejected(key, success, observation_origin=None):
        node.results.append((key, success, observation_origin))
        return False

    node._completed_followup_result = rejected
    assert node._verify_completed_followup() is False
    assert len(node.results) == 1 and node.results[0][1] is True
    assert not any('settled alternate viewpoint ready' in event for event in node.events)


def test_followup_does_not_cross_another_completed_cluster():
    node = FollowupHarness()
    node.regions.extend([region(x=0.0, y=3.0, radius=2.0, site_id=2),
                         region(x=0.0, y=-3.0, radius=2.0, site_id=3)])
    assert node._verify_completed_followup() is False
    assert not node.moves
    assert not node.results[0][1]


def test_tilted_local_frame_does_not_change_the_map_plane_baseline():
    node = FollowupHarness()
    node.conversion = Transform(translation=Vector3(x=100.0, y=-20.0, z=3.0),
                                rotation=Quaternion(y=math.sin(0.3), w=math.cos(0.3)))
    node.current = transform_pose(node.current, node.conversion)
    assert node._verify_completed_followup()
    actual = transform_pose(node.current, _inverse(node.conversion)).position
    assert actual.x == pytest.approx(0.0) and actual.y == pytest.approx(6.0)
    assert node.results[0][2] == pytest.approx((0.0, 6.0, 6.0))


def test_departure_runs_queued_followup_before_returning_to_exploration():
    node = FollowupHarness()
    node._verify_depart_completed_site(node.report)
    assert node.results[0][1]
    assert len(node.moves) == 2
    assert node.turns[-1] == pytest.approx(0.0)
    assert node.holds == [2.0]
    assert any('handoff complete' in event for event in node.events)


def test_no_safe_optional_onward_leg_still_preserves_successful_handoff():
    node = Harness()
    node.current.position.x = 27.0
    node.allow = lambda *_args: False
    node._verify_depart_completed_site(node.report)
    assert not node.moves and node.report.published
    assert any('no safe onward' in event for event in node.events)
    assert any('handoff complete' in event for event in node.events)


def test_followup_and_onward_progress_share_the_departure_movement_budget():
    node = FollowupHarness()
    node.config['verify_departure_max_legs'] = 1
    node._verify_depart_completed_site(node.report)
    assert len(node.moves) == 1
    assert not node.results[0][1]
    assert any('handoff complete' in event for event in node.events)


def test_onward_fallback_uses_actual_pose_after_a_partial_followup():
    node = FollowupHarness()
    received = []

    def partial_followup(**_kwargs):
        node.current.position.y = 2.0
        node._completed_followup_legs_used = 1
        return False

    def onward(current, position, conversion, guard):
        received.append((current.position.y, position.y))
        return False

    node._verify_completed_followup = partial_followup
    node._departure_onward_step = onward
    node._verify_depart_completed_site(node.report)
    assert received == [(2.0, 2.0)]
