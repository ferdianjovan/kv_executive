"""A published finding survives safe departure, alternate routes and bounded failures."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
import kv_executive.inspection_departure as departure
from kv_executive.inspection_departure import DEPARTURE_DEFAULTS, InspectionDepartureMixin
from kv_executive.mission_executive import MissionCanceled, MissionError
from kv_executive.mission_geometry import quaternion_from_yaw, transform_pose
from kv_executive.mission_navigation import _inverse
import pytest
from test_cluster_motion import region


class Harness(InspectionDepartureMixin):
    def __init__(self, fire=True):
        self.config = {**DEPARTURE_DEFAULTS, 'verify_planning_frame': 'map',
                       'local_frame': 'px4_odom'}
        self.conversion = Transform(rotation=Quaternion(w=1.0))
        self.current = Pose(position=Point(x=15.0, z=6.0),
                            orientation=quaternion_from_yaw(math.pi))
        self.report = SimpleNamespace(site_id=1, fire_found=fire, published=True)
        self.regions = [region(radius=25.0)]
        self.events = []
        self.moves = []
        self.turns = []
        self.settles = []
        self.settle = True
        self.allow = lambda *_: True
        self.on_check = lambda: None
        self.on_move = lambda: None
        self.candidate = None
        self._verify = SimpleNamespace(stable_fire=lambda: None,
                                       fresh_smoke=lambda: self.candidate)

    def _completed_cluster_regions(self):
        return deepcopy(self.regions)

    def _check_running(self):
        self.on_check()

    def _verify_control_pose(self):
        return None, deepcopy(self.current)

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def _report(self, text):
        self.events.append(text)

    def _verify_rotate(self, bearing, guard):
        guard()
        self.turns.append(bearing)

    def _verify_image_bearing(self, candidate):
        assert candidate is self.candidate
        return candidate.bearing

    def _verify_prepare_wall_observation(self, **options):
        options['guard']()
        self.settles.append(options)
        return self.settle

    def _verify_departure_leg(self, target, current, guard):
        guard()
        if not self.allow(target, current):
            return False
        self.moves.append((deepcopy(current.position), deepcopy(target.position)))
        self.current = deepcopy(target)
        self.on_move()
        return True


@pytest.mark.parametrize('fire', [True, False])
def test_completed_positive_and_negative_findings_depart_outward_before_exploration(fire):
    node = Harness(fire)
    node._verify_depart_completed_site(node.report)
    assert node.report.published and node.report.fire_found is fire
    assert len(node.moves) == 4
    assert node.current.position.x == pytest.approx(27.0)
    assert all(end.x > start.x and end.z == start.z for start, end in node.moves)
    assert node.turns[-1] == pytest.approx(0.0)
    assert node.settles[-1]['align_fire'] is False
    assert node.settles[-1]['wait_for_map'] is False
    assert any('handoff complete' in message for message in node.events)
    assert len(node.regions) == 1


def test_blocked_outward_leg_tries_a_safe_side_instead_of_climbing_over_cluster():
    node = Harness()
    node.allow = lambda target, current: target.position.y > current.position.y + 0.1
    node._verify_depart_completed_site(node.report)
    assert node.current.position.y > 0.0
    assert all(end.z == start.z for start, end in node.moves)
    assert math.hypot(node.current.position.x, node.current.position.y) >= 26.0


@pytest.mark.parametrize('reason', ['route', 'movement_budget', 'settling'])
def test_departure_failure_keeps_finding_and_cluster_without_claiming_handoff(reason):
    node = Harness()
    if reason == 'route':
        node.allow = lambda *_: False
    elif reason == 'movement_budget':
        node.config['verify_departure_max_legs'] = 1
    else:
        node.settle = False
    with pytest.raises(MissionError, match='DEPART_SITE:'):
        node._verify_depart_completed_site(node.report)
    assert node.report.published and node.regions
    assert not any('handoff complete' in message for message in node.events)


def test_cancel_is_checked_before_any_departure_motion():
    node = Harness()

    def cancel():
        raise MissionCanceled('user canceled')

    with pytest.raises(MissionCanceled, match='user canceled'):
        node._verify_depart_completed_site(node.report, guard=cancel)
    assert not node.moves and node.report.published


def test_departure_has_its_own_wall_deadline(monkeypatch):
    node = Harness()
    now = [0.0]
    monkeypatch.setattr(departure.time, 'monotonic', lambda: now[0])
    node.config['verify_departure_timeout'] = 1.0
    node.on_check = lambda: now.__setitem__(0, now[0] + 0.6)
    with pytest.raises(MissionError, match='departure time budget exhausted'):
        node._verify_depart_completed_site(node.report)
    assert node.report.published
    assert not any('handoff complete' in message for message in node.events)


def test_epoch_reset_during_departure_does_not_follow_a_stale_cluster():
    node = Harness()
    node.on_move = lambda: node.regions.clear()
    with pytest.raises(MissionError, match='registry changed epoch'):
        node._verify_depart_completed_site(node.report)
    assert len(node.moves) == 1 and node.report.published


def test_handoff_can_face_a_distinct_eligible_candidate_after_exit():
    node = Harness()
    node.candidate = SimpleNamespace(bearing=math.pi / 2)
    node._verify_depart_completed_site(node.report)
    assert node.turns[-1] == pytest.approx(math.pi / 2)


def test_map_plane_controls_departure_even_with_tilted_translated_local_frame():
    node = Harness()
    node.conversion = Transform(translation=Vector3(x=100.0, y=-20.0, z=3.0),
                                rotation=Quaternion(y=math.sin(0.3), w=math.cos(0.3)))
    node.current = transform_pose(node.current, node.conversion)
    node._verify_depart_completed_site(node.report)
    actual_map = transform_pose(node.current, _inverse(node.conversion)).position
    assert math.hypot(actual_map.x, actual_map.y) >= 26.0
    assert node.moves  # Local X alone was already >100 m and cannot establish departure.
