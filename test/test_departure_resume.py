"""Airborne mission retries finish saved departure before entering Exploration."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
from kv_executive.inspection_departure import InspectionDepartureMixin
import kv_executive.mission_executive as executive_module
from kv_executive.mission_executive import MissionCanceled, MissionError, MissionExecutive
from kv_executive.mission_geometry import quaternion_from_yaw, transform_pose
import pytest
from test_cluster_motion import region
from test_mission_executive import Clock, Goal, Harness as MissionHarness, make_pose, run_goal


class Harness(MissionHarness, InspectionDepartureMixin):
    _resume_completed_departure = MissionExecutive._resume_completed_departure

    def __init__(self, clock):
        super().__init__(clock)
        self.config.update(verify_planning_frame='map', verify_departure_clearance=1.0)
        # The real wall is x=15; its shifted routing circle starts at that plane.
        self.regions = [region(x=40.0, radius=25.0)]
        self.conversion = Transform(rotation=Quaternion(w=1.0))
        self.odometry.pose.pose = make_pose(16, 0, 6)
        self._owns_flight = True
        self._goal_handle = Goal(skip_takeoff=True)
        self._site_sequence = 7
        self.reports = ['already published inspection']
        self.on_depart = self._finish_departure
        release = self._verify.release_inspection_target

        def record_release():
            self.events.append(('release_target',))
            release()

        self._verify.release_inspection_target = record_release

    def _completed_cluster_regions(self):
        return deepcopy(self.regions)

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def _verify_depart_completed_site(self, report, guard=None):
        self.events.append(('checked_departure', report.site_id, self._parent_state))
        guard()
        self.on_depart(report, guard)

    def _finish_departure(self, report, guard):
        guard()
        self.odometry.pose.pose = transform_pose(make_pose(0, 0, 6), self.conversion)

    def _verify_report_inspection(self, *args, **kwargs):
        pytest.fail('Resuming departure must not publish another inspection')

    def _verify_complete_site(self, *args, **kwargs):
        pytest.fail('Resuming departure must not register another completed site')


@pytest.fixture
def node(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(executive_module, 'time', SimpleNamespace(monotonic=clock.monotonic))
    monkeypatch.setattr(executive_module.rclpy, 'ok', lambda context: True)
    return Harness(clock)


def _assert_existing_finding_unchanged(node, original):
    assert node.regions == original
    assert node._site_sequence == 7
    assert node.reports == ['already published inspection']


def test_airborne_retry_admits_verify_phase_before_checked_escape_then_exploration(node):
    original = deepcopy(node.regions)
    node._resume_completed_departure()
    phases = [event[1].state for event in node.events if event[0] == 'phase']
    assert phases == ['Verify']
    departure_index = next(i for i, event in enumerate(node.events)
                           if event[0] == 'checked_departure')
    assert any(event[0] == 'phase' and event[1].state == 'Verify'
               for event in node.events[:departure_index])
    assert ('checked_departure', 1, 'Verify') in node.events
    node._verify_parent_entry('Exploration')
    assert [event[1].state for event in node.events if event[0] == 'phase'] == [
        'Verify', 'Exploration']
    _assert_existing_finding_unchanged(node, original)


@pytest.mark.parametrize('clearance', [1.0, 1.01, 15.0])
def test_retry_outside_full_exit_margin_requires_no_departure_or_phase_change(node, clearance):
    node.odometry.pose.pose.position.x = 15.0 - clearance
    node._resume_completed_departure()
    assert not any(event[0] in {'phase', 'checked_departure'} for event in node.events)
    assert node._parent_state is None


def test_retry_in_exit_margin_resumes_even_when_already_outside_the_circle(node):
    node.odometry.pose.pose.position.x = 14.5
    node._resume_completed_departure()
    assert ('checked_departure', 1, 'Verify') in node.events


@pytest.mark.parametrize('ready,armed', [(False, False), (False, True)])
def test_ground_or_not_airborne_admission_does_not_attempt_departure(node, ready, armed):
    node._airborne_ready = ready
    node.state.armed = armed

    def unused_snapshot(*args, **kwargs):
        pytest.fail('Not-airborne startup must keep the ordinary takeoff admission path')

    node._snapshot = unused_snapshot
    node._resume_completed_departure()
    assert not node.events


def test_disarmed_airborne_request_keeps_existing_armed_telemetry_requirement(node):
    node.state.armed = False
    with pytest.raises(MissionError, match='Vehicle disarmed'):
        node._resume_completed_departure()
    assert not any(event[0] in {'phase', 'checked_departure'} for event in node.events)


def test_multiple_regions_resume_the_zone_with_least_signed_clearance(node):
    node.regions += [region(x=16.0, radius=3.0, site_id=2),
                     region(x=50.0, radius=1.0, site_id=3)]
    node._resume_completed_departure()
    assert ('checked_departure', 2, 'Verify') in node.events


def test_retry_uses_map_plane_after_rotated_and_translated_local_alignment(node):
    angle = math.pi / 3.0
    node.conversion = Transform(
        translation=Vector3(x=100.0, y=-50.0, z=20.0),
        rotation=Quaternion(y=math.sin(angle / 2.0), w=math.cos(angle / 2.0)))
    node.odometry.pose.pose = transform_pose(
        Pose(position=Point(x=16.0, z=6.0), orientation=quaternion_from_yaw(0.0)),
        node.conversion)
    node._resume_completed_departure()
    assert ('checked_departure', 1, 'Verify') in node.events


@pytest.mark.parametrize('failure', ['cancellation', 'no_safe_route'])
def test_failed_or_canceled_resume_propagates_without_erasing_existing_registry(node, failure):
    original = deepcopy(node.regions)

    def fail_departure(report, guard):
        if failure == 'cancellation':
            node._goal_handle.is_cancel_requested = True
            guard()
        raise MissionError('DEPART_SITE: no safe outward route')

    node.on_depart = fail_departure
    expected = MissionCanceled if failure == 'cancellation' else MissionError
    with pytest.raises(expected):
        node._resume_completed_departure()
    _assert_existing_finding_unchanged(node, original)
    assert [event[1].state for event in node.events if event[0] == 'phase'] == ['Verify']


def test_retry_after_failed_departure_resumes_before_exploration(node):
    original = deepcopy(node.regions)

    def unavailable(report, guard):
        raise MissionError('DEPART_SITE: no safe outward route')

    node.on_depart = unavailable
    with pytest.raises(MissionError, match='no safe outward route'):
        node._resume_completed_departure()
    node.on_depart = node._finish_departure
    node._resume_completed_departure()
    node._verify_parent_entry('Exploration')
    assert [event[1].state for event in node.events if event[0] == 'phase'] == [
        'Verify', 'Verify', 'Exploration']
    assert len([event for event in node.events if event[0] == 'checked_departure']) == 2
    _assert_existing_finding_unchanged(node, original)


def test_actual_new_airborne_action_resumes_saved_departure_before_building_exploration(
        node, monkeypatch):
    original = deepcopy(node.regions)

    def build_after_departure(executive, skip_takeoff=False):
        assert executive is node and skip_takeoff
        assert ('checked_departure', 1, 'Verify') in node.events
        node.events.append(('build_exploration',))

        def explore(blackboard):
            node._verify_parent_entry('Exploration')
            return 'succeeded'

        return explore

    monkeypatch.setattr(executive_module, 'build_state_machine', build_after_departure)
    goal = Goal(skip_takeoff=True)
    result = run_goal(node, goal)
    assert result.success and goal.outcome == 'succeeded'
    assert [event[1].state for event in node.events if event[0] == 'phase'] == [
        'Verify', 'Exploration']
    assert node._goal_handle is None and not node._mission_lock.locked()
    _assert_existing_finding_unchanged(node, original)
