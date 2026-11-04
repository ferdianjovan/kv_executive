"""Swept completed-cluster geometry and native departure-request contracts."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion, Transform
from kv_executive.inspection_departure import cluster_path_safe, InspectionDepartureMixin
from kv_executive.inspection_navigation import InspectionNavigationMixin
from kv_executive.mission_executive import MissionError, MissionExecutive
from kv_executive.mission_geometry import quaternion_from_yaw, transform_pose
from kv_executive.mission_navigation import _inverse
from kv_executive.srv import PlanInspectionPath
from nav_msgs.msg import Path
import pytest


def region(x=0.0, y=0.0, radius=1.0, site_id=1):
    point = PointStamped()
    point.header.frame_id = 'map'
    point.point = Point(x=float(x), y=float(y))
    return {'site_id': site_id, 'center': point, 'radius': radius}


def path(*points):
    result = Path()
    result.header.frame_id = 'px4_odom'
    result.poses = [PoseStamped(pose=Pose(position=Point(
        x=float(x), y=float(y), z=float(z)), orientation=quaternion_from_yaw(0.0)))
        for x, y, z in points]
    return result


@pytest.mark.parametrize('vertices,safe', [
    ([(-2, 0, 2), (2, 0, 2)], False),
    ([(-2, 0, 20), (2, 0, 20)], False),
    ([(-2, 1, 2), (2, 1, 2)], True),
    ([(-2, 0.999, 2), (2, 0.999, 2)], False),
    ([(-2, 0, 2), (-2, 1.5, 2), (2, 1.5, 2), (2, 0, 2)], True),
    ([(2, 0, 2), (0, 0, 20), (-2, 0, 2)], False),
])
def test_entire_swept_footprint_controls_clearance_at_every_altitude(vertices, safe):
    assert cluster_path_safe([p.pose.position for p in path(*vertices).poses],
                             [region()]) is safe


@pytest.mark.parametrize('vertices,safe', [
    ([(0.5, 0, 2), (1.5, 0, 2)], True),
    ([(0.5, 0, 2), (0.4, 0, 2)], False),
    ([(0.5, 0, 2), (0.5, 0.5, 2)], True),
    ([(0.5, 0, 2), (0.8, 0, 2), (0.7, 0, 2)], False),
    ([(0.5, 0, 2), (1.5, 0, 2), (0.7, 0, 2)], False),
    ([(0.8, 0, 2), (0.7, 0, 2)], False),
])
def test_departure_uses_fixed_anchor_and_never_reverses_inside_or_reenters(vertices, safe):
    assert cluster_path_safe([p.pose.position for p in path(*vertices).poses],
                             [region()], Point(x=0.5, z=2.0)) is safe


def test_escape_from_one_region_never_waives_another_completed_region():
    regions = [region(x=-0.5), region(x=0.5, site_id=2)]
    outward = path((0, 0, 2), (0, 1.5, 2))
    through_neighbor = path((0, 0, 2), (2, 0, 2))
    assert cluster_path_safe([p.pose.position for p in outward.poses], regions, Point(z=2.0))
    assert not cluster_path_safe([p.pose.position for p in through_neighbor.poses],
                                 regions, Point(z=2.0))


class Harness(InspectionDepartureMixin, InspectionNavigationMixin):
    def __init__(self):
        self.config = {
            'local_frame': 'px4_odom', 'verify_planning_frame': 'map',
            'verify_inspection_altitude': 3.0, 'verify_navigation_max_agl': 10.0,
            'position_tolerance': 0.3, 'verify_departure_step': 3.0,
            'verify_departure_timeout': 180.0,
        }
        self.conversion = Transform(rotation=Quaternion(w=1.0))
        self.regions = []
        self.queries = []
        self.plans = []
        self.current = Pose(position=Point(x=0.5, z=4.0),
                            orientation=quaternion_from_yaw(0.0))
        self._inspection_planning_client = object()

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def _completed_cluster_regions(self):
        return deepcopy(self.regions)

    def _inspection_query(self, request, require_map=False):
        assert require_map
        self.queries.append(deepcopy(request))
        return SimpleNamespace(corridor_safe=True, current_clearance_reached=False)

    def _inspection_ground(self, *args, **kwargs):
        return 0.0, 0.1

    def _verify_rotate(self, heading, guard=None):
        if guard:
            guard()

    def _verify_control_pose(self):
        return None, deepcopy(self.current)

    def _verify_plan_to(self, target, **kwargs):
        target_map = PoseStamped()
        target_map.header.frame_id = 'map'
        target_map.pose = transform_pose(target, _inverse(self.conversion))
        request = kwargs['request_builder'](target_map, _inverse(self.conversion))
        assert isinstance(request, PlanInspectionPath.Request)
        self.plans.append(request)
        candidate = Path()
        candidate.header.frame_id = 'px4_odom'
        candidate.poses = [PoseStamped(pose=deepcopy(self.current)),
                           PoseStamped(pose=deepcopy(target))]
        assert kwargs['heading_provider'](self.current) == pytest.approx(0.0)
        return kwargs['path_validator'](candidate)

    def _report(self, message):
        pass


def test_cluster_backup_converts_actual_path_to_registry_plane_with_pitched_alignment():
    node = Harness()
    angle = math.pi / 3.0
    node.conversion.rotation = Quaternion(y=math.sin(angle / 2.0), w=math.cos(angle / 2.0))
    node.regions = [region()]
    candidate = path((-2, 0, 3), (2, 0, 3))
    for vertex in candidate.poses:
        vertex.pose = transform_pose(vertex.pose, node.conversion)
    assert all(abs(p.pose.position.x) > 1.0 for p in candidate.poses)
    assert not node._verify_cluster_path(candidate)


def test_departure_native_plan_and_flight_query_explicitly_opt_into_cluster_escape():
    node = Harness()
    node.regions = [region()]
    target = deepcopy(node.current)
    target.position.x += 3.0
    assert node._verify_departure_leg(target, deepcopy(node.current), lambda: None)
    assert node.plans[-1].allow_completed_cluster_escape
    assert node.plans[-1].terrain_envelope and node.plans[-1].allow_terrain_escape
    assert node.queries[-1].allow_completed_cluster_escape
    assert node.queries[-1].escape_origin.header.frame_id == 'map'
    assert node.queries[-1].escape_origin.point == node.current.position


def test_unrelated_corridor_queries_do_not_gain_departure_escape_permission():
    node = Harness()
    candidate = path((2, 0, 4), (3, 0, 4))
    assert node._inspection_map_corridor(candidate)
    assert not node.queries[-1].allow_completed_cluster_escape


def test_flight_query_transforms_fixed_escape_anchor_without_reanchoring_to_remaining_path():
    node = Harness()
    node.conversion.translation.x = 10.0
    node.conversion.translation.z = 3.0
    anchor = Point(x=10.5, z=7.0)
    candidate = path((11, 0, 7), (12, 0, 7))
    state = {'origin': deepcopy(anchor), 'complete': False}
    assert node._inspection_map_corridor(
        candidate, escape_state=state, allow_completed_cluster_escape=True)
    request = node.queries[-1]
    assert request.allow_completed_cluster_escape
    assert request.escape_origin.point == Point(x=0.5, z=4.0)
    assert request.corridor.poses[0].pose.position == Point(x=1.0, z=4.0)
    assert state['origin'] == anchor


class ExplorationHarness(Harness):
    """Exercise actual executive route checks without creating a ROS node or flying."""

    _check_completed_motion = MissionExecutive._check_completed_motion
    _check_motion_observation = MissionExecutive._check_motion_observation
    _execute_path = MissionExecutive._execute_path

    def __init__(self):
        super().__init__()
        self.config['planning_timeout'] = 3.0
        self._airborne_ready = True
        self._parent_state = 'Exploration'
        self._recovering = False
        self._command_inflight = False
        self._motion_guard = None
        self._data_lock = threading.Lock()
        self._target = None
        self._exploration_remaining = None
        self._planning_client = object()
        self.events = []
        self.escape_arguments = []
        self.on_enter = lambda: None
        self.on_fly = lambda: None
        self.plan = path((-2, 1.1, 4), (2, 1.1, 4))
        self.plan.header.frame_id = 'map'
        self.current = deepcopy(self.plan.poses[0].pose)

    def _snapshot(self, expected_mode=None, require_armed=False):
        assert require_armed
        return None, SimpleNamespace(pose=SimpleNamespace(pose=deepcopy(self.current)))

    def _verify_cluster_path(self, candidate, escape_origin=None):
        self.escape_arguments.append(deepcopy(escape_origin))
        return super()._verify_cluster_path(candidate, escape_origin=escape_origin)

    def _call(self, client, request, description, timeout):
        assert client is self._planning_client
        return SimpleNamespace(success=True, path=deepcopy(self.plan))

    def _enter_offboard(self, current, allowed_modes):
        assert allowed_modes == ('AUTO.LOITER',)
        self.events.append('offboard')
        self.on_enter()

    def _fly_to(self, previous, target):
        self.on_fly()
        self._check_motion_observation()
        self.events.append('fly')
        self.current = deepcopy(target)
        return deepcopy(target)

    def _loiter(self):
        self.events.append('loiter')


@pytest.mark.parametrize('altitude', [2.0, 50.0])
def test_exploration_actual_to_target_cannot_cross_old_cluster_at_any_altitude(altitude):
    node = ExplorationHarness()
    node.regions = [region()]
    node.current = path((-2, 0, altitude)).poses[0].pose
    node._target = path((2, 0, altitude)).poses[0].pose
    with pytest.raises(MissionError, match='remaining route enters a completed smoke cluster'):
        node._check_completed_motion()
    assert node.escape_arguments == [None]


def test_exploration_checks_later_route_segments_beyond_the_immediate_target():
    node = ExplorationHarness()
    node.regions = [region(y=-2.0)]
    route = path((-3, -2, 4), (-3, 2, 4), (3, 2, 4), (3, -2, 4), (-3, -2, 4))
    node.current = deepcopy(route.poses[0].pose)
    node._exploration_remaining = [deepcopy(p.pose) for p in route.poses[1:]]
    node._target = deepcopy(route.poses[1].pose)
    with pytest.raises(MissionError, match='remaining route enters'):
        node._check_completed_motion()
    assert node.escape_arguments == [None]


def test_exploration_remaining_route_supersedes_stale_previous_setpoint():
    node = ExplorationHarness()
    node.regions = [region()]
    node.current = path((-2, 1.1, 4)).poses[0].pose
    node._target = path((0, 0, 4)).poses[0].pose
    node._exploration_remaining = [path((2, 1.1, 4)).poses[0].pose]
    node._check_completed_motion()
    assert node.escape_arguments == [None]


@pytest.mark.parametrize('gate', ['no_clusters', 'not_airborne', 'verify'])
def test_completed_motion_checks_leave_legacy_and_other_parent_flows_unchanged(gate):
    node = ExplorationHarness()
    node.regions = [region()] if gate != 'no_clusters' else []
    node._airborne_ready = gate != 'not_airborne'
    node._parent_state = 'Verify' if gate == 'verify' else 'Exploration'

    def unused_snapshot(*args, **kwargs):
        pytest.fail('Unrelated flow must not acquire a new completed-cluster telemetry dependency')

    node._snapshot = unused_snapshot
    node._check_completed_motion()
    assert not node.escape_arguments


def test_exploration_rejects_crossing_plan_before_entering_offboard():
    node = ExplorationHarness()
    node.regions = [region()]
    node.plan = path((-2, 0, 6), (2, 0, 6))
    node.plan.header.frame_id = 'map'
    node.current = deepcopy(node.plan.poses[0].pose)
    with pytest.raises(MissionError, match='remaining route enters'):
        node._execute_path({})
    assert not node.events
    assert node._exploration_remaining is None


def test_exploration_rechecks_new_registry_after_offboard_priming_before_first_move():
    node = ExplorationHarness()
    node.regions = [region(x=10.0, y=10.0)]
    node.plan = path((-2, 0, 6), (2, 0, 6))
    node.plan.header.frame_id = 'map'
    node.current = deepcopy(node.plan.poses[0].pose)
    node.on_enter = lambda: node.regions.append(region(site_id=2))
    with pytest.raises(MissionError, match='remaining route enters'):
        node._execute_path({})
    assert node.events == ['offboard']
    assert node._exploration_remaining is None
    assert node.escape_arguments == [None, None]


def test_exploration_motion_hook_rechecks_actual_drift_against_whole_remaining_route():
    node = ExplorationHarness()
    node.regions = [region()]
    calls = 0

    def drift_after_first_stationary_target():
        nonlocal calls
        calls += 1
        if calls == 2:
            node.current.position = Point(y=0.9, z=4.0)

    node.on_fly = drift_after_first_stationary_target
    with pytest.raises(MissionError, match='remaining route enters'):
        node._execute_path({})
    assert node.events == ['offboard', 'fly']
    assert node._exploration_remaining is None
    assert all(anchor is None for anchor in node.escape_arguments)


def test_exploration_safe_route_finishes_all_sections_without_escape_permission():
    node = ExplorationHarness()
    node.regions = [region()]
    node._execute_path({})
    assert node.events == ['offboard', 'fly', 'fly', 'fly', 'fly', 'loiter']
    assert node._exploration_remaining is None
    assert len(node.escape_arguments) >= 6
    assert all(anchor is None for anchor in node.escape_arguments)
