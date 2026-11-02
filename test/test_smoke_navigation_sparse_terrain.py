"""Exercise smoke navigation against the real native planner on incomplete terrain."""

from copy import deepcopy
import math

from geometry_msgs.msg import Transform
from kv_executive.inspection_navigation import InspectionNavigationMixin
from kv_executive.srv import QueryInspectionGeometry
from nav_msgs.msg import Path
import pytest

from test_planner_inputs import (
    _publish_terrain_scene, _terrain_planner, planner_factory as planner_fixture,
)


planner_factory = planner_fixture


class SmokeNavigationProbe(InspectionNavigationMixin):
    """Simulate arrival only; geometry, planning and path validation use DDS."""

    def __init__(self, planner):
        self.planner = planner
        self.pose = deepcopy(planner.odometry_message.pose.pose)
        self.config = {
            'local_frame': 'planner_test_map', 'verify_planning_frame': 'planner_test_map',
            'planning_timeout': 3.0, 'verify_geometry_timeout': 5.0,
            'verify_future_tolerance': 0.25, 'verify_observation_timeout': 2.0,
            'verify_minimum_altitude': 2.0, 'verify_inspection_altitude': 5.0,
            'verify_motion_speed': 0.5, 'position_tolerance': 0.3,
        }
        self._inspection_geometry_client = planner.geometry_client
        self._inspection_planning_client = planner.inspection_client
        self.messages = []
        self.paths = []

    def get_clock(self):
        return self.planner.node.get_clock()

    def _check_running(self):
        pass

    def _verify_control_pose(self):
        return None, deepcopy(self.pose)

    def _path_frame_transform(self, frame):
        assert frame == self.config['local_frame']
        transform = Transform()
        transform.rotation.w = 1.0
        return transform

    def _report(self, message):
        self.messages.append(message)

    def _call(self, client, request, description, timeout=None):
        assert client is self.planner.geometry_client
        return self.planner.query_geometry(request)

    def arrive(self, pose):
        self.pose = deepcopy(pose)
        self.planner.odometry_message.pose.pose = deepcopy(pose)
        self.planner.repeat_odometry()
        self.planner.wait_for_response(
            lambda response: response.success and all(
                abs(getattr(response.path.poses[0].pose.position, axis)
                    - getattr(pose.position, axis)) < 1e-6 for axis in 'xyz'),
            'native planner to observe arrival odometry')

    def _verify_plan_to(self, target, *, path_validator, request_builder,
                        planning_client, guard=None, **unused):
        assert planning_client is self.planner.inspection_client
        if guard:
            guard()
        stamped = self.planner.target()
        stamped.pose = deepcopy(target)
        request = request_builder(stamped, self._path_frame_transform(stamped.header.frame_id))
        assert request.minimum_agl == 5.0 and request.terrain_envelope
        response = self.planner.plan_inspection(request)
        if not response.success:
            self._report(response.message)
            return False
        assert path_validator(response.path), self.messages
        self.paths.append(deepcopy(response.path))
        self.arrive(response.path.poses[-1].pose)
        return True


@pytest.mark.parametrize('ground_absent', [False, True])
def test_fire_guided_descent_uses_native_terrain_checks(planner_factory, ground_absent):
    planner = _terrain_planner(
        planner_factory, ground_absent=ground_absent, planner_parameters={
            'inspection_require_observed_free_space': None,
            'inspection_map_timeout': 5.0, 'drone_radius': 0.45,
            'obstacle_clearance': 0.3, 'maximum_altitude': 30.0,
        })
    probe = SmokeNavigationProbe(planner)
    probe.arrive(planner.target(x=3.0, z=10.0).pose)
    initial = deepcopy(probe.pose)
    # Face back down the mapped slope. A lower fire requests half a metre of
    # descent, but an absent ground surface cannot authorize that descent.
    bearing = math.pi - 0.1
    moved = probe._verify_navigation_viewpoint(
        bearing, vertical_offset=-0.5, forward_only=True)
    assert moved is not ground_absent, '\n'.join(probe.messages)
    if ground_absent:
        assert probe.pose == initial
        assert not probe.paths
    else:
        assert probe.pose.position.z == pytest.approx(9.5)
        assert probe.pose.position.x == pytest.approx(3.0 + 3.0 * math.cos(bearing))
        assert len(probe.paths) == 1
        checked = planner.query_geometry(QueryInspectionGeometry.Request(
            corridor=probe.paths[0], minimum_agl=5.0, terrain_envelope=True))
        assert checked.success and checked.corridor_safe, checked.message


def test_smoke_advances_over_missing_ground_then_shortens_leg_before_new_canopy(planner_factory):
    planner = _terrain_planner(
        planner_factory, ground_absent=True, planner_parameters={
            'inspection_require_observed_free_space': None,
            'inspection_map_timeout': 5.0, 'drone_radius': 0.45,
            'obstacle_clearance': 0.3, 'maximum_altitude': 30.0,
        })
    probe = SmokeNavigationProbe(planner)
    probe.arrive(planner.target(x=-6.0, z=10.0).pose)

    for expected_x in (-3.0, 0.0):
        _publish_terrain_scene(planner, ground_absent=True)
        assert probe._verify_navigation_viewpoint(0.0, forward_only=True), probe.messages
        position = probe.pose.position
        assert (position.x, position.y, position.z) == pytest.approx((expected_x, 0.0, 10.0))
        assert all(vertex.pose.position.z >= 9.7 for vertex in probe.paths[-1].poses)

    # A fresh observation adds a tree on rising terrain in the smoke direction.
    # The former level corridor must now fail the very same native validation.
    _publish_terrain_scene(planner, tree=True)
    level_path = Path(poses=[planner.target(z=10.0), planner.target(x=3.0, z=10.0)])
    level_path.header.frame_id = 'planner_test_map'
    blocked = planner.query_geometry(QueryInspectionGeometry.Request(
        corridor=level_path, minimum_agl=5.0, terrain_envelope=True))
    assert blocked.success and not blocked.corridor_safe, blocked.message

    assert probe._verify_navigation_viewpoint(0.0, forward_only=True), probe.messages
    assert len(probe.paths) == 3
    # The canopy is ahead of the body. That future obstacle cannot authorize
    # an oversized safety climb; choose a shorter level leg while it is clear.
    assert probe.pose.position.x == pytest.approx(1.5)
    assert probe.pose.position.z == pytest.approx(10.0)
    assert all(vertex.pose.position.z <= 11.0 for vertex in probe.paths[-1].poses)
    assert any('proposed destination rejected' in text and 'ordinary approach' in text
               for text in probe.messages)
    checked = planner.query_geometry(QueryInspectionGeometry.Request(
        corridor=probe.paths[-1], minimum_agl=5.0, terrain_envelope=True))
    assert checked.success and checked.corridor_safe, checked.message
