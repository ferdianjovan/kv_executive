"""Exercise the installed planner through DDS without a flight controller."""

from copy import deepcopy
from functools import lru_cache
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Vector3Stamped
from kv_executive.inspection_navigation import InspectionNavigationMixin
from kv_executive.msg import (
    CompletedInspectionCluster, CompletedInspectionClusters, FireSourceReport,
)
from kv_executive.srv import PlanInspectionPath, PlanPath, PlanToPose, QueryInspectionGeometry
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster


class PlannerHarness:
    """Run the real C++ planner with all inputs confined to a test namespace."""

    def __init__(
        self, directory, *, home_from_first_odometry=True, cached_map=False,
        planner_parameters=None,
    ):
        self.namespace = f'/planner_inputs_{os.getpid()}_{uuid.uuid4().hex[:8]}'
        self.odometry_topic = f'{self.namespace}/external_odometry'
        self.home_topic = f'{self.namespace}/home'
        self.map_topic = f'{self.namespace}/retained_map'
        self.home_from_first_odometry = home_from_first_odometry
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node('planner_input_test', namespace=self.namespace, context=self.context)
        self.transform_broadcaster = StaticTransformBroadcaster(self.node)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.odometry_publisher = self.node.create_publisher(
            Odometry, self.odometry_topic, qos_profile_sensor_data,
        )
        self.odometry_message = None
        self.odometry_timer = self.node.create_timer(0.05, self.repeat_odometry)
        self.home_publisher = self.node.create_publisher(
            HomePosition, self.home_topic, qos_profile_sensor_data,
        )
        self.map_publisher = self.node.create_publisher(
            Octomap, self.map_topic,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.client = self.node.create_client(PlanPath, f'{self.namespace}/plan_path')
        self.target_client = self.node.create_client(
            PlanToPose, f'{self.namespace}/plan_to_pose',
        )
        self.inspection_client = self.node.create_client(
            PlanInspectionPath, f'{self.namespace}/plan_inspection_path',
        )
        self.geometry_client = self.node.create_client(
            QueryInspectionGeometry, f'{self.namespace}/query_inspection_geometry',
        )
        self.report_publisher = self.node.create_publisher(
            FireSourceReport, f'{self.namespace}/fire_sources',
            QoSProfile(
                depth=100, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.cluster_publisher = self.node.create_publisher(
            CompletedInspectionClusters, f'{self.namespace}/completed_inspection_clusters',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.map_publications = 0
        self.process = None
        self.log_path = directory / 'planner.log'
        self.log_file = self.log_path.open('w')
        try:
            if cached_map:
                # Publish before the planner even exists. Successful planning
                # therefore requires the subscriber to request retained data.
                self.publish_map()
            parameters = {
                'horizontal_range': 1.0,
                'vertical_range': 3.0,
                'minimum_altitude': 0.5,
                'maximum_altitude': 8.0,
                'bound_margin': 0.1,
                'yaw_weight': 1.0,
                'drone_radius': 0.1,
                'obstacle_clearance': 0.1,
                'validity_resolution_fraction': 0.01,
                'planning_timeout': 0.5,
                'simplification_time': 0.01,
                # Most fixtures exercise the optional observed-free contract.
                'inspection_require_observed_free_space': True,
                'odometry_topic': self.odometry_topic,
                'home_position_topic': self.home_topic,
                'octomap_topic': self.map_topic,
                'mavros_parameter_service': f'{self.namespace}/unused_px4_parameters',
            }
            if not home_from_first_odometry:
                # Leave the default unoverridden in the first-odometry case.
                parameters['home_from_first_odometry'] = False
            parameters.update(planner_parameters or {})
            binary = (
                Path(get_package_prefix('kv_executive'))
                / 'lib' / 'kv_executive' / 'motion_planning'
            )
            arguments = [str(binary), '--ros-args', '-r', f'__ns:={self.namespace}']
            for name, value in parameters.items():
                if value is None:
                    continue  # Exercise a native default instead of overriding it.
                if isinstance(value, bool):
                    value = str(value).lower()
                arguments.extend(['-p', f'{name}:={value}'])
            self.process = subprocess.Popen(
                arguments, stdout=self.log_file, stderr=subprocess.STDOUT,
            )
            self.wait_for(
                lambda: self.client.service_is_ready()
                and self.target_client.service_is_ready()
                and self.inspection_client.service_is_ready()
                and self.geometry_client.service_is_ready()
                and self.odometry_publisher.get_subscription_count() > 0,
                'planner service and odometry subscription',
            )
        except BaseException:
            self.close()
            raise

    def wait_for(self, predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process is not None and self.process.poll() is not None:
                pytest.fail(f'Planner exited while waiting for {description}:\n{self.logs()}')
            self.executor.spin_once(timeout_sec=0.02)
        pytest.fail(f'Timed out waiting for {description}:\n{self.logs()}')

    def logs(self):
        self.log_file.flush()
        return self.log_path.read_text()[-16000:]

    def plan(self):
        future = self.client.call_async(PlanPath.Request())
        self.wait_for(future.done, 'PlanPath response')
        return future.result()

    def plan_to(self, target):
        request = PlanToPose.Request()
        request.target = target
        future = self.target_client.call_async(request)
        self.wait_for(future.done, 'PlanToPose response')
        return future.result()

    def query_geometry(self, request=None):
        """Call the real planner's geometry service through DDS."""
        future = self.geometry_client.call_async(request or QueryInspectionGeometry.Request())
        self.wait_for(future.done, 'QueryInspectionGeometry response')
        return future.result()

    def plan_inspection(self, request):
        future = self.inspection_client.call_async(request)
        self.wait_for(future.done, 'PlanInspectionPath response', timeout=8.0)
        return future.result()

    def target(self, x=0.0, y=0.0, z=2.0, yaw=0.0, frame='planner_test_map'):
        target = PoseStamped()
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.header.frame_id = frame
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation.z = math.sin(yaw * 0.5)
        target.pose.orientation.w = math.cos(yaw * 0.5)
        return target

    def wait_for_response(self, predicate, description):
        response = None

        def received():
            nonlocal response
            response = self.plan()
            if predicate(response):
                return True
            time.sleep(0.02)
            return False

        self.wait_for(received, description)
        return response

    def publish_odometry(self, z):
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'planner_test_map'
        message.child_frame_id = 'planner_test_drone'
        message.pose.pose.position.z = float(z)
        message.pose.pose.orientation.w = 1.0
        self.odometry_message = message
        self.odometry_publisher.publish(message)

    def repeat_odometry(self):
        if self.odometry_message is not None:
            self.odometry_message.header.stamp = self.node.get_clock().now().to_msg()
            self.odometry_publisher.publish(self.odometry_message)

    def publish_home(self):
        message = HomePosition()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'planner_test_map'
        message.orientation.w = 1.0
        self.home_publisher.publish(message)

    def publish_map(self):
        message = Octomap()
        message.header.frame_id = 'planner_test_map'
        message.binary = True
        message.id = 'OcTree'
        message.resolution = 0.25
        # OctoMap's binary format uses two bits per child: 01 is free.
        # Eight free root children form a valid, nonempty, unobstructed tree.
        message.data = [0x55, 0x55]
        self.map_publisher.publish(message)
        self.map_publications += 1

    def close(self):
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.0)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
        self.log_file.close()


@pytest.fixture
def planner_factory(monkeypatch, tmp_path):
    # Never discover a real planner, MAVROS, or PX4 instance on this machine.
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get(
        'ROS_DOMAIN_ID', str(160 + os.getpid() % 30)))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    planners = []

    def create(**parameters):
        directory = tmp_path / f'instance_{len(planners)}'
        directory.mkdir()
        planner = PlannerHarness(directory, **parameters)
        planners.append(planner)
        return planner

    yield create
    for planner in reversed(planners):
        planner.close()


@pytest.mark.parametrize('home_from_first_odometry', [True, False])
def test_planner_identifies_each_missing_input(planner_factory, home_from_first_odometry):
    planner = planner_factory(home_from_first_odometry=home_from_first_odometry)
    home_source = planner.odometry_topic if home_from_first_odometry else planner.home_topic
    response = planner.plan()
    assert not response.success
    assert response.message == (
        f'Missing inputs: odometry ({planner.odometry_topic}), '
        f'home ({home_source}), OctoMap ({planner.map_topic})'
    )

    planner.publish_odometry(0.0)
    response = planner.wait_for_response(
        lambda result: f'odometry ({planner.odometry_topic})' not in result.message,
        'planner to receive external odometry',
    )
    if not home_from_first_odometry:
        assert response.message == (
            f'Missing inputs: home ({planner.home_topic}), OctoMap ({planner.map_topic})'
        )
        planner.wait_for(
            lambda: planner.home_publisher.get_subscription_count() > 0,
            'explicit home subscription',
        )
        planner.publish_home()
        response = planner.wait_for_response(
            lambda result: f'home ({planner.home_topic})' not in result.message,
            'planner to receive explicit home',
        )
    assert not response.success
    assert response.message == f'Missing inputs: OctoMap ({planner.map_topic})'


def test_planner_receives_retained_map_and_plans_from_external_odometry(planner_factory):
    planner = planner_factory(cached_map=True)
    planner.publish_odometry(0.0)
    planner.wait_for_response(
        lambda response: not response.message.startswith('Missing inputs:'),
        'retained OctoMap and initial home odometry',
    )
    planner.publish_odometry(2.0)
    response = planner.wait_for_response(
        lambda result: result.success,
        'collision-free plan from airborne external odometry',
    )
    assert planner.map_publications == 1
    assert response.path.header.frame_id == 'planner_test_map'
    assert len(response.path.poses) >= 2
    assert all(pose.header.frame_id == 'planner_test_map' for pose in response.path.poses)
    start = response.path.poses[0].pose.position
    assert (start.x, start.y, start.z) == pytest.approx((0.0, 0.0, 2.0))
    assert all(0.6 <= pose.pose.position.z <= 5.0 for pose in response.path.poses)


def airborne_planner(planner_factory, **parameters):
    planner = planner_factory(cached_map=True, **parameters)
    planner.publish_odometry(0.0)
    planner.wait_for_response(
        lambda response: not response.message.startswith('Missing inputs:'),
        'initial home and retained map',
    )
    planner.publish_odometry(2.0)
    planner.wait_for_response(lambda result: result.success, 'airborne planner readiness')
    return planner


def test_explicit_target_extends_rolling_bounds_and_preserves_endpoint(planner_factory):
    planner = airborne_planner(
        planner_factory, planner_parameters={'maximum_altitude': 30.0},
    )
    target = planner.target(x=20.0, y=1.0, z=12.0, yaw=1.2)
    response = planner.plan_to(target)
    assert response.success, response.message
    assert response.path.header.frame_id == 'planner_test_map'
    endpoint = response.path.poses[-1].pose
    assert (
        endpoint.position.x, endpoint.position.y, endpoint.position.z,
    ) == pytest.approx((20.0, 1.0, 12.0))
    assert (
        endpoint.orientation.z, endpoint.orientation.w,
    ) == pytest.approx((math.sin(0.6), math.cos(0.6)))
    assert all(0.6 <= pose.pose.position.z <= 29.9 for pose in response.path.poses)
    # An explicit request must not permanently expand exploration's rolling window.
    random_response = planner.plan()
    assert random_response.success, random_response.message
    assert all(
        abs(pose.pose.position.x) <= 1.0 and abs(pose.pose.position.y) <= 1.0
        and 0.6 <= pose.pose.position.z <= 5.0
        for pose in random_response.path.poses
    )


@pytest.mark.parametrize(
    ('target_change', 'reason'),
    [
        ({'frame': ''}, 'frame_id'),
        ({'x': float('nan')}, 'finite position'),
        ({'z': float('inf')}, 'finite position'),
        ({'x': 51.0}, 'maximum_target_distance'),
        ({'z': 0.5}, 'outside the current planning bounds'),
        ({'z': 8.0}, 'outside the current planning bounds'),
        ({'frame': 'unknown_target_frame'}, 'Cannot transform target'),
    ],
)
def test_explicit_target_rejects_unsafe_or_unresolvable_request(
    planner_factory, target_change, reason,
):
    planner = airborne_planner(planner_factory)
    response = planner.plan_to(planner.target(**target_change))
    assert not response.success
    assert reason in response.message
    assert not response.path.poses


def test_explicit_target_rejects_zero_quaternion(planner_factory):
    planner = airborne_planner(planner_factory)
    target = planner.target(x=1.0)
    target.pose.orientation.w = 0.0
    response = planner.plan_to(target)
    assert not response.success
    assert 'nonzero finite quaternion' in response.message
    assert not response.path.poses


def test_explicit_target_is_transformed_into_actual_octomap_frame(planner_factory):
    planner = airborne_planner(planner_factory)
    transform = TransformStamped()
    transform.header.stamp = planner.node.get_clock().now().to_msg()
    transform.header.frame_id = 'planner_test_map'
    transform.child_frame_id = planner.namespace.strip('/') + '_target'
    transform.transform.translation.x = 3.0
    transform.transform.translation.y = -2.0
    transform.transform.rotation.z = math.sin(math.pi / 4.0)
    transform.transform.rotation.w = math.cos(math.pi / 4.0)
    planner.transform_broadcaster.sendTransform(transform)
    target = planner.target(x=1.0, z=2.0, yaw=0.3, frame=transform.child_frame_id)
    response = planner.plan_to(target)
    assert response.success, response.message
    endpoint = response.path.poses[-1].pose
    assert (
        endpoint.position.x, endpoint.position.y, endpoint.position.z,
    ) == pytest.approx((3.0, -1.0, 2.0))
    expected_yaw = math.pi / 2.0 + 0.3
    assert (
        endpoint.orientation.z, endpoint.orientation.w,
    ) == pytest.approx((math.sin(expected_yaw / 2.0), math.cos(expected_yaw / 2.0)))


def test_reported_fire_regions_exclude_only_exploration_goals(planner_factory):
    planner = airborne_planner(planner_factory)
    planner.wait_for(
        lambda: planner.report_publisher.get_subscription_count() > 0,
        'fire-source report subscription',
    )
    report = FireSourceReport()
    report.site_id = 1
    report.label = 'Fire source found'
    report.planning_pose = planner.target()
    report.exclusion_radius = 15.0
    planner.report_publisher.publish(report)
    planner.wait_for(
        lambda: 'Recorded fire-source exploration exclusion region' in planner.logs(),
        'fire-source report reception',
    )
    response = planner.plan()
    assert not response.success
    assert 'No exploration goal outside reported fire-source regions' in response.message
    assert not response.path.poses
    response = planner.plan_to(planner.target(x=0.5))
    assert response.success, response.message


def test_exploration_endpoint_is_outside_partial_reported_region(planner_factory):
    planner = airborne_planner(planner_factory)
    report = FireSourceReport()
    report.site_id = 2
    report.label = 'Fire source found'
    report.planning_pose = planner.target()
    report.exclusion_radius = 0.5
    planner.report_publisher.publish(report)
    planner.wait_for(
        lambda: 'Recorded fire-source exploration exclusion region' in planner.logs(),
        'partial fire-source region reception',
    )
    for _ in range(5):
        response = planner.plan()
        assert response.success, response.message
        endpoint = response.path.poses[-1].pose.position
        assert math.hypot(endpoint.x, endpoint.y) >= 0.5


def _publish_completed_clusters(planner, circles, *, mutate=None, accepted=True):
    snapshot = CompletedInspectionClusters()
    snapshot.header.frame_id = 'planner_test_map'
    snapshot.header.stamp = planner.node.get_clock().now().to_msg()
    for site_id, x, y, radius in circles:
        cluster = CompletedInspectionCluster(site_id=site_id, radius=float(radius))
        cluster.center.header = deepcopy(snapshot.header)
        cluster.center.point.x, cluster.center.point.y = float(x), float(y)
        snapshot.clusters.append(cluster)
    if mutate:
        mutate(snapshot)
    marker = ('Completed inspection cluster snapshot accepted' if accepted
              else 'Completed inspection cluster registry rejected')
    previous = planner.logs().count(marker)
    planner.cluster_publisher.publish(snapshot)
    planner.wait_for(lambda: planner.logs().count(marker) > previous,
                     'completed-cluster snapshot reception')
    return snapshot


def _assert_path_outside_circles(path, circles):
    assert path.poses
    for index, vertex in enumerate(path.poses):
        end = vertex.pose.position
        start = path.poses[index - 1].pose.position if index else end
        for _, x, y, radius in circles:
            sx, sy = start.x - x, start.y - y
            dx, dy = end.x - start.x, end.y - start.y
            squared = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, -(sx * dx + sy * dy) / squared)) if squared else 0.0
            assert math.hypot(sx + fraction * dx, sy + fraction * dy) >= radius - 1e-6


@pytest.mark.parametrize('altitude', [2.0, 6.0])
def test_completed_cluster_cylinder_blocks_transit_even_above_smoke(
        planner_factory, altitude):
    planner = _strict_planner(planner_factory, wall=False)
    circles = [(1, 0.5, 0.0, 0.4)]
    _publish_completed_clusters(planner, circles)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=-1.0, z=altitude),
                              planner.target(x=2.0, z=altitude)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe
    assert 'completed smoke-cluster keepout' in result.message
    request.corridor.poses = [planner.target(x=-1.0, z=altitude),
                              planner.target(x=-1.0, y=1.0, z=altitude),
                              planner.target(x=2.0, y=1.0, z=altitude),
                              planner.target(x=2.0, z=altitude)]
    result = planner.query_geometry(request)
    assert result.success and result.corridor_safe, result.message


def test_target_and_exploration_plans_enforce_cluster_edges_not_only_goals(planner_factory):
    planner = airborne_planner(planner_factory, planner_parameters={'planning_timeout': 2.0})
    circles = [(1, 0.7, 0.0, 0.35)]
    _publish_completed_clusters(planner, circles)
    response = planner.plan_to(planner.target(x=1.5))
    assert response.success, response.message
    _assert_path_outside_circles(response.path, circles)
    for _ in range(5):
        response = planner.plan()
        assert response.success, response.message
        _assert_path_outside_circles(response.path, circles)


def test_inside_cluster_requires_explicit_escape_and_cannot_escape_by_climbing(planner_factory):
    planner = _strict_planner(planner_factory, wall=False)
    _publish_completed_clusters(planner, [(1, -0.5, 0.0, 1.0)])
    request = PlanInspectionPath.Request(target=planner.target(x=1.5))
    result = planner.plan_inspection(request)
    assert not result.success and result.error_code == result.START_INVALID
    assert not planner.plan().success
    request.allow_completed_cluster_escape = True
    result = planner.plan_inspection(request)
    assert result.success, result.message
    assert result.path.poses[-1].pose.position.x == pytest.approx(1.5)
    request.target = planner.target(z=3.0)
    result = planner.plan_inspection(request)
    assert not result.success
    assert not result.path.poses


@pytest.mark.parametrize('route,safe', [
    ([(0.0, 0.0), (1.5, 0.0)], True),
    ([(0.0, 0.0), (-0.2, 0.0)], False),
    ([(0.0, 0.0), (0.3, 0.0), (0.1, 0.0)], False),
    ([(0.0, 0.0), (1.5, 0.0), (0.2, 0.0)], False),
    ([(0.7, 0.0), (0.2, 0.0)], False),
])
def test_cluster_escape_is_outward_from_fixed_anchor_and_cannot_reenter(
        planner_factory, route, safe):
    planner = _strict_planner(planner_factory, wall=False)
    _publish_completed_clusters(planner, [(1, -0.5, 0.0, 1.0)])
    request = QueryInspectionGeometry.Request(allow_completed_cluster_escape=True)
    request.escape_origin = _geometry_point(x=0.0, y=0.0, z=2.0)
    request.corridor.poses = [planner.target(x=x, y=y) for x, y in route]
    result = planner.query_geometry(request)
    assert result.success and result.corridor_safe is safe, result.message


def test_overlapping_clusters_require_outward_progress_from_both(planner_factory):
    planner = _strict_planner(planner_factory, wall=False)
    _publish_completed_clusters(planner, [(1, -0.5, 0.0, 1.0), (2, 0.5, 0.0, 1.0)])
    request = QueryInspectionGeometry.Request(allow_completed_cluster_escape=True)
    request.corridor.poses = [planner.target(), planner.target(y=1.5)]
    result = planner.query_geometry(request)
    assert result.success and result.corridor_safe, result.message
    request.corridor.poses = [planner.target(), planner.target(x=2.0)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe


def test_cluster_registry_leaves_stationary_health_and_plain_geometry_available(planner_factory):
    planner = _strict_planner(planner_factory, wall=False)
    _publish_completed_clusters(planner, [(1, 0.0, 0.0, 25.0)])
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target()]
    assert planner.query_geometry(request).corridor_safe
    request.corridor.poses.append(planner.target(yaw=1.0))
    assert planner.query_geometry(request).corridor_safe
    assert planner.query_geometry(_inspection_request()).success


def test_cluster_escape_never_waives_occupied_collision_clearance(planner_factory):
    planner = _strict_planner(planner_factory, wall=True)
    _publish_completed_clusters(planner, [(1, -0.5, 0.0, 1.0)])
    request = QueryInspectionGeometry.Request(allow_completed_cluster_escape=True)
    request.corridor.poses = [planner.target(), planner.target(x=4.0)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe
    assert 'occupied' in result.message


def _wall_front_circle(distance, angle=0.0, site_id=1, radius=25.0):
    """Keep the true front at W; route around the circle centred beyond that front."""
    nx, ny = math.cos(angle), math.sin(angle)
    return site_id, (distance + radius) * nx, (distance + radius) * ny, radius


@pytest.mark.parametrize('wall_distance', [15.0, 20.0])
@pytest.mark.parametrize('angle', [0.0, math.pi / 2.0])
def test_shifted_wall_front_leaves_observer_outside_without_forced_retreat(
        planner_factory, wall_distance, angle):
    planner = _strict_planner(planner_factory, wall=False, planner_parameters={
        'inspection_require_observed_free_space': False})
    circles = [_wall_front_circle(wall_distance, angle)]
    _publish_completed_clusters(planner, circles)
    request = PlanInspectionPath.Request(target=planner.target(
        x=3.0 * math.cos(angle), y=3.0 * math.sin(angle)))
    assert not request.allow_completed_cluster_escape
    result = planner.plan_inspection(request)
    assert result.success, result.message
    start = result.path.poses[0].pose.position
    assert (start.x, start.y, start.z) == pytest.approx((0.0, 0.0, 2.0))
    _assert_path_outside_circles(result.path, circles)
    assert planner.plan().success


@pytest.mark.parametrize('wall_distance', [15.0, 20.0])
@pytest.mark.parametrize('angle', [0.0, math.pi / 3.0])
def test_shifted_front_allows_tangent_plane_but_blocks_penetration_and_overflight(
        planner_factory, wall_distance, angle):
    planner = _strict_planner(planner_factory, wall=False, planner_parameters={
        'inspection_require_observed_free_space': False})
    _publish_completed_clusters(planner, [_wall_front_circle(wall_distance, angle)])
    nx, ny = math.cos(angle), math.sin(angle)
    tx, ty = -ny, nx
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(
        x=wall_distance * nx + side * tx, y=wall_distance * ny + side * ty)
        for side in (-2.0, 2.0)]
    result = planner.query_geometry(request)
    assert result.success and result.corridor_safe, result.message
    for altitude in (2.0, 6.0):
        request.corridor.poses = [planner.target(
            x=distance * nx, y=distance * ny, z=altitude)
            for distance in (wall_distance - 0.1, wall_distance + 0.1)]
        result = planner.query_geometry(request)
        assert result.success and not result.corridor_safe
        assert 'completed smoke-cluster keepout' in result.message


def test_multiple_shifted_fronts_independently_protect_their_behind_wall_volume(planner_factory):
    planner = _strict_planner(planner_factory, wall=False, planner_parameters={
        'inspection_require_observed_free_space': False})
    circles = [_wall_front_circle(15.0), _wall_front_circle(20.0, math.pi / 2.0, site_id=2)]
    _publish_completed_clusters(planner, circles)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(), planner.target(x=14.5),
                              planner.target(y=19.5)]
    result = planner.query_geometry(request)
    assert result.success and result.corridor_safe, result.message
    for target in (planner.target(x=15.1), planner.target(y=20.1)):
        request.corridor.poses = [planner.target(), target]
        result = planner.query_geometry(request)
        assert result.success and not result.corridor_safe
        assert 'completed smoke-cluster keepout' in result.message


def test_start_seeded_behind_shifted_front_still_requires_checked_outward_escape(planner_factory):
    planner = _strict_planner(planner_factory, wall=False, planner_parameters={
        'inspection_require_observed_free_space': False})
    _publish_completed_clusters(planner, [_wall_front_circle(15.0)])
    planner.odometry_message.pose.pose.position.x = 16.0
    planner.repeat_odometry()
    request = PlanInspectionPath.Request(target=planner.target(x=13.0))
    planner.wait_for(
        lambda: planner.plan_inspection(request).error_code
        == PlanInspectionPath.Response.START_INVALID,
        'actual start behind the shifted wall front')
    request.allow_completed_cluster_escape = True
    result = planner.plan_inspection(request)
    assert result.success, result.message
    assert result.path.poses[0].pose.position.x == pytest.approx(16.0)
    assert result.path.poses[-1].pose.position.x == pytest.approx(13.0)
    check = QueryInspectionGeometry.Request(corridor=result.path)
    assert not planner.query_geometry(check).corridor_safe
    check.allow_completed_cluster_escape = True
    check.escape_origin = _geometry_point(x=16.0, y=0.0, z=2.0)
    assert planner.query_geometry(check).corridor_safe
    check.corridor.poses = [planner.target(x=16.0), planner.target(x=17.0)]
    assert not planner.query_geometry(check).corridor_safe


@pytest.mark.parametrize('x,safe', [(2.7, True), (2.9, False), (3.125, False)])
def test_shifted_semantic_front_never_waives_actual_wall_body_clearance(planner_factory, x, safe):
    planner = _strict_planner(planner_factory, wall=True)
    circles = [_wall_front_circle(3.125)]
    _publish_completed_clusters(planner, circles)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=x, y=-0.5), planner.target(x=x, y=0.5)]
    _assert_path_outside_circles(request.corridor, circles)
    result = planner.query_geometry(request)
    assert result.success and result.corridor_safe is safe, result.message
    if not safe:
        assert 'occupied' in result.message


@pytest.mark.parametrize('invalid', ['frame', 'point_frame', 'radius', 'ids', 'count', 'older'])
def test_invalid_cluster_snapshot_blocks_transit_until_valid_snapshot_replaces_it(
        planner_factory, invalid):
    planner = _strict_planner(planner_factory, wall=False)
    previous = _publish_completed_clusters(planner, [(1, 1.0, 0.0, 0.25)])

    def corrupt(snapshot):
        if invalid == 'frame':
            snapshot.header.frame_id = 'wrong_map'
        elif invalid == 'point_frame':
            snapshot.clusters[0].center.header.frame_id = 'wrong_map'
        elif invalid == 'radius':
            snapshot.clusters[0].radius = math.nan
        elif invalid == 'ids':
            snapshot.clusters.append(deepcopy(snapshot.clusters[0]))
        elif invalid == 'count':
            snapshot.clusters = [CompletedInspectionCluster(
                site_id=index + 1, center=deepcopy(snapshot.clusters[0].center), radius=0.25)
                for index in range(101)]
        else:
            snapshot.header.stamp = deepcopy(previous.header.stamp)
            snapshot.header.stamp.sec -= 1

    _publish_completed_clusters(planner, [(1, 1.0, 0.0, 0.25)], mutate=corrupt, accepted=False)
    result = planner.plan_inspection(PlanInspectionPath.Request(target=planner.target(x=0.5)))
    assert not result.success and result.error_code == result.INVALID_REQUEST
    assert 'cluster registry' in result.message
    assert planner.query_geometry(_inspection_request()).success
    _publish_completed_clusters(planner, [])
    result = planner.plan_inspection(PlanInspectionPath.Request(target=planner.target(x=0.5)))
    assert result.success, result.message


@lru_cache(maxsize=4)
def _inspection_scene(wall=True, rotate=False):
    """Encode observed 0.25 m voxels, a floor, and an optional thin wall."""
    root = {}
    for x in range(-8, 24):
        for y in range(-8, 8):
            for z in range(-1, 32):
                if wall == 'unknown_wall' and x == 12:
                    continue
                keys = ((y + 32768, x + 32768, z + 32768) if rotate
                        else (x + 32768, y + 32768, z + 32768))
                node = root
                for bit in range(15, -1, -1):
                    child = sum(((key >> bit) & 1) << axis
                                for axis, key in enumerate(keys))
                    if bit:
                        node = node.setdefault(child, {})
                    else:
                        blocked = wall and x == 12
                        if wall == 'pillar':
                            blocked = blocked and -2 <= y <= 2 and z <= 16
                        elif wall == 'lateral':
                            blocked = y >= 2
                        elif isinstance(wall, tuple):
                            blocked = blocked and (y, z) in wall
                        node[child] = 2 if z == -1 or blocked else 1

    def compress(node):
        values = {index: compress(value) if isinstance(value, dict) else value
                  for index, value in node.items()}
        if len(values) == 8 and all(value == 1 for value in values.values()):
            return 1
        if len(values) == 8 and all(value == 2 for value in values.values()):
            return 2
        return values

    data = []

    def encode(node):
        bits = sum((3 if isinstance(value, dict) else value) << (2 * index)
                   for index, value in node.items())
        for value in (bits & 255, bits >> 8):
            data.append(value if value < 128 else value - 256)
        for _, value in sorted(node.items()):
            if isinstance(value, dict):
                encode(value)

    encode(compress(root))
    return tuple(data)


def _geometry_point(x=0.125, y=0.125, z=2.125, frame='planner_test_map'):
    result = PointStamped()
    result.header.frame_id = frame
    result.point.x, result.point.y, result.point.z = float(x), float(y), float(z)
    return result


def _inspection_request():
    request = QueryInspectionGeometry.Request()
    request.origin = _geometry_point()
    request.direction = Vector3Stamped()
    request.direction.header.frame_id = 'planner_test_map'
    request.direction.vector.x = 1.0
    request.max_range = 5.0
    return request


def _publish_inspection_map(planner, *, wall=True, age=0.0, rotate=False):
    message = Octomap()
    message.header.frame_id = 'planner_test_map'
    message.header.stamp = planner.node.get_clock().now().to_msg()
    message.header.stamp.sec -= int(age)
    message.binary = True
    message.id = 'OcTree'
    message.resolution = 0.25
    message.data = _inspection_scene(wall, rotate)
    planner.map_publisher.publish(message)
    if age == 0.0:
        planner.wait_for(
            lambda: planner.query_geometry().map_header.stamp == message.header.stamp,
            'source-stamped inspection map reception',
        )
    else:
        planner.wait_for(
            lambda: 'Rejected OctoMap' in planner.logs(),
            'stale inspection map rejection',
        )
    return message


def test_inspection_geometry_requires_source_and_receipt_freshness(planner_factory):
    planner = planner_factory(planner_parameters={'inspection_map_timeout': 0.8})
    missing = planner.query_geometry()
    assert not missing.success and missing.error_code == missing.MAP_UNAVAILABLE
    assert 'no accepted OctoMap' in missing.message
    _publish_inspection_map(planner, age=10.0)
    stale = planner.query_geometry()
    assert not stale.success and stale.error_code == stale.MAP_UNAVAILABLE
    assert 'source observation is too old' in stale.message
    assert 'source_age=' in stale.message and 'receipt_silence=' in stale.message
    assert 'timeout=0.8 s' in stale.message
    _publish_inspection_map(planner)
    assert planner.query_geometry().success
    time.sleep(0.9)
    expired = planner.query_geometry()
    assert not expired.success and expired.error_code == expired.MAP_UNAVAILABLE
    assert 'source observation is too old' in expired.message
    assert expired.map_header.stamp.sec > 0


@pytest.mark.parametrize('problem', ['future', 'stale', 'missing_frame'])
def test_invalid_source_maps_are_rejected_before_decoding(planner_factory, problem):
    planner = planner_factory()
    message = Octomap(binary=True, id='OcTree', resolution=0.25)
    message.header.frame_id = 'planner_test_map'
    message.header.stamp = planner.node.get_clock().now().to_msg()
    # Deliberately empty payload: source validation must run first.
    if problem == 'future':
        message.header.stamp.sec += 60
        reason = 'source stamp is in the future'
    elif problem == 'stale':
        message.header.stamp.sec -= 60
        reason = 'source observation is too old'
    else:
        message.header.frame_id = ''
        reason = 'missing map frame_id'
    planner.map_publisher.publish(message)
    planner.wait_for(lambda: reason in planner.logs(), 'early source validation')

    result = planner.query_geometry()

    assert not result.success and result.error_code == result.MAP_UNAVAILABLE
    assert reason in result.message
    assert result.map_header == message.header
    assert 'source_age=' in result.message
    assert 'receipt_silence=' in result.message
    assert 'bytes=0, nodes=0' in planner.logs()


def test_unstamped_legacy_map_remains_available_only_for_ordinary_planning(planner_factory):
    planner = planner_factory(cached_map=True)
    planner.publish_odometry(0.0)
    planner.wait_for_response(
        lambda result: not result.message.startswith('Missing inputs:'), 'legacy map receipt')
    planner.publish_odometry(2.0)
    planner.wait_for_response(lambda result: result.success, 'legacy exploration planning')

    strict = planner.query_geometry()

    assert not strict.success and strict.error_code == strict.MAP_UNAVAILABLE
    assert 'zero or invalid source stamp' in strict.message
    assert 'source_age=unavailable' in strict.message


@pytest.mark.parametrize('replacement', ['older', 'duplicate', 'unstamped', 'stale', 'future'])
def test_rejected_maps_do_not_replace_fresh_snapshot_or_add_occupied_history(
        planner_factory, replacement):
    planner = planner_factory()
    accepted = _publish_inspection_map(planner, wall=False)
    rejected = deepcopy(accepted)
    rejected.data = _inspection_scene(wall=True)
    if replacement == 'older':
        rejected.header.stamp.sec -= 1
        reason = 'duplicate or out-of-order source stamp'
    elif replacement == 'unstamped':
        rejected.header.stamp.sec = rejected.header.stamp.nanosec = 0
        reason = 'zero source stamp cannot replace a stamped map'
    elif replacement == 'stale':
        rejected.header.stamp.sec -= 60
        reason = 'source observation is too old'
    elif replacement == 'future':
        rejected.header.stamp.sec += 60
        reason = 'source stamp is in the future'
    else:
        reason = 'duplicate or out-of-order source stamp'
    planner.map_publisher.publish(rejected)
    planner.wait_for(lambda: reason in planner.logs(), 'replacement rejection')
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]

    result = planner.query_geometry(request)

    assert result.success and result.corridor_safe, result.message
    assert result.map_header == accepted.header
    assert 'decode_elapsed=' in planner.logs()
    assert 'source_age_before=' in planner.logs() and 'source_age_after=' in planner.logs()


def _set_planner_clock(planner, publisher, seconds):
    clock = Clock()
    clock.clock.sec = seconds
    # The service reply provides a deterministic clock-observation barrier.

    def observed():
        publisher.publish(clock)
        return f'ros_now={seconds}' in planner.query_geometry().message
    planner.wait_for(observed, 'planner ROS clock observation')


def test_clock_watchdog_expires_while_simulation_clock_is_paused(planner_factory):
    planner = planner_factory(planner_parameters={
        'use_sim_time': True, 'inspection_map_timeout': 0.5,
    })
    publisher = planner.node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    _set_planner_clock(planner, publisher, 100)
    message = Octomap(binary=True, id='OcTree', resolution=0.25, data=[0x55, 0x55])
    message.header.frame_id = 'planner_test_map'
    message.header.stamp.sec = 100
    planner.map_publisher.publish(message)
    planner.wait_for(lambda: planner.query_geometry().success, 'fresh paused-clock map')
    time.sleep(0.3)
    planner.map_publisher.publish(message)
    planner.wait_for(
        lambda: 'duplicate or out-of-order source stamp' in planner.logs(),
        'duplicate map rejection',
    )
    time.sleep(0.3)

    result = planner.query_geometry()

    assert not result.success and result.error_code == result.MAP_UNAVAILABLE
    assert 'simulation clock has stopped advancing' in result.message
    assert 'source_age=0 s' in result.message and 'timeout=0.5 s' in result.message
    # Duplicate traffic cannot reset the clock watchdog, which uses the
    # configured 0.5 s deadline when ROS time does not advance.
    planner.wait_for(
        lambda: 'OctoMap unavailable for inspection on' in planner.logs(),
        'configured reception watchdog', timeout=2.0,
    )


def test_advancing_slow_simulation_uses_source_age_instead_of_wall_receipt_age(planner_factory):
    planner = planner_factory(planner_parameters={
        'use_sim_time': True, 'inspection_map_timeout': 0.5,
    })
    publisher = planner.node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    _set_planner_clock(planner, publisher, 100)
    message = Octomap(binary=True, id='OcTree', resolution=0.25, data=[0x55, 0x55])
    message.header.frame_id = 'planner_test_map'
    message.header.stamp.sec = 100
    planner.map_publisher.publish(message)
    planner.wait_for(lambda: planner.query_geometry().success, 'initial simulation map')
    started = time.monotonic()
    logs_before = planner.logs()
    for step in range(1, 11):
        clock = Clock()
        clock.clock.sec = 100
        clock.clock.nanosec = step * 30_000_000
        publisher.publish(clock)
        time.sleep(0.12)
        result = planner.query_geometry()
        assert result.success, result.message
        assert result.map_header == message.header
    assert time.monotonic() - started > 1.0  # Exercise the wall watchdog as well.
    assert 'OctoMap unavailable for inspection' not in planner.logs()[len(logs_before):]

    clock.clock.nanosec = 600_000_000
    publisher.publish(clock)
    planner.wait_for(lambda: not planner.query_geometry().success, 'source-age deadline')
    result = planner.query_geometry()
    assert result.error_code == result.MAP_UNAVAILABLE
    assert 'source observation is too old' in result.message
    assert 'source_age=0.6' in result.message


def test_clock_restart_accepts_new_epoch_without_old_occupied_evidence(planner_factory):
    planner = planner_factory(planner_parameters={'use_sim_time': True})
    publisher = planner.node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    _set_planner_clock(planner, publisher, 100)
    message = Octomap(binary=True, id='OcTree', resolution=0.25)
    message.header.frame_id = 'planner_test_map'
    message.header.stamp.sec = 100
    message.data = _inspection_scene(wall=True)
    planner.map_publisher.publish(message)
    planner.wait_for(lambda: planner.query_geometry().success, 'old-epoch occupied map')
    _set_planner_clock(planner, publisher, 10)
    message.header.stamp.sec = 10
    message.data = _inspection_scene(wall=False)
    planner.map_publisher.publish(message)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]
    planner.wait_for(
        lambda: planner.query_geometry().map_header.stamp == message.header.stamp,
        'new clock epoch map',
    )

    result = planner.query_geometry(request)

    assert result.success and result.corridor_safe, result.message
    assert result.map_header.stamp.sec == 10


def test_clock_pause_does_not_erase_recent_occupied_history_on_resume(planner_factory):
    planner = planner_factory(planner_parameters={
        'use_sim_time': True, 'inspection_map_timeout': 0.5,
        'inspection_require_observed_free_space': None,
    })
    publisher = planner.node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    _set_planner_clock(planner, publisher, 100)
    message = Octomap(binary=True, id='OcTree', resolution=0.25)
    message.header.frame_id = 'planner_test_map'
    message.header.stamp.sec = 100
    message.data = _inspection_scene(wall=True)
    planner.map_publisher.publish(message)
    planner.wait_for(lambda: planner.query_geometry().success, 'occupied map')
    message.header.stamp.nanosec = 100_000_000
    message.data = _inspection_scene(wall=False)
    planner.map_publisher.publish(message)
    planner.wait_for(
        lambda: planner.query_geometry().map_header.stamp == message.header.stamp,
        'newer free map')
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]
    before = planner.query_geometry(request)
    assert before.success and not before.corridor_safe
    assert 'occupied cell in recent map history' in before.message
    planner.wait_for(
        lambda: 'simulation clock has stopped advancing' in planner.logs(),
        'wall watchdog during paused simulation', timeout=2.5)
    assert not planner.query_geometry().success

    resumed = Clock()
    resumed.clock.sec = 100
    resumed.clock.nanosec = 200_000_000
    publisher.publish(resumed)
    planner.wait_for(lambda: planner.query_geometry().success, 'resumed simulation')
    result = planner.query_geometry(request)

    assert result.success and not result.corridor_safe, result.message
    assert 'occupied cell in recent map history' in result.message


def test_inspection_ray_reports_measured_surface_and_temporal_inconsistency(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    _publish_inspection_map(planner, wall=True)

    result = planner.query_geometry(_inspection_request())

    assert result.success and result.error_code == result.OK
    assert result.surface_valid
    assert result.surface_range == pytest.approx(3.0, abs=0.25)
    assert 0.0 < result.surface_uncertainty < 0.3
    assert result.surface.header.frame_id == 'planner_test_map'
    assert result.occupied_samples == result.free_samples == 1
    assert result.plume_candidate


def test_inspection_unknown_space_has_no_ray_hit_and_blocks_corridor(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner)
    request = _inspection_request()
    request.origin = _geometry_point(x=-4.0)
    request.origin.header.stamp = planner.node.get_clock().now().to_msg()
    request.direction.header.stamp = request.origin.header.stamp
    request.max_range = 8.0
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=-4.0)]

    result = planner.query_geometry(request)

    assert result.success
    assert not result.surface_valid
    assert not result.corridor_safe
    assert 'unknown cell in the latest map' in result.message
    assert 'path sample=(' in result.message
    assert 'blocking voxel=(' in result.message
    assert 'clearance radius=0.2 m' in result.message
    assert result.ray_origin.header.frame_id == 'planner_test_map'
    assert result.ray_origin.header.stamp == request.origin.header.stamp
    assert result.ray_origin.point.x == pytest.approx(-4.0)
    assert result.ray_direction.header.stamp == request.direction.header.stamp
    assert result.ray_direction.vector.x == pytest.approx(1.0)


def test_inspection_latest_occupied_voxel_identifies_the_blocking_map(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=True)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]

    result = planner.query_geometry(request)

    assert result.success and not result.corridor_safe
    assert 'occupied cell in the latest map' in result.message
    assert 'recent map history' not in result.message
    assert 'path sample=(' in result.message
    assert 'blocking voxel=(3.125,' in result.message
    assert 'clearance radius=0.2 m' in result.message


def test_inspection_recent_occupied_voxels_remain_blocking_after_free_flicker(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=True)
    _publish_inspection_map(planner, wall=False)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]

    result = planner.query_geometry(request)

    assert result.success and not result.corridor_safe
    assert 'occupied cell in recent map history' in result.message
    assert 'path sample=(' in result.message
    assert 'blocking voxel=(3.125,' in result.message
    assert 'clearance radius=0.2 m' in result.message


def test_expired_occupied_history_does_not_block_a_new_observed_free_map(planner_factory):
    planner = planner_factory(planner_parameters={'inspection_map_timeout': 0.5})
    _publish_inspection_map(planner, wall=True)
    time.sleep(0.6)
    latest = _publish_inspection_map(planner, wall=False)
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]

    result = planner.query_geometry(request)

    assert result.success and result.corridor_safe, result.message
    assert result.map_header == latest.header


def test_inspection_ground_needs_stable_patch_and_agl_clearance_along_entire_path(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    request = QueryInspectionGeometry.Request()
    request.ground_query = _geometry_point(z=5.125)
    assert not planner.query_geometry(request).ground_valid
    _publish_inspection_map(planner, wall=False)

    result = planner.query_geometry(request)

    assert result.success and result.ground_valid
    assert result.ground_surface.point.z == pytest.approx(-0.125, abs=0.13)
    assert 0.0 < result.ground_uncertainty <= 0.25
    request.minimum_agl = 5.0
    request.corridor.poses = [planner.target(x=0.125, z=5.125),
                              planner.target(x=1.125, z=5.125)]
    safe = planner.query_geometry(request)
    assert safe.success and safe.corridor_safe, safe.message
    request.corridor.poses.insert(1, planner.target(x=0.625, z=4.5))
    low = planner.query_geometry(request)
    assert low.success and not low.corridor_safe
    assert 'support-surface clearance' in low.message


def test_inspection_checks_segment_standoff_and_distinguishes_input_errors(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    request = QueryInspectionGeometry.Request()
    request.keepout_center = _geometry_point()
    request.minimum_radius = 0.8
    request.corridor.poses = [planner.target(x=0.125, y=1.125),
                              planner.target(x=0.125, y=-1.125)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe
    assert 'standoff radius' in result.message
    invalid = _inspection_request()
    invalid.direction.vector.x = 0.0
    result = planner.query_geometry(invalid)
    assert not result.success and result.error_code == result.INVALID_REQUEST
    invalid = _inspection_request()
    invalid.direction.header.frame_id = 'missing_camera_frame'
    result = planner.query_geometry(invalid)
    assert not result.success and result.error_code == result.TRANSFORM_ERROR


@pytest.mark.parametrize('x, safe', [
    (math.nextafter(1.125, -math.inf), True),
    (1.025, False),
])
def test_inspection_radius_accepts_boundary_roundoff_but_rejects_real_intrusion(
        planner_factory, x, safe):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    request = QueryInspectionGeometry.Request()
    request.keepout_center = _geometry_point()
    request.minimum_radius = 1.0
    request.corridor.poses = [planner.target(x=x, y=0.125)]

    result = planner.query_geometry(request)

    assert result.success
    assert result.corridor_safe is safe
    if not safe:
        assert 'standoff radius' in result.message
        assert 'actual=' in result.message and 'requested=1' in result.message
        assert 'point=(' in result.message


def _strict_planner(planner_factory, *, wall=False, planner_parameters=None):
    planner = planner_factory(
        home_from_first_odometry=False,
        planner_parameters={'planning_timeout': 2.0, **(planner_parameters or {})},
    )
    _publish_inspection_map(planner, wall=wall)
    _publish_inspection_map(planner, wall=wall)
    planner.publish_odometry(2.0)
    planner.wait_for(lambda: planner.home_publisher.get_subscription_count() > 0,
                     'explicit home subscription')
    planner.publish_home()
    planner.wait_for_response(lambda result: result.success, 'strict planner input readiness')
    return planner


class AdaptiveViewpointProbe(InspectionNavigationMixin):
    """Use real native candidate planning; simulate only arrival after a safe plan."""

    def __init__(self, planner):
        self.planner = planner
        self.pose = deepcopy(planner.odometry_message.pose.pose)
        self.config = {'verify_inspection_lateral_step': 5.0, 'position_tolerance': 0.3}
        self.requests = []
        self.messages = []
        self.paths = []

    def _verify_control_pose(self):
        return None, deepcopy(self.pose)

    def _check_running(self):
        pass

    def _report(self, message):
        self.messages.append(message)

    def _verify_recovery_plan_to(self, target, geometry=None, guard=None):
        if guard:
            guard()
        request = PlanInspectionPath.Request(target=self.planner.target())
        request.target.pose = deepcopy(target)
        result = self.planner.plan_inspection(request)
        self.requests.append((request, result))
        if result.success:
            checked = self.planner.query_geometry(QueryInspectionGeometry.Request(
                corridor=result.path))
            assert checked.success and checked.corridor_safe, checked.message
            self.pose = deepcopy(result.path.poses[-1].pose)
            self.paths.append(deepcopy(result.path))
        return result.success


@pytest.mark.parametrize('radius', [0.2, 0.75])
def test_localization_shortens_unknown_goal_using_the_real_strict_planner(
        planner_factory, radius):
    planner = _strict_planner(
        planner_factory, planner_parameters={'drone_radius': radius - 0.1})
    probe = AdaptiveViewpointProbe(planner)

    target = deepcopy(probe.pose)
    target.position.y += 5.0
    assert probe._verify_adaptive_localization_target(target)

    requests = [request.target.pose.position.y for request, _ in probe.requests]
    assert requests == ([5.0, 2.5, 1.25] if radius == 0.2 else [5.0, 2.5, 1.25, 0.625])
    for _, response in probe.requests[:-1]:
        assert not response.success
        assert 'unknown cell in the latest map' in response.message
        assert not response.path.poses
    assert probe.pose.position.y == requests[-1]
    assert len(probe.paths) == 1
    assert not probe._verify_source_at_standoff(None)


def test_localization_shorter_native_candidates_never_waive_an_occupied_barrier(planner_factory):
    planner = _strict_planner(planner_factory, wall='lateral')
    probe = AdaptiveViewpointProbe(planner)
    original = deepcopy(probe.pose)

    target = deepcopy(probe.pose)
    target.position.y += 5.0
    assert not probe._verify_adaptive_localization_target(target)

    assert [request.target.pose.position.y for request, _ in probe.requests] == [
        5.0, 2.5, 1.25, 0.625]
    assert all(not response.success and not response.path.poses for _, response in probe.requests)
    assert any('occupied cell in the latest map' in response.message
               for _, response in probe.requests)
    assert probe.pose == original and not probe.paths


def test_inspection_planner_searches_an_observed_free_detour(planner_factory):
    planner = _strict_planner(planner_factory, wall='pillar')
    request = PlanInspectionPath.Request(target=planner.target(x=5.125))
    direct = QueryInspectionGeometry.Request()
    direct.corridor.poses = [planner.target(), request.target]
    assert not planner.query_geometry(direct).corridor_safe

    result = planner.plan_inspection(request)

    assert result.success, result.message
    assert len(result.path.poses) >= 3
    checked = QueryInspectionGeometry.Request(corridor=result.path)
    assert planner.query_geometry(checked).corridor_safe
    end = result.path.poses[-1].pose.position
    assert (end.x, end.y, end.z) == pytest.approx((5.125, 0.0, 2.0))


def test_inspection_planner_rejects_unknown_start_without_a_waiver(planner_factory):
    planner = _strict_planner(planner_factory)
    planner.odometry_message.pose.pose.position.x = -4.0
    planner.repeat_odometry()
    planner.wait_for_response(
        lambda result: result.success, 'ordinary planner unknown-space input')
    request = PlanInspectionPath.Request(target=planner.target(x=-3.5))

    result = planner.plan_inspection(request)

    assert not result.success
    assert result.error_code == result.START_INVALID
    assert 'unknown cell in the latest map' in result.message


def test_inspection_planner_climbs_vertically_before_enforcing_full_agl(planner_factory):
    planner = _strict_planner(planner_factory)
    request = PlanInspectionPath.Request(
        target=planner.target(x=1.0, z=5.2), minimum_agl=5.0, allow_initial_climb=True,
    )

    result = planner.plan_inspection(request)

    assert result.success, result.message
    assert len(result.path.poses) >= 3
    start, raised = (vertex.pose.position for vertex in result.path.poses[:2])
    assert (start.x, start.y, start.z) == pytest.approx((0.0, 0.0, 2.0))
    assert (raised.x, raised.y) == pytest.approx((start.x, start.y))
    assert raised.z >= 5.0
    checked = QueryInspectionGeometry.Request(
        corridor=result.path, minimum_agl=5.0, allow_initial_climb=True,
    )
    assert planner.query_geometry(checked).corridor_safe
    checked.allow_initial_climb = False
    assert not planner.query_geometry(checked).corridor_safe


@pytest.mark.parametrize('middle', [
    (0.3, 0.0, 3.0),  # Lateral movement while below clearance.
    (0.0, 0.0, 1.9),  # A descent before the climb.
])
def test_inspection_climb_exception_rejects_non_upward_prefixes(planner_factory, middle):
    planner = _strict_planner(planner_factory)
    request = QueryInspectionGeometry.Request(minimum_agl=5.0, allow_initial_climb=True)
    request.corridor.poses = [planner.target(z=2.0),
                              planner.target(x=middle[0], y=middle[1], z=middle[2]),
                              planner.target(z=5.2)]

    result = planner.query_geometry(request)

    assert result.success and not result.corridor_safe
    assert 'only a fixed-horizontal-position upward initial climb' in result.message


def test_inspection_radius_uses_the_supplied_horizontal_plane(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    request = QueryInspectionGeometry.Request(minimum_radius=1.0)
    request.keepout_center = _geometry_point(z=2.0)
    request.keepout_axis.header.frame_id = 'planner_test_map'
    request.keepout_axis.vector.x = 1.0
    request.corridor.poses = [planner.target(x=4.125, y=1.125)]
    assert planner.query_geometry(request).corridor_safe
    # The 4 m map-X separation is vertical in this requested constraint plane.
    request.corridor.poses = [planner.target(x=4.125, y=0.125)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe
    assert 'standoff radius' in result.message


def test_inspection_revalidation_allows_only_bounded_upward_tracking_drift(planner_factory):
    planner = _strict_planner(planner_factory)
    request = QueryInspectionGeometry.Request(minimum_agl=5.0, allow_initial_climb=True)
    request.corridor.poses = [planner.target(z=2.0), planner.target(x=0.05, z=3.0),
                              planner.target(x=0.05, z=5.2)]
    assert not planner.query_geometry(request).corridor_safe
    request.initial_climb_tracking_tolerance = 0.1
    bounded = planner.query_geometry(request)
    assert bounded.success and bounded.corridor_safe, bounded.message
    request.corridor.poses[1] = planner.target(x=0.15, z=3.0)
    assert not planner.query_geometry(request).corridor_safe
    request.initial_climb_tracking_tolerance = 0.11
    invalid = planner.query_geometry(request)
    assert not invalid.success and invalid.error_code == invalid.INVALID_REQUEST


def test_inspection_planner_distinguishes_transform_and_input_failures(planner_factory):
    planner = _strict_planner(planner_factory)
    request = PlanInspectionPath.Request(target=planner.target(frame='missing_target_frame'))
    transformed = planner.plan_inspection(request)
    assert not transformed.success and transformed.error_code == transformed.TRANSFORM_ERROR
    request.target = planner.target()
    request.minimum_agl = -1.0
    invalid = planner.plan_inspection(request)
    assert not invalid.success and invalid.error_code == invalid.INVALID_REQUEST


@lru_cache(maxsize=6)
def _terrain_scene(*, tree=False, unknown_air=False, unknown_column=False, roof=False,
                   offset_roof=False, sealed_roof=False, ground_holes=False,
                   ground_absent=False, terrain_spike=False, coarse_terrain=False):
    """Encode a rising voxel hill and optional tree/unknown volume for native planning."""
    root = {}
    for x in range(-8, 36):
        for y in range(-12, 12):
            floor = -1 if roof or sealed_roof or offset_roof else max(-1, min(7, x // 2 - 1))
            for z in range(-1, 49):
                if (unknown_air and z >= 32) or (unknown_column and z == 12):
                    continue
                if z <= floor and (ground_absent or (ground_holes and x % 2 == y % 2 == 0)):
                    continue
                keys = (x + 32768, y + 32768, z + 32768)
                node = root
                for bit in range(15, -1, -1):
                    child = sum(((key >> bit) & 1) << axis
                                for axis, key in enumerate(keys))
                    if bit:
                        node = node.setdefault(child, {})
                    else:
                        canopy = tree and 12 <= x <= 15 and -2 <= y <= 2 and z <= 25
                        low_roof = roof and -1 <= x <= 1 and -1 <= y <= 1 and z == 12
                        # The safe climb aperture is between the eight radial
                        # directions tried by simple lateral-to-climb prefixes.
                        aperture = 3 <= x < 10 and 0 <= y < 5
                        shifted_roof = offset_roof and not aperture and z == 12
                        closed_roof = sealed_roof and z == 12
                        spike = terrain_spike and x == y == 0 and z == 8
                        coarse = coarse_terrain and 4 <= x < 8 and 0 <= y < 4 and 8 <= z < 12
                        occupied = (z <= floor or canopy or low_roof or spike or coarse
                                    or shifted_roof or closed_roof)
                        node[child] = 2 if occupied else 1

    def compress(node):
        values = {index: compress(value) if isinstance(value, dict) else value
                  for index, value in node.items()}
        if len(values) == 8 and all(value == 1 for value in values.values()):
            return 1
        if len(values) == 8 and all(value == 2 for value in values.values()):
            return 2
        return values

    data = []

    def encode(node):
        bits = sum((3 if isinstance(value, dict) else value) << (2 * index)
                   for index, value in node.items())
        for value in (bits & 255, bits >> 8):
            data.append(value if value < 128 else value - 256)
        for _, value in sorted(node.items()):
            if isinstance(value, dict):
                encode(value)

    encode(compress(root))
    return tuple(data)


def _publish_terrain_scene(planner, **scene):
    message = Octomap()
    message.header.frame_id = 'planner_test_map'
    message.header.stamp = planner.node.get_clock().now().to_msg()
    message.binary, message.id, message.resolution = True, 'OcTree', 0.25
    message.data = _terrain_scene(**scene)
    planner.map_publisher.publish(message)
    planner.wait_for(
        lambda: planner.query_geometry().map_header.stamp == message.header.stamp,
        'terrain map reception')


def _terrain_planner(planner_factory, *, planner_parameters=None, **scene):
    planner = planner_factory(
        home_from_first_odometry=False,
        planner_parameters={'maximum_altitude': 15.0, 'horizontal_range': 3.0,
                            'planning_timeout': 2.0, 'inspection_map_timeout': 15.0,
                            **(planner_parameters or {})},
    )
    for _ in range(2):
        _publish_terrain_scene(planner, **scene)
    planner.publish_odometry(2.0 if any(scene.get(key) for key in (
        'roof', 'offset_roof', 'sealed_roof')) else 6.0)
    planner.wait_for(lambda: planner.home_publisher.get_subscription_count() > 0,
                     'terrain home subscription')
    planner.publish_home()
    planner.wait_for_response(lambda result: result.success, 'terrain planner readiness')
    return planner


def test_terrain_envelope_accepts_slopes_that_are_not_flat_support(planner_factory):
    planner = _terrain_planner(planner_factory)
    request = QueryInspectionGeometry.Request(ground_query=_geometry_point(x=2.125, z=7.5))
    assert not planner.query_geometry(request).ground_valid

    request.terrain_envelope = True
    result = planner.query_geometry(request)

    assert result.success and result.ground_valid
    assert result.ground_surface.point.z >= 0.75
    # Occupied interior leaves may merge to 1 m on the voxel hill; retain
    # their half-leaf uncertainty rather than pretending the surface is exact.
    assert 0.0 < result.ground_uncertainty <= 0.5


def test_terrain_height_estimate_does_not_certify_unknown_air_as_free(planner_factory):
    planner = _terrain_planner(planner_factory, unknown_air=True)
    request = QueryInspectionGeometry.Request(
        ground_query=_geometry_point(x=2.125, z=10.0), terrain_envelope=True,
        corridor=PlanInspectionPath.Response().path,
    )
    request.corridor.poses = [planner.target(x=2.125, z=10.0)]
    estimate = planner.query_geometry(request)
    assert estimate.success and estimate.ground_valid
    assert not estimate.corridor_safe
    assert not estimate.current_clearance_reached
    assert 'unknown cell' in estimate.message

    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=2.125, z=10.0), minimum_agl=5.0, terrain_envelope=True,
    ))
    assert not result.success and result.error_code == result.GOAL_INVALID
    assert not result.path.poses


def test_terrain_height_estimate_does_not_waive_unknown_downward_clearance(planner_factory):
    planner = _terrain_planner(planner_factory, unknown_column=True)
    request = QueryInspectionGeometry.Request(
        ground_query=_geometry_point(z=6.0), terrain_envelope=True,
    )
    estimate = planner.query_geometry(request)
    assert estimate.success and estimate.ground_valid
    request.minimum_agl = 5.0
    request.corridor.poses = [planner.target(z=6.0)]
    assert not planner.query_geometry(request).corridor_safe

    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=2.0, z=8.0), minimum_agl=5.0,
        terrain_envelope=True, allow_initial_climb=True,
    ))
    assert not result.success and result.error_code == result.START_INVALID


@pytest.mark.parametrize('scene', ['unknown_air', 'unknown_column'])
def test_default_lidar_policy_plans_through_sparse_air_above_observed_terrain(
        planner_factory, scene):
    planner = _terrain_planner(
        planner_factory,
        planner_parameters={'inspection_require_observed_free_space': None},
        **{scene: True},
    )
    target = planner.target(x=2.125, z=10.0)
    support = planner.query_geometry(QueryInspectionGeometry.Request(
        ground_query=_geometry_point(z=10.0)))
    assert support.success and support.ground_valid, support.message
    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=target, minimum_agl=5.0, terrain_envelope=True,
    ))

    assert result.success, result.message
    assert result.path.poses[-1].pose.position == target.pose.position
    checked = planner.query_geometry(QueryInspectionGeometry.Request(
        corridor=result.path, minimum_agl=5.0, terrain_envelope=True))
    assert checked.success and checked.corridor_safe, checked.message
    assert 'avoids current and recent occupied voxels' in checked.message
    assert 'permit unknown air' in planner.logs()


@pytest.mark.parametrize('ground_holes,ground_absent', [(True, False), (False, True)])
def test_default_lidar_policy_advances_without_dense_ground_returns(
        planner_factory, ground_holes, ground_absent):
    planner = _terrain_planner(
        planner_factory, ground_holes=ground_holes, ground_absent=ground_absent,
        planner_parameters={'inspection_require_observed_free_space': None})
    estimate = planner.query_geometry(QueryInspectionGeometry.Request(
        ground_query=_geometry_point(z=6.0), terrain_envelope=True))
    assert estimate.success
    # A sparse real surface may select a height; no surface must never invent one.
    assert estimate.ground_valid is ground_holes
    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=0.75, z=6.0), minimum_agl=5.0,
        terrain_envelope=True, allow_initial_climb=True, allow_terrain_escape=True))
    assert result.success, result.message
    check = QueryInspectionGeometry.Request(
        corridor=result.path, minimum_agl=5.0, terrain_envelope=True,
        allow_initial_climb=True, allow_terrain_escape=True,
        escape_origin=_geometry_point(z=6.0))
    validated = planner.query_geometry(check)
    assert validated.success and validated.corridor_safe, validated.message
    assert validated.current_clearance_reached


def test_strict_policy_still_requires_ground_returns(planner_factory):
    planner = _terrain_planner(planner_factory, ground_absent=True)
    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=0.75, z=6.0), minimum_agl=5.0,
        terrain_envelope=True, allow_initial_climb=True))
    assert not result.success and result.error_code == result.START_INVALID
    assert 'observed stable support' in result.message


@pytest.mark.parametrize('history_only', [False, True])
def test_sparse_terrain_checks_include_new_and_recent_occupied_voxels(
        planner_factory, history_only):
    planner = _terrain_planner(
        planner_factory, ground_absent=True,
        planner_parameters={'inspection_require_observed_free_space': None})
    # A single new terrain hit is enough to block, even without repeated evidence
    # and even after the latest map calls that same cell free again.
    _publish_terrain_scene(planner, ground_absent=True, terrain_spike=True)
    if history_only:
        _publish_terrain_scene(planner, ground_absent=True)
    request = QueryInspectionGeometry.Request(
        ground_query=_geometry_point(z=6.0), minimum_agl=5.0, terrain_envelope=True)
    request.corridor.poses = [planner.target(z=6.0)]
    result = planner.query_geometry(request)
    assert result.success and result.ground_valid
    assert result.ground_surface.point.z == pytest.approx(2.25)
    assert not result.corridor_safe and not result.current_clearance_reached
    assert 'support-surface clearance' in result.message
    planned = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=0.4, z=6.0), minimum_agl=5.0, terrain_envelope=True))
    assert not planned.success and planned.error_code == planned.START_INVALID


def test_sparse_terrain_footprint_includes_overlapping_coarse_voxel_edges(planner_factory):
    planner = _terrain_planner(
        planner_factory, ground_absent=True, coarse_terrain=True,
        planner_parameters={'inspection_require_observed_free_space': None})
    # The merged 1 m leaf center x=1.5 is outside the 0.5 m footprint at x=0.6;
    # its near face x=1.0 still overlaps, so center-only tests would miss terrain.
    request = QueryInspectionGeometry.Request(
        ground_query=_geometry_point(x=0.6, y=0.5, z=6.0),
        minimum_agl=5.0, terrain_envelope=True)
    request.corridor.poses = [planner.target(x=0.6, y=0.5, z=6.0)]
    result = planner.query_geometry(request)
    assert result.success and result.ground_valid
    assert result.ground_surface.point.z == pytest.approx(3.0)
    assert result.ground_uncertainty == pytest.approx(0.5)
    assert not result.corridor_safe


def test_missing_ground_does_not_certify_clearance_beyond_checked_range(planner_factory):
    planner = _terrain_planner(
        planner_factory, ground_absent=True,
        planner_parameters={'inspection_require_observed_free_space': None,
                            'inspection_max_range': 4.0})
    request = QueryInspectionGeometry.Request(minimum_agl=5.0, terrain_envelope=True)
    request.corridor.poses = [planner.target(z=6.0)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe


def test_default_lidar_policy_retains_known_obstacles_and_terrain_floor(planner_factory):
    planner = _terrain_planner(
        planner_factory, tree=True, unknown_air=True,
        planner_parameters={'inspection_require_observed_free_space': None})
    obstacle = QueryInspectionGeometry.Request()
    obstacle.corridor.poses = [planner.target(x=3.125, z=6.0)]
    blocked = planner.query_geometry(obstacle)
    assert blocked.success and not blocked.corridor_safe
    assert 'occupied cell in the latest map' in blocked.message

    low = QueryInspectionGeometry.Request(minimum_agl=8.0, terrain_envelope=True)
    low.corridor.poses = [planner.target(x=0.125, z=6.0)]
    blocked = planner.query_geometry(low)
    assert blocked.success and not blocked.corridor_safe
    assert 'support-surface clearance' in blocked.message


def test_default_lidar_policy_keeps_recent_occupied_cells_when_latest_is_unknown(planner_factory):
    planner = planner_factory(
        planner_parameters={'inspection_require_observed_free_space': None})
    _publish_inspection_map(planner, wall=True)
    _publish_inspection_map(planner, wall='unknown_wall')
    request = QueryInspectionGeometry.Request()
    request.corridor.poses = [planner.target(x=0.125), planner.target(x=5.125)]

    result = planner.query_geometry(request)

    assert result.success and not result.corridor_safe
    assert 'occupied cell in recent map history' in result.message


def test_terrain_planner_finds_bent_route_around_tree_above_rising_hill(planner_factory):
    planner = _terrain_planner(planner_factory, tree=True)
    request = PlanInspectionPath.Request(
        target=planner.target(x=6.0, z=9.8), minimum_agl=5.0, terrain_envelope=True,
    )
    direct = QueryInspectionGeometry.Request(minimum_agl=5.0, terrain_envelope=True)
    direct.corridor.poses = [planner.target(z=6.0), request.target]
    assert not planner.query_geometry(direct).corridor_safe

    result = planner.plan_inspection(request)

    assert result.success, result.message
    assert len(result.path.poses) >= 3
    checked = planner.query_geometry(QueryInspectionGeometry.Request(
        corridor=result.path, minimum_agl=5.0, terrain_envelope=True))
    assert checked.success and checked.corridor_safe, checked.message
    endpoint = result.path.poses[-1].pose.position
    assert (endpoint.x, endpoint.y, endpoint.z) == pytest.approx((6.0, 0.0, 9.8))


def test_terrain_escape_moves_bounded_sideways_then_climbs_when_vertical_is_blocked(
        planner_factory):
    planner = _terrain_planner(planner_factory, roof=True)
    request = PlanInspectionPath.Request(
        target=planner.target(x=3.0, z=5.5), minimum_agl=5.0,
        terrain_envelope=True, allow_initial_climb=True,
    )
    vertical = planner.plan_inspection(request)
    assert not vertical.success and vertical.error_code == vertical.NO_CLEARANCE_ESCAPE
    request.allow_terrain_escape = True

    result = planner.plan_inspection(request)

    assert result.success, result.message
    start, side, raised = (vertex.pose.position for vertex in result.path.poses[:3])
    assert (start.x, start.y, start.z) == pytest.approx((0.0, 0.0, 2.0))
    assert 0.5 <= math.hypot(side.x, side.y) <= 2.0 + 1e-6
    assert side.z == pytest.approx(start.z)
    assert (raised.x, raised.y) == pytest.approx((side.x, side.y))
    assert raised.z >= 5.0
    checked = QueryInspectionGeometry.Request(
        corridor=result.path, minimum_agl=5.0, terrain_envelope=True,
        allow_initial_climb=True, allow_terrain_escape=True,
    )
    assert planner.query_geometry(checked).corridor_safe
    checked.allow_terrain_escape = False
    assert not planner.query_geometry(checked).corridor_safe


def test_terrain_escape_finds_multisegment_connector_between_radial_prefixes(planner_factory):
    planner = _terrain_planner(planner_factory, offset_roof=True)
    request = PlanInspectionPath.Request(
        target=planner.target(x=1.5, y=0.5, z=5.5), minimum_agl=5.0,
        terrain_envelope=True, allow_initial_climb=True,
    )
    vertical = planner.plan_inspection(request)
    assert not vertical.success and vertical.error_code == vertical.NO_CLEARANCE_ESCAPE
    request.allow_terrain_escape = True

    result = planner.plan_inspection(request)

    assert result.success, result.message
    points = [vertex.pose.position for vertex in result.path.poses]
    assert len(points) >= 4
    assert any(abs(b.x - a.x) > 0.1 and abs(b.y - a.y) > 0.1
               for a, b in zip(points, points[1:]))
    checked = planner.query_geometry(QueryInspectionGeometry.Request(
        corridor=result.path, minimum_agl=5.0, terrain_envelope=True,
        allow_initial_climb=True, allow_terrain_escape=True,
        escape_origin=_geometry_point(z=2.0),
    ))
    assert checked.success and checked.corridor_safe, checked.message
    assert (points[-1].x, points[-1].y, points[-1].z) == pytest.approx((1.5, 0.5, 5.5))


def test_terrain_escape_exhaustion_is_distinct_from_unknown_current_body(planner_factory):
    planner = _terrain_planner(planner_factory, sealed_roof=True)
    body_request = QueryInspectionGeometry.Request()
    body_request.corridor.poses = [planner.target(z=2.0)]
    assert planner.query_geometry(body_request).corridor_safe

    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=3.0, z=5.5), minimum_agl=5.0,
        terrain_envelope=True, allow_initial_climb=True, allow_terrain_escape=True,
    ))

    assert not result.success and result.error_code == result.NO_CLEARANCE_ESCAPE
    assert 'direct climb:' in result.message
    assert 'multi-segment escape exhausted' in result.message
    assert not result.path.poses


def test_terrain_escape_validates_ascending_diagonal_and_contour_legs(planner_factory):
    planner = _terrain_planner(planner_factory, offset_roof=True)
    request = QueryInspectionGeometry.Request(
        minimum_agl=5.0, terrain_envelope=True, allow_terrain_escape=True,
        escape_origin=_geometry_point(z=2.0),
    )
    request.corridor.poses = [
        planner.target(z=2.0), planner.target(x=0.5, y=0.5, z=2.5),
        planner.target(x=1.5, y=0.5, z=2.5), planner.target(x=1.5, y=0.5, z=5.5),
    ]

    result = planner.query_geometry(request)

    assert result.success and result.corridor_safe, result.message
    # The same trajectory must not become a low-clearance descent waiver.
    request.corridor.poses[2].pose.position.z = 2.0
    rejected = planner.query_geometry(request)
    assert rejected.success and not rejected.corridor_safe
    assert 'never decrease' in rejected.message


def test_terrain_escape_cannot_trade_clearance_for_lateral_progress(planner_factory):
    planner = _terrain_planner(planner_factory)
    request = QueryInspectionGeometry.Request(
        minimum_agl=5.0, terrain_envelope=True, allow_terrain_escape=True,
    )
    request.corridor.poses = [planner.target(z=2.0), planner.target(x=1.5, z=2.0),
                              planner.target(x=1.5, z=6.0)]
    result = planner.query_geometry(request)
    assert result.success and not result.corridor_safe
    assert 'never decrease' in result.message


def test_terrain_escape_never_waives_unknown_current_body(planner_factory):
    planner = _terrain_planner(planner_factory)
    planner.odometry_message.pose.pose.position.x = -4.0
    planner.repeat_odometry()
    planner.wait_for_response(lambda result: result.success, 'ordinary unknown-start readiness')
    result = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(x=1.0, z=7.5), minimum_agl=5.0,
        terrain_envelope=True, allow_terrain_escape=True,
    ))
    assert not result.success and result.error_code == result.START_INVALID
    assert 'unknown cell' in result.message
    assert not result.path.poses


@pytest.mark.parametrize('minimum_agl, terrain', [(0.0, True), (5.0, False)])
def test_terrain_escape_requires_an_explicit_positive_terrain_clearance(
        planner_factory, minimum_agl, terrain):
    planner = _strict_planner(planner_factory)
    query = planner.query_geometry(QueryInspectionGeometry.Request(
        minimum_agl=minimum_agl, terrain_envelope=terrain, allow_terrain_escape=True))
    assert not query.success and query.error_code == query.INVALID_REQUEST
    plan = planner.plan_inspection(PlanInspectionPath.Request(
        target=planner.target(z=5.5), minimum_agl=minimum_agl,
        terrain_envelope=terrain, allow_terrain_escape=True))
    assert not plan.success and plan.error_code == plan.INVALID_REQUEST


def test_terrain_escape_revalidation_keeps_original_horizontal_anchor(planner_factory):
    planner = _terrain_planner(planner_factory, roof=True)
    request = QueryInspectionGeometry.Request(
        minimum_agl=5.0, terrain_envelope=True, allow_terrain_escape=True,
    )
    request.corridor.poses = [planner.target(x=1.5, z=2.0),
                              planner.target(x=3.0, z=2.0), planner.target(x=3.0, z=5.5)]
    # Relative to a newly sampled first vertex, this looks like a 1.5 m escape.
    assert planner.query_geometry(request).corridor_safe
    request.escape_origin = _geometry_point(x=0.0, y=0.0, z=2.0)

    anchored = planner.query_geometry(request)

    assert anchored.success and not anchored.corridor_safe
    assert 'within 2 m' in anchored.message


def test_terrain_escape_completion_cannot_reopen_for_a_lower_actual_pose(planner_factory):
    planner = _terrain_planner(planner_factory, roof=True)
    request = QueryInspectionGeometry.Request(
        minimum_agl=5.0, terrain_envelope=True, allow_terrain_escape=True,
        escape_origin=_geometry_point(x=2.0, y=0.0, z=2.0),
    )
    request.corridor.poses = [planner.target(x=2.0, z=4.9), planner.target(x=2.0, z=5.5)]
    unfinished = planner.query_geometry(request)
    assert unfinished.success and unfinished.corridor_safe
    assert not unfinished.current_clearance_reached
    request.terrain_escape_complete = True

    completed = planner.query_geometry(request)

    assert completed.success and not completed.corridor_safe
    assert not completed.current_clearance_reached
    assert 'escape completed' in completed.message


def test_terrain_actual_clearance_proof_survives_rejection_of_a_later_segment(planner_factory):
    planner = _terrain_planner(planner_factory, roof=True)
    request = QueryInspectionGeometry.Request(
        minimum_agl=5.0, terrain_envelope=True, allow_terrain_escape=True,
        escape_origin=_geometry_point(x=2.0, y=0.0, z=2.0),
    )
    request.corridor.poses = [planner.target(x=2.0, z=5.5), planner.target(x=2.0, z=4.9),
                              planner.target(x=2.0, z=5.5)]

    result = planner.query_geometry(request)

    assert result.success and not result.corridor_safe
    assert result.current_clearance_reached
    assert 'escape completed' in result.message


@pytest.mark.parametrize('latest_wall', [True, False])
def test_center_fire_wall_aperture_requires_current_occupied_support(
        planner_factory, latest_wall):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    _publish_inspection_map(planner, wall=True)
    if not latest_wall:
        _publish_inspection_map(planner, wall=False)
    request = _inspection_request()
    request.smoke_wall_width = 3.0
    response = planner.query_geometry(request)
    assert response.success, response.message
    assert response.smoke_wall_valid is latest_wall
    assert response.smoke_wall_samples == (25 if latest_wall else 0)
    if latest_wall:
        assert response.smoke_wall_range == pytest.approx(3.0, abs=0.25)
        assert response.smoke_wall_nearest_range <= response.smoke_wall_range
        assert 0.0 < response.smoke_wall_uncertainty < 0.3


@pytest.mark.parametrize('wall', [True, False, 'unknown_wall'])
def test_center_fire_wall_aperture_does_not_label_static_or_unknown_as_smoke(
        planner_factory, wall):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=wall)
    request = _inspection_request()
    request.smoke_wall_width = 3.0
    response = planner.query_geometry(request)
    assert response.success, response.message
    assert not response.smoke_wall_valid
    assert response.smoke_wall_samples == 0
    assert 'occupied/free history rays=0' in response.message
    assert 'qualifying rays=0' in response.message


def test_wall_aperture_is_opt_in_and_bounded(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    _publish_inspection_map(planner, wall=True)
    request = _inspection_request()
    response = planner.query_geometry(request)
    assert response.surface_valid and response.plume_candidate
    assert not response.smoke_wall_valid
    request.smoke_wall_width = 4.0
    response = planner.query_geometry(request)
    assert not response.success and response.error_code == response.INVALID_REQUEST


def test_wall_aperture_does_not_treat_unknown_to_occupied_as_flicker(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall='unknown_wall')
    _publish_inspection_map(planner, wall=True)
    request = _inspection_request()
    request.smoke_wall_width = 3.0
    response = planner.query_geometry(request)
    assert response.success and not response.smoke_wall_valid
    assert 'current occupied rays=25/25' in response.message
    assert 'occupied/free history rays=0' in response.message


def test_wall_aperture_remains_transverse_for_navigation_along_map_y(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False, rotate=True)
    _publish_inspection_map(planner, wall=True, rotate=True)
    request = _inspection_request()
    request.direction.vector.x = 0.0
    request.direction.vector.y = 1.0
    request.smoke_wall_width = 3.0

    response = planner.query_geometry(request)

    assert response.success, response.message
    assert response.smoke_wall_valid and response.smoke_wall_samples == 25
    assert response.smoke_wall_range == pytest.approx(3.0, abs=0.25)
    # A wrongly map-Y-aligned 'lateral' axis changes ray start depths instead
    # of sampling across the wall, creating an artificial spread in range.
    assert response.smoke_wall_nearest_range == pytest.approx(response.smoke_wall_range)
    assert 0.0 < response.smoke_wall_uncertainty < 0.3


@pytest.mark.parametrize('samples, accepted', [
    (((-1, -1), (0, 0)), False),
    (((-1, -1), (-1, 0), (-1, 1)), False),
    (((-1, -1), (0, -1), (1, -1)), False),
    (((-1, -1), (-1, 0), (0, 0)), True),
    (((-1, -1), (-1, 0), (0, 0), (0, 1), (1, 1)), True),
    (((-1, -1), (-1, 0), (0, 0), (0, 1), (1, 1), (1, -1)), True),
])
def test_wall_aperture_requires_three_returns_across_two_rows_and_columns(
        planner_factory, samples, accepted):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    # At 0.25 m resolution, the 0.75 m grid spacing spans three voxel keys.
    cells = tuple((3 * column, 8 + 3 * row) for row, column in samples)
    _publish_inspection_map(planner, wall=cells)
    request = _inspection_request()
    request.smoke_wall_width = 3.0

    response = planner.query_geometry(request)

    assert response.success, response.message
    assert response.smoke_wall_samples == len(samples)
    assert response.smoke_wall_mixed_samples == len(samples)
    assert response.smoke_wall_valid is accepted
    assert response.smoke_wall_evidence == (
        response.WALL_EVIDENCE_FLICKER if accepted else response.WALL_EVIDENCE_NONE)
    assert f'qualifying rays={len(samples)}' in response.message
    assert 'required=3 rays/2 rows/2 columns' in response.message


def test_wall_aperture_observation_identity_tracks_new_current_support(planner_factory):
    planner = planner_factory()
    request = _inspection_request()
    request.smoke_wall_width = 3.0
    _publish_inspection_map(planner, wall=False)
    first_map = _publish_inspection_map(planner, wall=True)
    first = planner.query_geometry(request)
    replay = planner.query_geometry(request)
    assert first.smoke_wall_valid and replay.smoke_wall_valid
    assert first.map_header.stamp == replay.map_header.stamp == first_map.header.stamp

    free_map = _publish_inspection_map(planner, wall=False)
    missing = planner.query_geometry(request)
    assert missing.map_header.stamp == free_map.header.stamp
    assert not missing.smoke_wall_valid and missing.smoke_wall_samples == 0

    second_map = _publish_inspection_map(planner, wall=True)
    second = planner.query_geometry(request)
    assert second.smoke_wall_valid
    assert second.map_header.stamp == second_map.header.stamp != first.map_header.stamp


@pytest.mark.parametrize('slope', [1.0, 2.0])
def test_wall_guard_rejects_previously_observed_jittering_inclined_terrain(
        planner_factory, slope):
    from test_smoke_wall_false_positives import _publish_and_query, _request, _terrain_map
    planner = planner_factory()
    frame = 'guarded_inclined_terrain'
    _publish_and_query(planner, frame, _terrain_map(12.0, slope, 0.03, 0.09))
    baseline = _publish_and_query(planner, frame, _terrain_map(12.0, slope, 0.03, -0.09))
    assert baseline.smoke_wall_valid and baseline.smoke_wall_samples >= 3
    request = _request(frame)
    request.smoke_wall_reject_terrain = True
    guarded = planner.query_geometry(request)
    assert guarded.success and not guarded.smoke_wall_valid, guarded.message
    assert 'stable terrain rays excluded=25' in guarded.message
    assert 'prior terrain support=25' in guarded.message


def test_wall_guard_preserves_repeated_vertical_smoke_fronts(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query, _request, _terrain_map
    planner = planner_factory()
    frame = 'guarded_vertical_smoke'
    _publish_and_query(
        planner, frame, _terrain_map(16.0, math.inf, 0.01, 0.0, free_surface=True))
    _publish_and_query(planner, frame, _terrain_map(16.0, math.inf, 0.01, 0.0))
    request = _request(frame)
    request.smoke_wall_reject_terrain = True
    for _ in range(3):
        _publish_and_query(planner, frame, _terrain_map(16.0, math.inf, 0.01, 0.0))
        result = planner.query_geometry(request)
        assert result.success and result.smoke_wall_valid, result.message
        assert result.smoke_wall_samples == 25
        assert 'stable terrain rays excluded=0' in result.message


def test_wall_guard_does_not_call_a_new_inclined_volume_preexisting_terrain(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query, _request, _terrain_map
    planner = planner_factory()
    frame = 'new_inclined_volume'
    _publish_and_query(planner, frame, _terrain_map(16.0, 1.0, 0.0, 0.0, free_surface=True))
    _publish_and_query(planner, frame, _terrain_map(16.0, 1.0, 0.0, 0.0))
    request = _request(frame)
    request.smoke_wall_reject_terrain = True
    result = planner.query_geometry(request)
    assert result.success and result.smoke_wall_valid, result.message
    assert result.smoke_wall_samples == 25


def _guard_aperture_scene(*, smoke=False, curved=False, translation=0.0):
    from test_smoke_wall_false_positives import ORIGIN, RAY_GRID, _encode_octomap, _float32, _key
    leaves = {}
    for row, column in RAY_GRID:
        y = _float32(_float32(ORIGIN[1]) + column * 0.75)
        z = _float32(_float32(ORIGIN[2]) + row * 0.75)
        if curved:
            depth = 16.0 + 0.6 * row * row + 0.4 * column * column + translation
            occupied = True
        else:
            depth = 8.0 + row * 0.75 + column * 0.75 * 0.15 if row <= 0 else 16.0
            occupied = row <= 0 or smoke
        hit = _key(ORIGIN[0] + depth)
        for x in range(_key(ORIGIN[0]), hit + 1):
            leaves[(x, _key(y), _key(z))] = 2 if x == hit and occupied else 1
    return _encode_octomap(leaves)


def test_wall_guard_excludes_foreground_slope_but_preserves_smoke_above_it(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query, _request
    planner = planner_factory()
    frame = 'smoke_and_foreground_slope'
    _publish_and_query(planner, frame, _guard_aperture_scene(smoke=False))
    _publish_and_query(planner, frame, _guard_aperture_scene(smoke=True))
    request = _request(frame)
    request.smoke_wall_reject_terrain = True
    result = planner.query_geometry(request)
    assert result.success and result.smoke_wall_valid, result.message
    assert result.smoke_wall_samples == 10
    assert 'stable terrain rays excluded=15' in result.message
    assert result.smoke_wall_nearest_range > 15.0


def test_wall_guard_does_not_reject_unstructured_returns_using_a_weak_plane_fit(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query, _request
    planner = planner_factory()
    frame = 'irregular_changing_smoke'
    _publish_and_query(planner, frame, _guard_aperture_scene(curved=True, translation=0.21))
    _publish_and_query(planner, frame, _guard_aperture_scene(curved=True))
    request = _request(frame)
    request.smoke_wall_reject_terrain = True
    result = planner.query_geometry(request)
    assert result.success and result.smoke_wall_valid, result.message
    assert 'stable terrain rays excluded=0' in result.message


def _camera_association_request(planner, frame='planner_test_map'):
    request = _inspection_request()
    request.smoke_wall_width = 3.0
    request.smoke_wall_camera_roi = True
    request.smoke_camera_header.frame_id = frame
    request.smoke_camera_header.stamp = planner.node.get_clock().now().to_msg()
    # The test camera origin is the map origin, so the 3 m wall spans these angles.
    request.smoke_yaw_min_rad, request.smoke_yaw_max_rad = -0.7, 0.7
    request.smoke_vertical_min_rad, request.smoke_vertical_max_rad = -0.1, 1.0
    return request


def test_wall_camera_association_rejects_hits_outside_detected_smoke(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    _publish_inspection_map(planner, wall=True)
    request = _camera_association_request(planner)
    accepted = planner.query_geometry(request)
    assert accepted.success and accepted.smoke_wall_valid, accepted.message
    request.smoke_yaw_min_rad, request.smoke_yaw_max_rad = 1.0, 1.2
    rejected = planner.query_geometry(request)
    assert rejected.success and not rejected.smoke_wall_valid, rejected.message
    assert not rejected.smoke_wall_nearest_point_valid
    assert 'camera-associated rays=0' in rejected.message


def test_wall_camera_association_uses_translated_and_rotated_source_frame(planner_factory):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False, rotate=True)
    _publish_inspection_map(planner, wall=True, rotate=True)
    camera = TransformStamped()
    camera.header.stamp = planner.node.get_clock().now().to_msg()
    camera.header.frame_id = 'planner_test_map'
    camera.child_frame_id = planner.namespace.strip('/') + '_rgb_camera'
    camera.transform.translation.x = 0.125
    camera.transform.translation.y = 0.125
    camera.transform.translation.z = 2.125
    camera.transform.rotation.z = math.sin(math.pi / 4.0)
    camera.transform.rotation.w = math.cos(math.pi / 4.0)
    planner.transform_broadcaster.sendTransform(camera)
    request = _camera_association_request(planner, camera.child_frame_id)
    request.direction.vector.x = 0.0
    request.direction.vector.y = 1.0
    request.smoke_yaw_min_rad, request.smoke_yaw_max_rad = -0.6, 0.6
    request.smoke_vertical_min_rad, request.smoke_vertical_max_rad = -0.6, 0.6
    response = planner.query_geometry(request)
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_samples == 25
    assert response.smoke_wall_nearest_point_valid
    assert response.smoke_wall_nearest_point.header == response.map_header
    assert response.smoke_wall_nearest_point.header.frame_id != camera.child_frame_id
    assert response.smoke_wall_nearest_point.point.y > 2.5
    assert 'camera-associated rays=25' in response.message


@pytest.mark.parametrize('problem', ['bounds', 'image_age', 'missing_frame'])
def test_wall_camera_association_requires_valid_source_metadata(planner_factory, problem):
    planner = planner_factory()
    _publish_inspection_map(planner, wall=False)
    _publish_inspection_map(planner, wall=True)
    request = _camera_association_request(planner)
    if problem == 'bounds':
        request.smoke_yaw_min_rad = request.smoke_yaw_max_rad
    elif problem == 'image_age':
        request.smoke_camera_header.stamp.sec -= 10
    else:
        request.smoke_camera_header.frame_id = ''
    response = planner.query_geometry(request)
    assert not response.success and response.error_code == response.INVALID_REQUEST


EXTENDED_WALL_GRID = tuple((row, column) for row in range(-3, 4) for column in range(-3, 4))


def _extended_wall_scene(occupied_samples=EXTENDED_WALL_GRID, *, free_samples=(),
                         slope=math.inf, translation=0.0, distance=12.0, phase=0.0):
    """Encode occupied/free returns at the production 0.75 m aperture spacing."""
    from test_smoke_wall_false_positives import ORIGIN, _encode_octomap, _float32, _key
    occupied_samples = set(occupied_samples)
    leaves = {}
    for row, column in occupied_samples | set(free_samples):
        y = _float32(_float32(ORIGIN[1]) + column * 0.75)
        z = _float32(_float32(ORIGIN[2]) + row * 0.75)
        depth = distance + phase + row * 0.75 / slope + column * 0.75 * 0.15
        base, hit = _key(ORIGIN[0] + depth), _key(ORIGIN[0] + depth + translation)
        for x in range(base - 2, hit + 1):
            leaves[(x, _key(y), _key(z))] = (
                2 if x == hit and (row, column) in occupied_samples else 1)
    return _encode_octomap(leaves)


def _extended_wall_request(planner, frame, *, camera_roi=True, reject_terrain=True):
    from test_smoke_wall_false_positives import _request
    request = _request(frame)
    request.smoke_wall_extended_evidence = True
    request.smoke_wall_width = 4.5
    request.smoke_wall_reject_terrain = reject_terrain
    request.smoke_wall_camera_roi = camera_roi
    request.smoke_camera_header.frame_id = frame
    request.smoke_camera_header.stamp = planner.node.get_clock().now().to_msg()
    request.smoke_yaw_min_rad, request.smoke_yaw_max_rad = -1.3, 1.3
    request.smoke_vertical_min_rad, request.smoke_vertical_max_rad = -1.3, 1.3
    return request


def test_extended_wall_uses_one_mixed_ray_and_other_current_support(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query
    planner = planner_factory()
    frame = 'extended_wall_one_mixed_ray'
    previous = tuple(sample for sample in EXTENDED_WALL_GRID if sample != (0, 0))
    _publish_and_query(planner, frame, _extended_wall_scene(previous, free_samples=((0, 0),)))
    _publish_and_query(planner, frame, _extended_wall_scene())
    response = planner.query_geometry(_extended_wall_request(planner, frame))
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_evidence == response.WALL_EVIDENCE_FLICKER
    assert response.smoke_wall_samples == 49 and response.smoke_wall_mixed_samples == 1
    assert response.smoke_wall_rows == response.smoke_wall_columns == 7
    assert 'grid=7x7, width=4.5 m, evidence=FLICKER' in response.message
    assert 'camera ROI enabled=1, terrain rejection enabled=1' in response.message


@pytest.mark.parametrize('samples, accepted', [
    (((-1, -1), (0, 0), (1, 1)), True),
    (((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 0), (0, 1)), False),
    (((-1, -1), (0, -1), (1, -1), (-1, 0), (0, 0), (1, 0)), False),
])
def test_extended_wall_requires_three_rows_and_columns(planner_factory, samples, accepted):
    from test_smoke_wall_false_positives import _publish_and_query
    planner = planner_factory()
    frame = 'extended_wall_coverage'
    _publish_and_query(planner, frame, _extended_wall_scene((), free_samples=samples))
    _publish_and_query(planner, frame, _extended_wall_scene(samples))
    response = planner.query_geometry(_extended_wall_request(planner, frame))
    assert response.success, response.message
    assert response.smoke_wall_valid is accepted
    expected_samples = len(samples) if accepted else 0
    assert response.smoke_wall_samples == response.smoke_wall_mixed_samples == expected_samples
    assert f'largest group rays={len(samples)}' in response.message
    assert response.smoke_wall_evidence == (
        response.WALL_EVIDENCE_FLICKER if accepted else response.WALL_EVIDENCE_NONE)
    assert 'required=3 rays/3 rows/3 columns' in response.message


@pytest.mark.parametrize('samples, accepted', [
    (((-1, -1), (-1, 0), (0, -1), (0, 1), (1, 0), (1, 1)), True),
    (((-1, -1), (-1, 0), (0, -1), (0, 1), (1, 1)), False),
])
def test_extended_wall_persistent_route_requires_six_current_hits(
        planner_factory, samples, accepted):
    from test_smoke_wall_false_positives import _publish_and_query
    planner = planner_factory()
    frame = 'extended_wall_persistent_minimum'
    _publish_and_query(planner, frame, _extended_wall_scene(samples))
    _publish_and_query(planner, frame, _extended_wall_scene(samples))
    response = planner.query_geometry(_extended_wall_request(planner, frame))
    assert response.success, response.message
    assert response.smoke_wall_valid is accepted
    assert response.smoke_wall_mixed_samples == 0
    assert response.smoke_wall_samples == (len(samples) if accepted else 0)
    assert response.smoke_wall_rows == response.smoke_wall_columns == (3 if accepted else 0)
    assert f'largest group rays={len(samples)}' in response.message
    assert response.smoke_wall_evidence == (
        response.WALL_EVIDENCE_PERSISTENT if accepted else response.WALL_EVIDENCE_NONE)


@pytest.mark.parametrize('missing_guard', ['camera_roi', 'terrain_rejection'])
def test_extended_wall_persistent_route_requires_both_classification_guards(
        planner_factory, missing_guard):
    from test_smoke_wall_false_positives import _publish_and_query
    planner = planner_factory()
    frame = 'extended_wall_requires_guards'
    _publish_and_query(planner, frame, _extended_wall_scene())
    _publish_and_query(planner, frame, _extended_wall_scene())
    request = _extended_wall_request(
        planner, frame, camera_roi=missing_guard != 'camera_roi',
        reject_terrain=missing_guard != 'terrain_rejection')
    response = planner.query_geometry(request)
    assert response.success and not response.smoke_wall_valid, response.message
    assert response.smoke_wall_evidence == response.WALL_EVIDENCE_NONE
    assert response.smoke_wall_samples == response.smoke_wall_mixed_samples == 0
    assert 'largest group rays=49' in response.message


def test_extended_wall_preserves_a_steady_vertical_front_without_mixed_history(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query
    planner = planner_factory()
    frame = 'extended_wall_steady_front'
    for index in range(3):
        _publish_and_query(planner, frame, _extended_wall_scene())
        response = planner.query_geometry(_extended_wall_request(planner, frame))
        assert response.success, response.message
        assert response.smoke_wall_valid is (index > 0)
        assert response.smoke_wall_evidence == (
            response.WALL_EVIDENCE_PERSISTENT if index else response.WALL_EVIDENCE_NONE)
        assert response.smoke_wall_samples == (49 if index else 0)
        assert response.smoke_wall_mixed_samples == 0
        assert 'stable terrain rays excluded=0' in response.message
        assert f'prior map context={int(index > 0)}' in response.message


@pytest.mark.parametrize('slope', [1.0, 2.0])
def test_extended_wall_guard_rejects_persistent_jittering_terrain(
        planner_factory, slope, record_testsuite_property):
    from test_smoke_wall_false_positives import _publish_and_query
    planner = planner_factory()
    frame = 'extended_wall_inclined_terrain'
    _publish_and_query(planner, frame, _extended_wall_scene(slope=slope, translation=0.09))
    initial = planner.query_geometry(_extended_wall_request(planner, frame))
    assert initial.success and not initial.smoke_wall_valid, initial.message
    assert initial.smoke_wall_evidence == initial.WALL_EVIDENCE_NONE
    assert initial.smoke_wall_samples == 0
    assert 'current occupied rays=49/49' in initial.message
    assert 'prior map context=0' in initial.message
    _publish_and_query(planner, frame, _extended_wall_scene(slope=slope, translation=-0.09))
    request = _extended_wall_request(planner, frame)
    elapsed = []
    for _ in range(3):
        started = time.perf_counter()
        response = planner.query_geometry(request)
        elapsed.append(time.perf_counter() - started)
        assert response.success and not response.smoke_wall_valid, response.message
        assert response.smoke_wall_evidence == response.WALL_EVIDENCE_NONE
        assert response.smoke_wall_samples == response.smoke_wall_mixed_samples == 0
        assert 'stable terrain rays excluded=49' in response.message
        assert 'prior terrain support=49' in response.message
    median = sorted(elapsed)[len(elapsed) // 2]
    prefix = f'wall_probe_49_ray_slope_{slope:g}'
    record_testsuite_property(prefix + '_median_seconds', median)
    record_testsuite_property(prefix + '_max_seconds', max(elapsed))
    print(f'49-ray terrain guard slope={slope:g}: median query {median:.4f} s, '
          f'maximum query {max(elapsed):.4f} s')


def test_extended_wall_samples_outer_rays_at_two_and_a_quarter_metres(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query, _request
    planner = planner_factory()
    frame = 'extended_wall_outer_grid'
    samples = ((-3, -3), (0, 0), (3, 3))
    _publish_and_query(planner, frame, _extended_wall_scene((), free_samples=samples))
    _publish_and_query(planner, frame, _extended_wall_scene(samples))
    legacy = planner.query_geometry(_request(frame))
    assert not legacy.smoke_wall_valid and legacy.smoke_wall_samples == 1
    extended = planner.query_geometry(_extended_wall_request(planner, frame))
    assert extended.success and extended.smoke_wall_valid, extended.message
    assert extended.smoke_wall_evidence == extended.WALL_EVIDENCE_FLICKER
    assert extended.smoke_wall_samples == 3
    assert extended.smoke_wall_rows == extended.smoke_wall_columns == 3


def test_extended_wall_width_limit_is_separate_from_legacy_measurement(planner_factory):
    from test_smoke_wall_false_positives import _publish_and_query, _request
    planner = planner_factory()
    frame = 'extended_wall_width_limit'
    _publish_and_query(planner, frame, _extended_wall_scene())
    request = _request(frame)
    request.smoke_wall_width = 4.5
    legacy = planner.query_geometry(request)
    assert not legacy.success and legacy.error_code == legacy.INVALID_REQUEST
    request = _extended_wall_request(planner, frame)
    assert planner.query_geometry(request).success
    request.smoke_wall_width = 4.5001
    oversized = planner.query_geometry(request)
    assert not oversized.success and oversized.error_code == oversized.INVALID_REQUEST


def _wall_depth_scene(depths, *, free_samples=()):
    """Map independently chosen depths without changing the aperture ray lanes."""
    from test_smoke_wall_false_positives import ORIGIN, _encode_octomap, _float32, _key
    free_samples = set(free_samples)
    leaves = {}
    for (row, column), depth in depths.items():
        y = _float32(_float32(ORIGIN[1]) + column * 0.75)
        z = _float32(_float32(ORIGIN[2]) + row * 0.75)
        hit = _key(ORIGIN[0] + depth)
        for x in range(hit - 2, hit + 1):
            leaves[(x, _key(y), _key(z))] = (
                2 if x == hit and (row, column) not in free_samples else 1)
    return _encode_octomap(leaves)


def _observe_wall_depths(planner, frame, depths, *, mixed_samples=()):
    from test_smoke_wall_false_positives import _publish_and_query
    _publish_and_query(planner, frame, _wall_depth_scene(depths, free_samples=mixed_samples))
    _publish_and_query(planner, frame, _wall_depth_scene(depths))
    return planner.query_geometry(_extended_wall_request(planner, frame))


@pytest.mark.parametrize('route', ['flicker', 'persistent'])
def test_wall_depth_groups_separate_near_front_from_far_background(planner_factory, route):
    from test_smoke_wall_false_positives import _request
    planner = planner_factory()
    frame = 'wall_front_and_background'
    front = {(row, column) for row in range(-1, 2) for column in range(-1, 2)}
    depths = {sample: 18.0 if sample in front else 30.0 for sample in EXTENDED_WALL_GRID}
    mixed = ((0, 0),) if route == 'flicker' else ()
    response = _observe_wall_depths(planner, frame, depths, mixed_samples=mixed)
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_evidence == (
        response.WALL_EVIDENCE_FLICKER if mixed else response.WALL_EVIDENCE_PERSISTENT)
    assert response.smoke_wall_samples == 9
    assert response.smoke_wall_mixed_samples == len(mixed)
    assert response.smoke_wall_rows == response.smoke_wall_columns == 3
    assert response.smoke_wall_range == pytest.approx(18.0, abs=0.2)
    assert response.smoke_wall_nearest_range == pytest.approx(18.0, abs=0.2)
    assert response.smoke_wall_uncertainty < 0.3
    assert 'depth groups=2, qualified depth groups=2' in response.message
    assert 'selected depth span=0 m, voxel uncertainty=' in response.message
    assert 'depth spread=0 m' in response.message
    if mixed:
        # Legacy history-only pooling still uses its original 25-ray aperture.
        legacy_request = _request(frame)
        legacy_request.smoke_wall_reject_terrain = True
        legacy = planner.query_geometry(legacy_request)
        assert not legacy.smoke_wall_valid and legacy.smoke_wall_samples == 1


def test_wall_depth_groups_do_not_use_an_isolated_nearest_return(planner_factory):
    planner = planner_factory()
    frame = 'wall_front_with_sparse_outlier'
    depths = {sample: 18.0 for sample in EXTENDED_WALL_GRID}
    depths[(-3, -3)] = 8.0
    response = _observe_wall_depths(planner, frame, depths, mixed_samples=((0, 0),))
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_evidence == response.WALL_EVIDENCE_FLICKER
    assert response.smoke_wall_samples == 48
    assert response.smoke_wall_nearest_range > 17.5
    assert response.smoke_wall_range == pytest.approx(18.0, abs=0.2)
    assert response.smoke_wall_nearest_point_valid
    assert response.smoke_wall_nearest_point.point.x > 17.5
    assert 'depth groups=2, qualified depth groups=1' in response.message


def test_wall_depth_groups_cannot_combine_coverage_across_fronts(planner_factory):
    planner = planner_factory()
    frame = 'wall_fronts_with_separate_partial_coverage'
    depths = {(row, column): 18.0 for row in (-1, 0) for column in (-1, 0)}
    depths.update({(row, column): 30.0 for row in (1, 2) for column in (1, 2)})
    response = _observe_wall_depths(planner, frame, depths, mixed_samples=depths)
    assert response.success and not response.smoke_wall_valid, response.message
    assert response.smoke_wall_samples == response.smoke_wall_mixed_samples == 0
    assert response.smoke_wall_rows == response.smoke_wall_columns == 0
    assert not response.smoke_wall_nearest_point_valid
    assert 'depth groups=2, qualified depth groups=0' in response.message
    assert ('largest group rays=4, largest group rows=2, largest group columns=2'
            in response.message)


def test_wall_depth_groups_cannot_borrow_a_distant_groups_mixed_history(planner_factory):
    planner = planner_factory()
    frame = 'wall_fronts_with_separate_history_evidence'
    near = ((-1, -1), (0, 0), (1, 1))
    far = ((-3, -3), (0, 3), (3, 0))
    depths = {sample: 18.0 for sample in near}
    depths.update({sample: 30.0 for sample in far})
    response = _observe_wall_depths(planner, frame, depths, mixed_samples=far)
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_evidence == response.WALL_EVIDENCE_FLICKER
    assert response.smoke_wall_samples == response.smoke_wall_mixed_samples == 3
    assert response.smoke_wall_nearest_range > 29.5
    assert response.smoke_wall_nearest_point_valid
    assert response.smoke_wall_nearest_point.point.x > 29.5
    assert 'depth groups=2, qualified depth groups=1' in response.message


def test_wall_depth_groups_cannot_pool_persistent_sample_counts(planner_factory):
    planner = planner_factory()
    frame = 'wall_fronts_with_separate_persistent_support'
    near = ((-1, -1), (-1, 0), (0, -1), (0, 1), (1, 1))
    depths = {sample: 18.0 for sample in near}
    depths[(3, 3)] = 30.0
    response = _observe_wall_depths(planner, frame, depths)
    assert response.success and not response.smoke_wall_valid, response.message
    assert response.smoke_wall_samples == 0
    assert 'depth groups=2, qualified depth groups=0' in response.message
    assert 'largest group rays=5' in response.message


@pytest.mark.parametrize('span, accepted', [(4.0, True), (2.0, False)])
def test_wall_depth_groups_bound_chains_independently_of_neighbor_gaps(
        planner_factory, span, accepted):
    planner = planner_factory(planner_parameters={'inspection_wall_cluster_span': span})
    frame = 'wall_depth_chain'
    depths = {(index - 3, index - 3): 18.0 + index * 2.0 for index in range(7)}
    response = _observe_wall_depths(planner, frame, depths, mixed_samples=depths)
    assert response.success, response.message
    assert response.smoke_wall_valid is accepted
    if accepted:
        assert response.smoke_wall_evidence == response.WALL_EVIDENCE_FLICKER
        assert response.smoke_wall_samples == 3
        assert response.smoke_wall_range == pytest.approx(20.0, abs=0.2)
        assert response.smoke_wall_uncertainty < 2.0
        assert 'depth groups=3, qualified depth groups=2' in response.message
        assert 'selected depth span=4 m' in response.message
    else:
        assert response.smoke_wall_samples == 0
        assert 'depth groups=4, qualified depth groups=0' in response.message


def test_wall_depth_groups_do_not_trim_support_to_make_uncertainty_pass(planner_factory):
    planner = planner_factory()
    frame = 'wall_depth_group_uncertainty_is_retained'
    near = ((-1, -1), (0, 0), (1, 1))
    far = ((-1, 0), (0, 1), (1, -1))
    depths = {sample: 18.0 for sample in near}
    depths.update({sample: 22.0 for sample in far})
    response = _observe_wall_depths(planner, frame, depths, mixed_samples=depths)
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_samples == response.smoke_wall_mixed_samples == 6
    assert response.smoke_wall_range == pytest.approx(20.0, abs=0.2)
    assert response.smoke_wall_uncertainty > 2.0  # The unchanged client cap must reject it.
    assert 'depth groups=1, qualified depth groups=1' in response.message
    assert 'depth spread=2 m' in response.message


def test_wall_depth_groups_leave_legacy_range_pooling_unchanged(planner_factory):
    from test_smoke_wall_false_positives import _request
    planner = planner_factory()
    frame = 'wall_depth_groups_legacy_compatibility'
    front = {(row, column) for row in range(-1, 2) for column in range(-1, 2)}
    depths = {sample: 18.0 if sample in front else 30.0 for sample in EXTENDED_WALL_GRID}
    extended = _observe_wall_depths(planner, frame, depths, mixed_samples=depths)
    assert extended.smoke_wall_valid and extended.smoke_wall_samples == 9
    legacy = planner.query_geometry(_request(frame))
    assert legacy.success and legacy.smoke_wall_valid, legacy.message
    assert legacy.smoke_wall_evidence == legacy.WALL_EVIDENCE_FLICKER
    assert legacy.smoke_wall_samples == legacy.smoke_wall_mixed_samples == 25
    assert legacy.smoke_wall_range == pytest.approx((9 * 18.0 + 16 * 30.0) / 25, abs=0.2)
    assert legacy.smoke_wall_uncertainty > 5.0
    assert legacy.smoke_wall_nearest_point_valid
    assert 'depth groups=' not in legacy.message


def _wall_voxel_center(sample, depth):
    from test_smoke_wall_false_positives import ORIGIN, RESOLUTION, _float32, _key
    row, column = sample
    coordinates = (ORIGIN[0] + depth,
                   _float32(_float32(ORIGIN[1]) + column * 0.75),
                   _float32(_float32(ORIGIN[2]) + row * 0.75))
    return tuple((_key(value) - 32768 + 0.5) * RESOLUTION for value in coordinates)


@pytest.mark.parametrize('route', ['flicker', 'persistent'])
def test_wall_point_uses_euclidean_nearest_hit_without_changing_nearest_depth(
        planner_factory, route):
    planner = planner_factory()
    frame = 'wall_point_euclidean_nearest'
    depths = {(-3, -3): 17.8, (0, 0): 18.0, (3, 3): 18.6,
              (-3, 0): 18.4, (0, 3): 18.4, (3, -3): 18.4}
    response = _observe_wall_depths(
        planner, frame, depths, mixed_samples=((-3, -3),) if route == 'flicker' else ())
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_nearest_point_valid
    assert response.smoke_wall_evidence == (
        response.WALL_EVIDENCE_FLICKER if route == 'flicker'
        else response.WALL_EVIDENCE_PERSISTENT)
    point = response.smoke_wall_nearest_point
    assert point.header == response.map_header
    assert point.header.frame_id == frame
    assert point.header.stamp.sec > 0
    assert tuple(getattr(point.point, axis) for axis in 'xyz') == pytest.approx(
        _wall_voxel_center((0, 0), 18.0), abs=1e-4)
    assert response.smoke_wall_nearest_range == pytest.approx(17.8, abs=0.11)
    assert point.point.x - response.ray_origin.point.x > response.smoke_wall_nearest_range + 0.1
    assert response.smoke_wall_samples == 6


@pytest.mark.parametrize('route', ['flicker', 'persistent'])
def test_wall_point_preserves_the_actual_off_axis_qualified_hit(planner_factory, route):
    from test_smoke_wall_false_positives import ORIGIN
    planner = planner_factory()
    frame = 'wall_point_off_axis_hit'
    samples = ((-1, -1), (0, 1), (1, 0), (-1, 1), (0, -1), (1, 1))
    depths = {sample: 18.0 for sample in samples}
    depths[(0, 1)] = 17.5
    response = _observe_wall_depths(
        planner, frame, depths, mixed_samples=((0, 1),) if route == 'flicker' else ())
    assert response.success and response.smoke_wall_valid, response.message
    assert response.smoke_wall_nearest_point_valid
    point = response.smoke_wall_nearest_point
    assert point.header == response.map_header
    assert tuple(getattr(point.point, axis) for axis in 'xyz') == pytest.approx(
        _wall_voxel_center((0, 1), 17.5), abs=1e-4)
    assert point.point.y - ORIGIN[1] > 0.5
    assert response.smoke_wall_nearest_range == pytest.approx(17.5, abs=0.2)


def test_wall_point_stays_invalid_for_a_surface_query_without_wall_evidence(planner_factory):
    planner = planner_factory()
    # Ordinary single-ray queries stop at unknown cells. Use the dense
    # observed-free fixture so this test really reaches an unrelated surface.
    _publish_inspection_map(planner, wall=False)
    _publish_inspection_map(planner, wall=True)
    request = _inspection_request()
    request.smoke_wall_width = 3.0
    qualified = planner.query_geometry(request)
    assert qualified.smoke_wall_valid and qualified.smoke_wall_nearest_point_valid
    request.smoke_wall_width = 0.0
    surface = planner.query_geometry(request)
    assert surface.success and surface.surface_valid, surface.message
    assert not surface.smoke_wall_valid
    assert not surface.smoke_wall_nearest_point_valid
