"""Exercise the installed planner through DDS without a flight controller."""

import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from kv_executive.srv import PlanPath
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


class PlannerHarness:
    """Run the real C++ planner with all inputs confined to a test namespace."""

    def __init__(self, directory, *, home_from_first_odometry=True, cached_map=False):
        self.namespace = f'/planner_inputs_{os.getpid()}_{uuid.uuid4().hex[:8]}'
        self.odometry_topic = f'{self.namespace}/external_odometry'
        self.home_topic = f'{self.namespace}/home'
        self.map_topic = f'{self.namespace}/retained_map'
        self.home_from_first_odometry = home_from_first_odometry
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node('planner_input_test', namespace=self.namespace, context=self.context)
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
                'odometry_topic': self.odometry_topic,
                'home_position_topic': self.home_topic,
                'octomap_topic': self.map_topic,
                'mavros_parameter_service': f'{self.namespace}/unused_px4_parameters',
            }
            if not home_from_first_odometry:
                # Leave the default unoverridden in the first-odometry case.
                parameters['home_from_first_odometry'] = False
            binary = (
                Path(get_package_prefix('kv_executive'))
                / 'lib' / 'kv_executive' / 'motion_planning'
            )
            arguments = [str(binary), '--ros-args', '-r', f'__ns:={self.namespace}']
            for name, value in parameters.items():
                if isinstance(value, bool):
                    value = str(value).lower()
                arguments.extend(['-p', f'{name}:={value}'])
            self.process = subprocess.Popen(
                arguments, stdout=self.log_file, stderr=subprocess.STDOUT,
            )
            self.wait_for(
                lambda: self.client.service_is_ready()
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
    monkeypatch.setenv('ROS_DOMAIN_ID', str(160 + os.getpid() % 30))
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
