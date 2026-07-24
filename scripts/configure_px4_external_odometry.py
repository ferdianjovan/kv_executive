#!/usr/bin/env python3
"""Configure PX4 EKF2 to fuse MAVROS external odometry."""

import math
import sys
import time

from mavros_msgs.srv import ParamPull
from mavros_msgs.srv import ParamSetV2
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


class ExternalOdometryConfigurator(Node):
    """Set the PX4 parameters exposed by the MAVROS parameter plugin."""

    def __init__(self):
        super().__init__('configure_px4_external_odometry')
        target_node = self.declare_parameter(
            'mavros_parameter_node', '/mavros/param'
        ).value.rstrip('/')
        self.odometry_topic = self.declare_parameter(
            'external_odometry_topic', '/external/odometry'
        ).value
        self.expected_parent_frame = self.declare_parameter(
            'expected_parent_frame', ''
        ).value
        self.expected_child_frame = self.declare_parameter(
            'expected_child_frame', ''
        ).value
        self.timeout = float(self.declare_parameter('timeout', 60.0).value)
        height_reference = int(
            self.declare_parameter('ekf2_hgt_ref', -1).value
        )
        self.values = {
            'EKF2_EV_CTRL': int(
                self.declare_parameter('ekf2_ev_ctrl', 15).value
            ),
            'EKF2_EV_POS_X': float(
                self.declare_parameter('ekf2_ev_pos_x', 0.0).value
            ),
            'EKF2_EV_POS_Y': float(
                self.declare_parameter('ekf2_ev_pos_y', 0.0).value
            ),
            'EKF2_EV_POS_Z': float(
                self.declare_parameter('ekf2_ev_pos_z', 0.0).value
            ),
        }
        if height_reference >= 0:
            if height_reference > 3:
                raise ValueError('ekf2_hgt_ref must be -1 or in the range 0..3')
            self.values['EKF2_HGT_REF'] = height_reference

        self.valid_odometry_samples = 0
        self.odometry_subscription = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.pull_client = self.create_client(
            ParamPull, f'{target_node}/pull'
        )
        self.set_client = self.create_client(
            ParamSetV2, f'{target_node}/set'
        )

    def _odometry_callback(self, message):
        if message.header.frame_id != self.expected_parent_frame:
            self.get_logger().warning(
                'Ignoring external odometry with parent frame '
                f"'{message.header.frame_id}'; expected "
                f"'{self.expected_parent_frame}'",
                throttle_duration_sec=5.0,
            )
            return
        if message.child_frame_id != self.expected_child_frame:
            self.get_logger().warning(
                'Ignoring external odometry with child frame '
                f"'{message.child_frame_id}'; expected "
                f"'{self.expected_child_frame}'",
                throttle_duration_sec=5.0,
            )
            return

        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        quaternion_norm = math.sqrt(
            pose.orientation.x**2
            + pose.orientation.y**2
            + pose.orientation.z**2
            + pose.orientation.w**2
        )
        if not all(math.isfinite(value) for value in values):
            return
        if quaternion_norm < 0.5 or quaternion_norm > 1.5:
            return

        self.valid_odometry_samples += 1

    def _wait_for_odometry(self, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.valid_odometry_samples >= 5:
                self.get_logger().info(
                    f'Validated external odometry on {self.odometry_topic}'
                )
                return True

        self.get_logger().error(
            'No valid external odometry was received; PX4 parameters were '
            'not changed'
        )
        return False

    def _call(self, client, request, deadline):
        timeout = min(5.0, max(0.0, deadline - time.monotonic()))
        if timeout <= 0.0:
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None

        try:
            return future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f'MAVROS parameter request failed: {error}')
            return None

    def _wait_for_parameter_services(self, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            pull_ready = self.pull_client.wait_for_service(timeout_sec=0.5)
            set_ready = self.set_client.wait_for_service(timeout_sec=0.5)
            if pull_ready and set_ready:
                return True

            self.get_logger().info(
                'Waiting for the MAVROS parameter services...',
                throttle_duration_sec=5.0,
            )

        return False

    def _pull_parameters(self, deadline):
        request = ParamPull.Request(force_pull=True)
        while rclpy.ok() and time.monotonic() < deadline:
            response = self._call(self.pull_client, request, deadline)
            if (
                response is not None
                and response.success
                and response.param_received > 0
            ):
                self.get_logger().info(
                    'Imported '
                    f'{response.param_received} PX4 parameters into MAVROS'
                )
                return True

            self.get_logger().warning(
                'PX4 parameter import is not ready; retrying',
                throttle_duration_sec=5.0,
            )
            time.sleep(1.0)

        return False

    def _set_parameter(self, name, value, deadline):
        request = ParamSetV2.Request(
            force_set=False,
            param_id=name,
            value=Parameter(name=name, value=value).get_parameter_value(),
        )
        while rclpy.ok() and time.monotonic() < deadline:
            response = self._call(self.set_client, request, deadline)
            if response is not None and response.success:
                self.get_logger().info(f'Set PX4 parameter {name}={value}')
                return True

            self.get_logger().warning(
                f'PX4 rejected {name}={value}; retrying',
                throttle_duration_sec=5.0,
            )
            time.sleep(1.0)

        return False

    def configure(self):
        """Validate odometry, then wait for MAVROS and set EKF2 values."""
        deadline = time.monotonic() + self.timeout
        if not self._wait_for_odometry(deadline):
            return False

        if not self._wait_for_parameter_services(deadline):
            self.get_logger().error(
                'Timed out waiting for the MAVROS parameter services'
            )
            return False

        if not self._pull_parameters(deadline):
            self.get_logger().error(
                'Timed out importing the PX4 parameter catalogue'
            )
            return False

        for name, value in self.values.items():
            if not self._set_parameter(name, value, deadline):
                self.get_logger().error(
                    f'Timed out setting PX4 parameter {name}'
                )
                return False

        ev_ctrl = self.values['EKF2_EV_CTRL']
        height_reference = self.values.get('EKF2_HGT_REF')
        height_message = (
            f'EKF2_HGT_REF={height_reference}'
            if height_reference is not None
            else 'EKF2_HGT_REF preserved'
        )
        self.get_logger().info(
            'PX4 external odometry fusion configured: '
            f'EKF2_EV_CTRL={ev_ctrl}, '
            f'{height_message}'
        )
        if height_reference is not None:
            self.get_logger().warning(
                'EKF2_HGT_REF is reboot-required. Restart PX4 if its value '
                'changed during this run.'
            )
        return True


def main():
    rclpy.init()
    node = ExternalOdometryConfigurator()
    success = node.configure()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
