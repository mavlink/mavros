#!/usr/bin/env python3
#
# Copyright 2025.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#
# =============================================================================
# This script demonstrates OFFBOARD mode control using MAVROS in ROS 2.
# It communicates with PX4 through MAVROS to perform position control
# by continuously publishing setpoints and managing flight mode transitions.
#
#
# The following MAVROS services and topics are used:
#    • /mavros/cmd/arming           — to arm the UAV
#    • /mavros/set_mode             — to set OFFBOARD flight mode
#    • /mavros/state                — to monitor connection and flight state
#    • /mavros/setpoint_position/local — to publish position setpoints
#
# Note:
#    Setpoint publishing MUST be faster than 2Hz before entering OFFBOARD mode
#    and must continue at this rate to maintain OFFBOARD control.
# =============================================================================

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import rclpy
from rclpy.node import Node


class OffboardControl(Node):
    """OFFBOARD mode control node."""

    def __init__(self):
        super().__init__('offb_node_py')

        # Current state
        self.current_state = State()

        # State subscriber
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)

        # Local position publisher
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('MAVROS services are ready!')

        # Setpoint publishing MUST be faster than 2Hz
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

        # Target pose
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0

        # Control variables
        self.setpoint_counter = 0
        self.last_req_time = self.get_clock().now()
        self.offboard_requested = False
        self.arm_requested = False

    def state_cb(self, msg):
        """State callback."""
        self.current_state = msg

    def wait_for_services(self):
        """Wait for all services to be available."""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

    def set_mode(self, mode: str):
        """Set flight mode asynchronously."""
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def arm(self):
        """Arm the vehicle asynchronously."""
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    def land(self):
        """Command the vehicle to land by switching to AUTO.LAND mode."""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            self.get_logger().info('Land command sent')
        else:
            self.get_logger().warn('Land command timed out')

    def timer_callback(self):
        """Execute the main control loop callback."""
        # Wait for Flight Controller connection
        if not self.current_state.connected:
            return

        # Send a few setpoints before starting OFFBOARD mode
        if self.setpoint_counter < 100:
            self.local_pos_pub.publish(self.pose)
            self.setpoint_counter += 1
            return

        current_time = self.get_clock().now()
        time_since_last_req = (current_time - self.last_req_time).nanoseconds / 1e9

        # Try to enable OFFBOARD mode
        if self.current_state.mode != 'OFFBOARD' and time_since_last_req > 5.0:
            self.set_mode('OFFBOARD')
            if not self.offboard_requested:
                self.get_logger().info('OFFBOARD mode requested')
                self.offboard_requested = True
            self.last_req_time = current_time

        # Try to arm the vehicle
        elif not self.current_state.armed and time_since_last_req > 5.0:
            self.arm()
            if not self.arm_requested:
                self.get_logger().info('Arming requested')
                self.arm_requested = True
            self.last_req_time = current_time

        # Continue publishing setpoints
        self.local_pos_pub.publish(self.pose)


def main(args=None):
    rclpy.init(args=args)

    try:
        offboard_control = OffboardControl()

        # Wait for connection
        offboard_control.get_logger().info('Waiting for FCU connection...')
        while rclpy.ok() and not offboard_control.current_state.connected:
            rclpy.spin_once(offboard_control, timeout_sec=0.1)

        offboard_control.get_logger().info('FCU connected! Starting OFFBOARD control...')

        # Spin the node
        rclpy.spin(offboard_control)

    except KeyboardInterrupt:
        offboard_control.get_logger().info('OFFBOARD control interrupted by user')
        offboard_control.get_logger().info('Landing...')
        offboard_control.land()
    except Exception as e:
        print(f'An error occurred: {e}')
        if 'offboard_control' in locals():
            offboard_control.get_logger().info('Landing due to error...')
            offboard_control.land()
    finally:
        if 'offboard_control' in locals():
            offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
