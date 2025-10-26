#!/usr/bin/env python3
#
# Copyright 2025 Haroon Rasheed.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#
# =============================================================================
# This script demonstrates UAV task control using MAVROS services in ROS 2.
# It communicates with ArduPilot through MAVROS to perform flight operations
# such as arming, changing flight modes, takeoff, landing, and setting home.
#
# The following MAVROS services are used:
#    • /mavros/cmd/arming      — to arm or disarm the UAV
#    • /mavros/set_mode        — to set the flight mode (e.g., GUIDED, RTL, LAND)
#    • /mavros/cmd/takeoff     — to initiate takeoff
#    • /mavros/cmd/land        — to command landing
#    • /mavros/cmd/set_home    — to set the home position for the UAV
#
# Note:
#    It is assumed that all pre-arm checks (such as GPS lock, RC calibration,
#    battery level, and sensor health) are passed successfully before executing
#    this script.
# =============================================================================

import rclpy  # noqa: I100
from geometry_msgs.msg import PoseStamped  # noqa: F401,I100
from mavros_msgs.msg import State  # noqa: F401
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode
from rclpy.node import Node


class TaskControl(Node):
    """Control node."""

    def __init__(self):
        super().__init__('task_control')

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('MAVROS services are ready!')

    def wait_for_services(self):
        """Wait for all services to be available."""
        services = [
            self.arming_client,
            self.set_mode_client,
            self.takeoff_client,
            self.land_client,
            self.set_home_client,
        ]

        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')

    def arm(self) -> bool:
        """Arm the quadcopter."""
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle armed successfully')
                return True
            else:
                self.get_logger().warn('Failed to arm vehicle')
                return False
        else:
            self.get_logger().error('Arming service call failed')
            return False

    def disarm(self) -> bool:
        """Disarm the quadcopter."""
        req = CommandBool.Request()
        req.value = False

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle disarmed successfully')
                return True
            else:
                self.get_logger().warn('Failed to disarm vehicle')
                return False
        else:
            self.get_logger().error('Disarming service call failed')
            return False

    def set_mode(self, mode: str) -> bool:
        """
        Set flight mode.

        Common modes: STABILIZED, GUIDED, RTL, LAND
        """
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().warn(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('Set mode service call failed')
            return False

    def takeoff(self, altitude: float) -> bool:
        """
        Takeoff to specified altitude.

        :param altitude: in meters
        """
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0  # Use current position
        req.longitude = 0.0
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Takeoff command sent. Target altitude: {altitude}m')
                return True
            else:
                self.get_logger().warn('Takeoff command failed')
                return False
        else:
            self.get_logger().error('Takeoff service call failed')
            return False

    def land(self) -> bool:
        """Land the quadcopter."""
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Landing command sent')
                return True
            else:
                self.get_logger().warn('Landing command failed')
                return False
        else:
            self.get_logger().error('Land service call failed')
            return False

    def set_home_current(self) -> bool:
        """Set home position to current location."""
        req = CommandHome.Request()
        req.current_gps = True

        future = self.set_home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Home position set to current location')
                return True
            else:
                self.get_logger().warn('Failed to set home position')
                return False
        else:
            self.get_logger().error('Set home service call failed')
            return False


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create TaskControl instance
        task_control = TaskControl()

        task_control.get_logger().info('=== Example 1: Basic Flight Sequence ===')
        import time

        time.sleep(2)

        # Set home position
        task_control.get_logger().info('Setting home position to current location...')
        if not task_control.set_home_current():
            task_control.get_logger().warn('Failed to set home position, continuing anyway...')
        time.sleep(1)

        # Set to GUIDED mode
        task_control.get_logger().info('Setting mode to GUIDED...')
        if not task_control.set_mode('GUIDED'):
            task_control.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        time.sleep(2)

        # Arm the vehicle
        task_control.get_logger().info('Arming the vehicle...')
        if not task_control.arm():
            task_control.get_logger().error('Failed to arm vehicle. Exiting...')
            return
        time.sleep(2)

        # Takeoff to 5 meters
        task_control.get_logger().info('Taking off to 5 meters...')
        if not task_control.takeoff(5.0):
            task_control.get_logger().error('Failed to send takeoff command. Landing...')
            task_control.land()
            return
        task_control.get_logger().info('Drone is taking off...')
        time.sleep(10)  # Wait for takeoff to complete

        task_control.get_logger().info('=== Example 1 Complete ===')

    except KeyboardInterrupt:
        task_control.get_logger().info('Flight interrupted by user')
    except Exception as e:
        task_control.get_logger().error(f'An error occurred: {e}')
    finally:
        task_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
