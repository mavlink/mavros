#!/usr/bin/env python3
#
# Copyright 2025 Haroon Rasheed.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#
# =============================================================================
# This Python script demonstrates how to manage waypoint missions using MAVROS.
# It provides examples of how to interact with MAVLink services and topics
# through ROS to perform mission management tasks such as:
#
#    • Uploading (pushing) waypoints to the flight controller.
#    • Downloading (pulling) existing mission waypoints.
#    • Clearing all mission waypoints from the flight controller.
#    • Setting a specific waypoint as the current active one.
#    • Changing flight mode to AUTO for mission execution.
#
# Note:
#    - This example only demonstrates the usage of MAVROS services and messages
#      related to waypoint missions.
#    - It assumes that the quadcopter is already airborne and hovering at its
#      home position before executing any mission-related operations.
#
# Covered Services (from mavros_msgs):
#    - WaypointPush.srv
#    - WaypointPull.srv
#    - WaypointClear.srv
#    - WaypointSetCurrent.srv
#    - SetMode.srv
#
# Related MAVROS Topics (waypoint mission related):
#    - /mavros/mission/waypoints         : Publishes current mission waypoints.
#    - /mavros/mission/reached           : Publishes notifications when a waypoint is reached.
#
# The code provides a practical foundation for autonomous waypoint mission
# management using MAVROS with ArduPilot-based flight controllers.
# =============================================================================

import typing as ty

import rclpy  # noqa: I100
from mavros_msgs.msg import (  # noqa: I100
    Waypoint,
    WaypointList,
    WaypointReached,
)
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class WaypointMissionManager(Node):
    """Manager node."""

    def __init__(self):
        super().__init__('waypoint_mission_manager')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Create service clients
        self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.set_current_client = self.create_client(
            WaypointSetCurrent, '/mavros/mission/set_current'
        )
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Create subscribers
        self.waypoints_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            qos_profile,
        )

        self.reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            qos_profile,
        )

        # State variables
        self.current_waypoints = []
        self.last_reached_wp = None

        self.get_logger().info('Waypoint Mission Manager initialized')
        self.wait_for_services()

    def wait_for_services(self):
        """Wait for all MAVROS services to become available."""
        self.get_logger().info('Waiting for MAVROS services...')

        services = [
            (self.push_client, 'WaypointPush'),
            (self.pull_client, 'WaypointPull'),
            (self.clear_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
        ]

        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

        self.get_logger().info('All MAVROS services are available!')

    def waypoints_callback(self, msg: WaypointList):
        self.current_waypoints = msg.waypoints
        # self.get_logger().info(f'Received {len(self.current_waypoints)} waypoints from mission')

    def waypoint_reached_callback(self, msg: WaypointReached):
        self.last_reached_wp = msg.wp_seq
        self.get_logger().info(f'Waypoint {msg.wp_seq} reached!')

    def set_mode(self, mode: str = 'AUTO') -> bool:
        """
        Set the flight mode.

        :param mode: Flight mode string (e.g., 'AUTO', 'GUIDED', 'STABILIZE', 'LOITER')
        :returns: True if successful, False otherwise
        """
        request = SetMode.Request()
        request.custom_mode = mode

        self.get_logger().info(f'Setting flight mode to {mode}...')

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Successfully set mode to {mode}')
                return True
            else:
                self.get_logger().error(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('SetMode service call failed')
            return False

    def create_waypoint(
        self,
        lat: float,
        lon: float,
        alt: float,
        frame: int = 0,
        command: int = 16,
        is_current: bool = False,
        autocontinue: bool = True,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
    ) -> Waypoint:
        """
        Create a MAVLink waypoint.

        :param lat: Latitude in degrees
        :param lon: Longitude in degrees
        :param alt: Altitude in meters
        :param frame: MAVLink frame (0=GLOBAL, 3=GLOBAL_RELATIVE_ALT, etc.)
        :param command: MAVLink command (16=WAYPOINT, 22=TAKEOFF, 21=LAND, etc.)
        :param is_current: Whether this is the current waypoint
        :param autocontinue: Auto-continue to next waypoint
        :param param1-4: Command-specific parameters
        :returns: Waypoint message
        """
        wp = Waypoint()
        wp.frame = frame
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = autocontinue
        wp.param1 = param1
        wp.param2 = param2
        wp.param3 = param3
        wp.param4 = param4
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt

        return wp

    def create_sample_mission(
        self, home_lat: float = -35.363262, home_lon: float = 149.165237, home_alt: float = 2.0
    ) -> ty.List[Waypoint]:
        """
        Create a sample square mission with takeoff and landing.

        :param home_lat: Home latitude
        :param home_lon: Home longitude
        :param home_alt: Home altitude
        :returns: List of waypoints
        """
        waypoints = []

        # Waypoint 0: Home position (required)
        wp_home = self.create_waypoint(
            home_lat,
            home_lon,
            home_alt,
            frame=0,  # GLOBAL
            command=16,  # WAYPOINT
            is_current=True,
        )
        waypoints.append(wp_home)

        # Waypoint 1: Takeoff
        wp_takeoff = self.create_waypoint(
            home_lat,
            home_lon,
            home_alt + 10.0,
            frame=3,  # GLOBAL_RELATIVE_ALT
            command=22,  # TAKEOFF
            param1=15.0,  # Pitch angle
        )
        waypoints.append(wp_takeoff)

        # Square pattern waypoints (30m sides, 10m altitude)
        offset = 0.0003  # Approximately 30m in lat/lon
        square_alt = 10.0  # Relative altitude

        # Waypoint 2: North
        wp2 = self.create_waypoint(
            home_lat + offset,
            home_lon,
            square_alt,
            frame=3,  # GLOBAL_RELATIVE_ALT
            command=16,  # WAYPOINT
        )
        waypoints.append(wp2)

        # Waypoint 3: North-East
        wp3 = self.create_waypoint(
            home_lat + offset, home_lon + offset, square_alt, frame=3, command=16
        )
        waypoints.append(wp3)

        # Waypoint 4: South-East
        wp4 = self.create_waypoint(home_lat, home_lon + offset, square_alt, frame=3, command=16)
        waypoints.append(wp4)

        # Waypoint 5: Return to home position
        wp5 = self.create_waypoint(home_lat, home_lon, square_alt, frame=3, command=16)
        waypoints.append(wp5)

        # Waypoint 6: Land
        wp_land = self.create_waypoint(
            home_lat,
            home_lon,
            0.0,
            frame=3,
            command=21,  # LAND
        )
        waypoints.append(wp_land)

        return waypoints

    def push_waypoints(self, waypoints: ty.List[Waypoint]) -> bool:
        """
        Upload waypoints to the flight controller.

        :param waypoints: List of Waypoint messages
        :returns: True if successful, False otherwise
        """
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints

        self.get_logger().info(f'Pushing {len(waypoints)} waypoints to FCU...')

        future = self.push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully pushed {response.wp_transfered} waypoints')
                return True
            else:
                self.get_logger().error('Failed to push waypoints')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False

    def pull_waypoints(self) -> ty.Optional[ty.List[Waypoint]]:
        """
        Download waypoints from the flight controller.

        :returns: List of waypoints or None if failed
        """
        request = WaypointPull.Request()

        self.get_logger().info('Pulling waypoints from FCU...')

        future = self.pull_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully pulled {response.wp_received} waypoints')
            else:
                self.get_logger().error('Failed to pull waypoints')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None

    def clear_waypoints(self) -> bool:
        """
        Clear all waypoints from the flight controller.

        :returns: True if successful, False otherwise
        """
        request = WaypointClear.Request()

        self.get_logger().info('Clearing all waypoints from FCU...')

        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully cleared all waypoints')
                return True
            else:
                self.get_logger().error('Failed to clear waypoints')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False

    def set_current_waypoint(self, wp_seq: int) -> bool:
        """
        Set the current active waypoint.

        :param wp_seq: Waypoint sequence number to set as current
        :returns: True if successful, False otherwise
        """
        request = WaypointSetCurrent.Request()
        request.wp_seq = wp_seq

        self.get_logger().info(f'Setting waypoint {wp_seq} as current...')

        future = self.set_current_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully set waypoint {wp_seq} as current')
                return True
            else:
                self.get_logger().error(f'Failed to set waypoint {wp_seq} as current')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False

    def print_waypoint_info(self, waypoints: ty.List[Waypoint]):
        """Print information about waypoints."""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total waypoints: {len(waypoints)}')
        self.get_logger().info('=' * 60)

        for i, wp in enumerate(waypoints):
            cmd_name = self.get_command_name(wp.command)
            frame_name = self.get_frame_name(wp.frame)

            self.get_logger().info(
                f'WP {i}: {cmd_name} | '
                f'Frame: {frame_name} | '
                f'Lat: {wp.x_lat:.6f}, Lon: {wp.y_long:.6f}, Alt: {wp.z_alt:.2f}m | '
                f'Current: {wp.is_current}'
            )
        self.get_logger().info('=' * 60)

    @staticmethod
    def get_command_name(command: int) -> str:
        """Convert MAVLink command number to name."""
        commands = {
            16: 'WAYPOINT',
            17: 'LOITER_UNLIM',
            18: 'LOITER_TURNS',
            19: 'LOITER_TIME',
            21: 'LAND',
            22: 'TAKEOFF',
            84: 'NAV_VTOL_TAKEOFF',
            85: 'NAV_VTOL_LAND',
        }
        return commands.get(command, f'CMD_{command}')

    @staticmethod
    def get_frame_name(frame: int) -> str:
        """Convert MAVLink frame number to name."""
        frames = {
            0: 'GLOBAL',
            3: 'GLOBAL_RELATIVE_ALT',
            6: 'GLOBAL_TERRAIN_ALT',
        }
        return frames.get(frame, f'FRAME_{frame}')


def main(args=None):
    rclpy.init(args=args)

    manager = WaypointMissionManager()

    try:
        # Wait a moment for initial connections
        import time

        time.sleep(2)

        manager.get_logger().info('=== Example 2: Basic Waypoint Mission ===')
        manager.get_logger().info('\n--- Clearing Existing Waypoints ---')
        manager.clear_waypoints()

        time.sleep(1)

        # Example 2: Create and push a sample mission
        manager.get_logger().info('\n--- Creating and Pushing Sample Mission ---')
        sample_mission = manager.create_sample_mission()
        manager.print_waypoint_info(sample_mission)
        manager.push_waypoints(sample_mission)

        time.sleep(1)

        # Example 3: Pull waypoints from FCU
        manager.get_logger().info('\n--- Pulling Waypoints from FCU ---')
        pulled_waypoints = manager.pull_waypoints()
        if pulled_waypoints:
            manager.print_waypoint_info(pulled_waypoints)

        time.sleep(1)

        # Example 4: Set a specific waypoint as current (skip to waypoint 2)
        manager.get_logger().info('\n--- Setting Current Waypoint ---')
        manager.set_current_waypoint(0)

        time.sleep(1)

        # Example 5: Set mode to AUTO
        manager.get_logger().info('\n--- Setting Mode to AUTO ---')
        manager.set_mode('AUTO')

        # Keep node alive to receive callbacks
        manager.get_logger().info('\nNode is running. Press Ctrl+C to exit.')
        manager.get_logger().info('Monitoring waypoint reached events...')

        rclpy.spin(manager)

    except KeyboardInterrupt:
        manager.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
