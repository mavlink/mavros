# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import threading

import rclpy

from . import command, mission
from .base import STATE_QOS, BaseNode, cached_property


class Client(BaseNode):
    """
    Client provides some convinient methods to work with MAVROS API
    """
    @cached_property
    def command(self) -> command.CommandPlugin:
        return command.CommandPlugin(self)

    @cached_property
    def waypoint(self) -> mission.WaypointPlugin:
        return mission.WaypointPlugin(self)

    @cached_property
    def geofence(self) -> mission.GeofencePlugin:
        return mission.GeofencePlugin(self)

    @cached_property
    def rallypoint(self) -> mission.RallypointPlugin:
        return mission.RallypointPlugin(self)

    def wait_fcu_connection(self, timeout=None):
        """
        Wait until establishing FCU connection
        """

        from mavros_msgs.msg import State

        connected = threading.Event()

        def handler(msg: State):
            self.get_logger().debug(f"got state: {msg}")
            if msg.connected:
                connected.set()

        self.create_subscription(State, self.get_topic('state'), handler,
                                 STATE_QOS)

        return connected.wait(timeout)
