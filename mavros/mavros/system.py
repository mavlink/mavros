# -*- python -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import threading
import typing

import rclpy
from sensor_msgs.msg import BatteryState

from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, StatusText
from mavros_msgs.srv import MessageInterval, SetMode, StreamRate, VehicleInfoGet

from .base import (
    SENSOR_QOS,
    STATE_QOS,
    PluginModule,
    SubscriptionCallable,
    cached_property,
)


class SystemPlugin(PluginModule):
    """System plugin."""

    def subscribe_state(
        self, callback: SubscriptionCallable, qos_profile=STATE_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(State, "state", callback, qos_profile)

    def subscribe_extended_state(
        self, callback: SubscriptionCallable, qos_profile=STATE_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            ExtendedState, "extended_state", callback, qos_profile
        )

    def subscribe_estimator_status(
        self, callback: SubscriptionCallable, qos_profile=STATE_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            EstimatorStatus, "estimator_status", callback, qos_profile
        )

    def subscribe_battery_state(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(BatteryState, "battery", callback, qos_profile)

    def subscribe_statustest(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            StatusText, ("statustext", "recv"), callback, qos_profile
        )

    @cached_property
    def pub_statustext(
        self,
    ) -> rclpy.node.Publisher:
        return self.create_publisher(StatusText, ("statustext", "send"), SENSOR_QOS)

    @cached_property
    def cli_set_mode(self) -> rclpy.node.Client:
        return self.create_client(SetMode, "set_mode")

    @cached_property
    def cli_set_stream_rate(self) -> rclpy.node.Client:
        return self.create_client(StreamRate, "set_stream_rate")

    @cached_property
    def cli_set_message_interval(self) -> rclpy.node.Client:
        return self.create_client(MessageInterval, "set_message_interval")

    @cached_property
    def cli_get_vehicle_info(self) -> rclpy.node.Client:
        return self.create_client(VehicleInfoGet, "vehicle_info_get")

    def wait_fcu_connection(self, timeout: typing.Optional[float] = None) -> bool:
        """Wait until establishing FCU connection."""
        connected = threading.Event()

        def handler(msg: State):
            self.get_logger().debug(f"got state: {msg}")
            if msg.connected:
                connected.set()

        self.subscribe_state(handler)
        return connected.wait(timeout)
