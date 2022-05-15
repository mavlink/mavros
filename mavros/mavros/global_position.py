# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, UInt32

from .base import SENSOR_QOS, PluginModule, SubscriptionCallable


class GlobalPositionPlugin(PluginModule):
    """Global position plugin."""

    def subscribe_raw_fix(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            NavSatFix, ("global_position", "raw", "fix"), callback, qos_profile
        )

    def subscribe_raw_gps_vel(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            TwistStamped, ("global_position", "raw", "gps_vel"), callback, qos_profile
        )

    def subscribe_raw_satellites(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            UInt32, ("global_position", "raw", "satellites"), callback, qos_profile
        )

    def subscribe_fix(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            NavSatFix, ("global_position", "global"), callback, qos_profile
        )

    def subscribe_odom(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            Odometry, ("global_position", "local"), callback, qos_profile
        )

    def subscribe_rel_alt(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            Float64, ("global_position", "rel_alt"), callback, qos_profile
        )

    def subscribe_compass_hdg(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            Float64, ("global_position", "compass_hdg"), callback, qos_profile
        )
