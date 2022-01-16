# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rclpy
from geometry_msgs.msg import (
    AccelWithCovarianceStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
    TwistWithCovarianceStamped,
)
from nav_msgs.msg import Odometry

from .base import SENSOR_QOS, PluginModule, SubscriptionCallable


class LocalPositionPlugin(PluginModule):
    """Local position plugin."""

    def subscribe_pose(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            PoseStamped, ("local_position", "pose"), callback, qos_profile
        )

    def subscribe_pose_cov(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            PoseWithCovarianceStamped,
            ("local_position", "pose_cov"),
            callback,
            qos_profile,
        )

    def subscribe_velocity_local(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            TwistStamped, ("local_position", "velocity_local"), callback, qos_profile
        )

    def subscribe_velocity_body(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            TwistStamped, ("local_position", "velocity_body"), callback, qos_profile
        )

    def subscribe_velocity_body_cov(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            TwistWithCovarianceStamped,
            ("local_position", "velocity_body_cov"),
            callback,
            qos_profile,
        )

    def subscribe_accel(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            AccelWithCovarianceStamped,
            ("local_position", "accel"),
            callback,
            qos_profile,
        )

    def subscribe_odom(
        self, callback: SubscriptionCallable, qos_profile=SENSOR_QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            Odometry, ("local_position", "local"), callback, qos_profile
        )
