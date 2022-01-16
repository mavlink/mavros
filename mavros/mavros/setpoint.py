# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory

from mavros_msgs.msg import AttitudeTarget, GlobalPositionTarget, PositionTarget, Thrust

from .base import SENSOR_QOS, PluginModule, SubscriptionCallable, cached_property

QOS = SENSOR_QOS


class SetpointAccelPlugin(PluginModule):
    @cached_property
    def pub_accel(self) -> rclpy.node.Publisher:
        return self.create_publisher(Vector3Stamped, ("setpoint_accel", "accel"), QOS)


class SetpointAttitudePlugin(PluginModule):
    @cached_property
    def pub_attitude(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            PoseStamped, ("setpoint_attitude", "attitude"), QOS
        )

    @cached_property
    def pub_cmd_vel(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            TwistStamped, ("setpoint_attitude", "cmd_vel"), QOS
        )

    @cached_property
    def pub_thrust(self) -> rclpy.node.Publisher:
        return self.create_publisher(Thrust, ("setpoint_attitude", "thrust"), QOS)


class SetpointPositionPlugin(PluginModule):
    @cached_property
    def pub_local(self) -> rclpy.node.Publisher:
        return self.create_publisher(PoseStamped, ("setpoint_position", "local"), QOS)

    @cached_property
    def pub_global(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            GeoPoseStamped, ("setpoint_position", "global"), QOS
        )

    @cached_property
    def pub_global_to_local(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            GeoPoseStamped, ("setpoint_position", "global_to_local"), QOS
        )


class SetpointRawPlugin(PluginModule):
    @cached_property
    def pub_local(self) -> rclpy.node.Publisher:
        return self.create_publisher(PositionTarget, ("setpoint_raw", "local"), QOS)

    @cached_property
    def pub_global(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            GlobalPositionTarget, ("setpoint_raw", "global"), QOS
        )

    @cached_property
    def pub_attitude(self) -> rclpy.node.Publisher:
        return self.create_publisher(AttitudeTarget, ("setpoint_raw", "attitude"), QOS)

    def subscribe_target_local(
        self, callback: SubscriptionCallable, qos_profile=QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            PositionTarget, ("setpoint_raw", "target_local"), callback, qos_profile
        )

    def subscribe_target_global(
        self, callback: SubscriptionCallable, qos_profile=QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            GlobalPositionTarget,
            ("setpoint_raw", "target_global"),
            callback,
            qos_profile,
        )

    def subscribe_target_attitude(
        self, callback: SubscriptionCallable, qos_profile=QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            AttitudeTarget, ("setpoint_raw", "target_attitude"), callback, qos_profile
        )


class SetpointTrajectoryPlugin(PluginModule):
    @cached_property
    def pub_local(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            MultiDOFJointTrajectory, ("setpoint_trajectory", "local"), QOS
        )

    def subscribe_desired(
        self, callback: SubscriptionCallable, qos_profile=QOS
    ) -> rclpy.node.Subscription:
        return self.create_subscription(
            Path, ("setpoint_trajectory", "desired"), callback, qos_profile
        )

    @cached_property
    def reset(self) -> rclpy.node.Client:
        return self.create_client(Trigger, ("setpoint_trajectory", "reset"))


class SetpointVelocityPlugin(PluginModule):
    @cached_property
    def pub_cmd_vel(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            TwistStamped, ("setpoint_velocity", "cmd_vel"), QOS
        )

    @cached_property
    def pub_cmd_vel_unstamped(self) -> rclpy.node.Publisher:
        return self.create_publisher(
            Twist, ("setpoint_velocity", "cmd_vel_unstamped"), QOS
        )
