# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rclpy
from mavros_msgs.srv import (CommandBool, CommandHome, CommandInt, CommandLong,
                             CommandTOL, CommandTriggerControl,
                             CommandTriggerInterval)

from .base import PluginModule, cached_property


class CommandPlugin(PluginModule):
    """
    Interface to command plugin
    """

    @cached_property
    def long(self) -> rclpy.Client:
        return self._node.create_client(
            CommandLong, self._node.get_topic('cmd', 'command'))

    @cached_property
    def int(self) -> rclpy.Client:
        return self._node.create_client(
            CommandInt, self._node.get_topic('cmd', 'command_int'))

    @cached_property
    def arming(self) -> rclpy.Client:
        return self._node.create_client(
            CommandBool, self._node.get_topic('cmd', 'arming'))

    @cached_property
    def set_home(self) -> rclpy.Client:
        return self._node.create_client(
            CommandHome, self._node.get_topic('cmd', 'set_home'))

    @cached_property
    def takeoff(self) -> rclpy.Client:
        return self._node.create_client(
            CommandTOL, self._node.get_topic('cmd', 'takeoff'))

    @cached_property
    def land(self) -> rclpy.Client:
        return self._node.create_client(
            CommandTOL, self._node.get_topic('cmd', 'land'))

    @cached_property
    def trigger_control(self) -> rclpy.Client:
        return self._node.create_client(
            CommandTriggerControl,
            self._node.get_topic('cmd', 'trigger_control'))

    @cached_property
    def trigger_interval(self) -> rclpy.Client:
        return self._node.create_client(
            CommandTriggerInterval,
            self._node.get_topic('cmd', 'trigger_interval'))
