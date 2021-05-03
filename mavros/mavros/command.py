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
    def long(self) -> rclpy.node.Client:
        return self.create_client(CommandLong, ('cmd', 'command'))

    @cached_property
    def int(self) -> rclpy.node.Client:
        return self.create_client(CommandInt, ('cmd', 'command_int'))

    @cached_property
    def arming(self) -> rclpy.node.Client:
        return self.create_client(CommandBool, ('cmd', 'arming'))

    @cached_property
    def set_home(self) -> rclpy.node.Client:
        return self.create_client(CommandHome, ('cmd', 'set_home'))

    @cached_property
    def takeoff(self) -> rclpy.node.Client:
        return self.create_client(CommandTOL, ('cmd', 'takeoff'))

    @cached_property
    def land(self) -> rclpy.node.Client:
        return self.create_client(CommandTOL, ('cmd', 'land'))

    @cached_property
    def trigger_control(self) -> rclpy.node.Client:
        return self.create_client(CommandTriggerControl,
                                  ('cmd', 'trigger_control'))

    @cached_property
    def trigger_interval(self) -> rclpy.node.Client:
        return self.create_client(CommandTriggerInterval,
                                  ('cmd', 'trigger_interval'))
