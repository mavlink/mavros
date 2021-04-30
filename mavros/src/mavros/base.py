# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from functools import cached_property

import rclpy

DEFAULT_NAMESPACE = 'mavros'

# STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
STATE_QOS = rclpy.QoSProfile(history=10,
                             durability=rclpy.QoSDurability.TRANSIENT_LOCAL)

# SENSOR_QOS used for most of sensor streams
SENSOR_QOS = rclpy.QoSPresetProfiles.SENSOR_DATA


class BaseNode(rclpy.Node):
    """
    Base class for mavros client object. It's used to hide plugin parameters.
    """

    _ns: str

    def __init__(self, node_name: str, mavros_ns: str = DEFAULT_NAMESPACE):
        super().__init__(node_name)

    def get_topic(self, *args: str) -> str:
        return '/'.join((self._ns, ) + args)


class PluginModule:
    """
    PluginModule is a base class for modules used to talk to mavros plugins
    """

    _node: BaseNode

    def __init__(self, parent_node: BaseNode):
        self._node = parent_node
