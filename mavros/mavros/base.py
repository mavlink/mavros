# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import random
import string
import typing
from functools import cached_property  # noqa F401

import rclpy  # noqa F401
import rclpy.node
import rclpy.qos

DEFAULT_NAMESPACE = 'mavros'
DEFAULT_NODE_NAME_PREFIX = 'mavpy_'

# STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)

# SENSOR_QOS used for most of sensor streams
SENSOR_QOS = rclpy.qos.QoSPresetProfiles.SENSOR_DATA


class BaseNode(rclpy.node.Node):
    """
    Base class for mavros client object. It's used to hide plugin parameters.
    """

    _ns: str

    def __init__(self,
                 node_name: typing.Optional[str] = None,
                 mavros_ns: str = DEFAULT_NAMESPACE):
        """
        :param node_name: name of the node, would be random if None
        :param mavros_ns: node name of mavros::UAS
        """
        if node_name is None:
            node_name = DEFAULT_NODE_NAME_PREFIX + ''.join(
                random.choices(string.ascii_lowercase + string.digits, k=4))

        super().__init__(node_name)
        self._ns = mavros_ns

    def get_topic(self, *args: str) -> str:
        return '/'.join((self._ns, ) + args)


class PluginModule:
    """
    PluginModule is a base class for modules used to talk to mavros plugins
    """

    _node: BaseNode

    def __init__(self, parent_node: BaseNode):
        self._node = parent_node
