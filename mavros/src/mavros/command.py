# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

__all__ = (
    'long',
    'int',
    'arming',
    'set_home',
    'takeoff',
    'land',
    'trigger_control',
)

import rospy
import mavros

from mavros_msgs.srv import CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval


def _get_proxy(service, type):
    return rospy.ServiceProxy(mavros.get_topic('cmd', service), type)


long = None
int = None
arming = None
set_home = None
takeoff = None
land = None
trigger_control = None


def _setup_services():
    global long, int, arming, set_home, takeoff, land, trigger_control
    long = _get_proxy('command', CommandLong)
    int = _get_proxy('command_int', CommandInt)
    arming = _get_proxy('arming', CommandBool)
    set_home = _get_proxy('set_home', CommandHome)
    takeoff = _get_proxy('takeoff', CommandTOL)
    land = _get_proxy('land', CommandTOL)
    trigger_control = _get_proxy('trigger_control', CommandTriggerControl)
    trigger_interval = _get_proxy('trigger_interval', CommandTriggerInterval)


# register updater
mavros.register_on_namespace_update(_setup_services)
