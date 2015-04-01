# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

__all__ = (
    'arming',
)

import rospy
import mavros

from mavros.srv import CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL

def _get_proxy(service, type):
    return rospy.ServiceProxy(mavros.get_topic('cmd', service), type)



long = None
int = None
arming = None
set_home = None
takeoff = None
land = None
guided_enable = None


def setup_services():
    global long, int, arming, set_home, takeoff, land, guided_enable
    long = _get_proxy('command', CommandLong)
    int = _get_proxy('command_int', CommandInt)
    arming = _get_proxy('arming', CommandBool)
    set_home = _get_proxy('set_home', CommandHome)
    takeoff = _get_proxy('takeoff', CommandTOL)
    land = _get_proxy('land', CommandTOL)
    guided_enable = _get_proxy('guided_enable', CommandBool)


# preinit defaults
setup_services()
