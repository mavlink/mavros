# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import os
import sys

import rospy
import mavros
import threading


def print_if(cond, *args, **kvargs):
    if cond:
        print(*args, **kvargs)


def fault(*args, **kvargs):
    kvargs['file'] = sys.stderr
    print(*args, **kvargs)
    sys.exit(1)


def wait_fcu_connection(timeout=None):
    """
    Wait until establishing FCU connection
    """
    from mavros_msgs.msg import State
    try:
        msg = rospy.wait_for_message(mavros.get_topic('state'), State, timeout)
        if msg.connected:
            return True
    except rospy.ROSException as e:
        return False

    connected = threading.Event()
    def handler(msg):
        if msg.connected:
            connected.set()

    sub = rospy.Subscriber(
        mavros.get_topic('state'),
        State,
        handler
    )

    return connected.wait(timeout)
