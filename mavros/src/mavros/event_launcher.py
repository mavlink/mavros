# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import rospy
import subprocess
from roslaunch.scriptapi import ROSLaunch, Node

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from mavros_msgs.msg import State


class EventHandler(object):
    """
    Base class for event handlers
    """

    __slots__ = [
        'name', 'process', 'args', 'events', 'actions'
    ]

    def __init__(self, name, args=[], events=[], actions=[]):
        self.name = name
        self.process = None
        self.args = args
        self.events = events
        self.actions = actions

    def __str__(self):
        return self.name

    def __call__(self, event):
        if event not in self.events:
            return  # not our event

        idx = self.events.index(event)
        action = self.actions[idx]

        if hasattr(self, 'action_' + action):
            getattr(self, 'action_' + action)()
        else:
            rospy.logerr("Misconfiguration of '{}', there no action '{}'".format(
                str(self), action))


class ShellHandler(EventHandler):
    """
    Simple program runner
    """

    __slots__ = [
        'command'
    ]

    def __init__(self, name, command, args=[], events=[], actions=[]):
        super(ShellHandler, self).__init__(name, args, events, actions)
        self.command = command

    def action_run(self):
        if self.process:
            rospy.loginfo("Process %s still active, terminating before run", str(self))
            self.action_stop()

        cmd = [self.command] + self.args

    def action_stop(self):
        pass


class RosrunHandler(EventHandler):
    pass


class RoslaunchHandler(EventHandler):
    pass


def main():
    rospy.init_node("event_launcher")

    rospy.loginfo("Starting event launcher...")
