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

    def spin_once(self):
        raise NotImplementedError


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

        rospy.loginfo("Starting %s", self)

        args = [self.command] + self.args
        rospy.logdebug("Starting subprocess: %s", ' '.join(args))

        child_stdout = subprocess.PIPE
        child_stderr = subprocess.PIPE

        self.process = subprocess.Popen(args, stdout=child_stdout, stderr=child_stderr, close_fds=False)

        poll_result = self.process.poll()
        if poll_result is None or poll_result == 0:
            rospy.loginfo("Process '%s' started, pid: %s", self, self.process.pid)
        else:
            rospy.logerr("Failed to start '%s'", self)
            self.process = None

    def action_stop(self):
        if self.process:
            self._log_messages()
            rospy.loginfo("Stopping %s", self)
            self.process.terminate()
            self.process = None

    def _log_messages(self):
        for msg in self.process.stdout.readlines():
            rospy.loginfo("%s: %s", self, msg.strip())

        for msg in self.process.stderr.readlines():
            rospy.logerr("%s: %s", self, msg.strip())

    def spin_once(self):
        if self.process:
            self._log_messages()

            poll_result = self.process.poll()
            if poll_result is not None:
                rospy.loginfo("Process %s (pid %s) terminated, exit code: %s", self, self.process.pid, poll_result)
                self.process = None


class RosrunHandler(EventHandler):
    def __call__(self, event):
        raise NotImplementedError


class RoslaunchHandler(EventHandler):
    def __call__(self, event):
        raise NotImplementedError



def main():
    rospy.init_node("event_launcher")

    rospy.loginfo("Starting event launcher...")

    handlers = []

    # TODO: load config from rosparam

    if not handlers:
        rospy.logwarn("No event handlers defined, terminating.")
        return

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        for h in handlers:
            h.spin_once()

        rate.sleep()
