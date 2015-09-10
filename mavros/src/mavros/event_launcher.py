# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import shlex
import rospy
import mavros
import os.path
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


class Launcher(object):
    __slots__ = [
        'handlers',
        'known_events',
        'triggers',
        'prev_armed',
        'state_sub',
    ]

    def __init__(self):
        self.handlers = []
        self.known_events = ['armed', 'disarmed']
        self.triggers = {}
        self.prev_armed = False

        self.state_sub = rospy.Subscriber(
            mavros.get_topic('state'),
            State,
            self.mavros_state_cb)

        try:
            params = rospy.get_param('~')
            if not isinstance(params, dict):
                raise KeyError("bad configuration")
        except KeyError as e:
            rospy.logerr('Config error: %s', e)
            return

        # load configuration
        for k, v in params.iteritems():
            # TODO: add checks for mutually exclusive options

            if v.has_key('service'):
                self._load_trigger(k, v)
            elif v.has_key('shell'):
                self._load_shell(k, v)
            elif v.has_key('rosrun'):
                self._load_rosrun(k, v)
            elif v.has_key('roslaunch'):
                self._load_roslaunch(k, v)
            else:
                rospy.logwarn("Skipping unknown section: %s", k)

        # check that events are known
        rospy.loginfo("Known events: %s", ', '.join(self.known_events))
        for h in self.handlers:
            for evt in h.events:
                if evt not in self.known_events:
                    rospy.logwarn("%s: unknown event: %s", h.name, evt)

    def _load_trigger(self, name, params):
        rospy.logdebug("Loading trigger: %s", name)

        def gen_cb(event):
            def cb(req):
                self(event)
                return TriggerResponse(success=True)    # try later to check success
            return cb

        self.known_events.append(name)
        self.triggers = rospy.Service(params['service'], Trigger, gen_cb(name))
        rospy.loginfo("Trigger: %s (%s)", name, params['service'])

    def _load_shell(self, name, params):
        rospy.logdebug("Loading shell: %s", name)

        events, actions = self._get_evt_act(params)

        args = params['shell']
        if not isinstance(args, list):
            args = shlex.split(args)

        command = os.path.expandvars(os.path.expanduser(args[0]))
        args = args[1:]

        handler = ShellHandler(name, command, args, events, actions)
        rospy.loginfo("Shell: %s (%s)", name, ' '.join([command] + [repr(v) for v in args]))
        self.handlers.append(handler)

    def _load_rosrun(self, name, params):
        pass

    def _load_roslaunch(self, name, params):
        pass

    def _get_evt_act(self, params):
        evt = self._param_to_list(params['event'])
        act = self._param_to_list(params['action'])
        if len(evt) != len(act):
            raise ValueError("event and action fileds has different size!")
        return evt, act

    def _param_to_list(self, str_or_list):
        if isinstance(str_or_list, list):
            return [it.strip() for it in str_or_list]
        else:
            ret = []
            for it in str_or_list.split():
                ret.extend([v.strip() for v in it.split(',') if v])
            return ret


    def __call__(self, event):
        rospy.logdebug('Event: %s', event)
        for h in self.handlers:
            h(event)

    def spin(self):
        if not self.handlers:
            rospy.logwarn("No event handlers defined, terminating.")
            return

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            for h in self.handlers:
                h.spin_once()

            rate.sleep()

    def mavros_state_cb(self, msg):
        if msg.armed != self.prev_armed:
            self.prev_armed = msg.armed
            self('armed' if msg.armed else 'disarmed')


def main():
    rospy.init_node("event_launcher")
    mavros.set_namespace()  # TODO initialize me

    rospy.loginfo("Starting event launcher...")

    launcher = Launcher()
    launcher.spin()

