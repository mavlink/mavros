# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import os
import sys
import time
import shlex
import rospy
import mavros
import signal
import argparse
import threading
import subprocess

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from mavros_msgs.msg import State


class EventHandler(object):
    """
    Base class for event handlers
    """

    __slots__ = [
        'name', 'events', 'actions', 'lock',
    ]

    def __init__(self, name, events=[], actions=[]):
        self.name = name
        self.events = events
        self.actions = actions
        self.lock = threading.RLock()

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
            rospy.logerr("Misconfiguration of %s, there no action '%s'",
                         self, action)

    def spin_once(self):
        raise NotImplementedError


class ShellHandler(EventHandler):
    """
    Simple program runner
    """

    # XXX TODO:
    #   simple readlines() from Popen.stdout and .stderr simply don't work - it blocks
    #   so need to find a way for such logging, but for now i don't process process output.

    __slots__ = [
        'process', 'command', 'args', 'logfile',
    ]

    def __init__(self, name, command, args=[], logfile=None, events=[], actions=[]):
        super(ShellHandler, self).__init__(name, events, actions)
        self.process = None
        self.command = command
        self.args = args
        self.logfile = logfile

    def action_run(self):
        with self.lock:
            if self.process:
                rospy.loginfo("%s: process still active, terminating before run", str(self))
                self.action_stop()

            rospy.loginfo("%s: starting...", self)

            args = [self.command] + self.args
            if self.logfile:
                child_stdout = open(self.logfile, 'a')
                child_stderr = subprocess.STDOUT
            else:
                child_stdout = subprocess.PIPE
                child_stderr = subprocess.PIPE

            if hasattr(child_stdout, 'write'):
                child_stdout.write("\n--- run cut: %s ---\n" % time.ctime())

            self.process = subprocess.Popen(args, stdout=child_stdout, stderr=child_stderr,
                                            close_fds=True, preexec_fn=os.setsid)

            poll_result = self.process.poll()
            if poll_result is None or poll_result == 0:
                rospy.loginfo("%s: started, pid: %s", self, self.process.pid)
            else:
                rospy.logerr("Failed to start '%s'", self)
                self.process = None

    def action_stop(self):
        with self.lock:
            if not self.process:
                return

            rospy.loginfo("%s: stoppig...", self)

            # code from roslaunch.nodeprocess _stop_unix
            pid = self.process.pid
            pgid = os.getpgid(pid)

            def poll_timeout(timeout):
                timeout_t = time.time() + timeout
                retcode = self.process.poll()
                while time.time() < timeout_t and retcode is None:
                    time.sleep(0.1)
                    retcode = self.process.poll()
                return retcode

            try:
                rospy.loginfo("%s: sending SIGINT to pid [%s] pgid [%s]", self, pid, pgid)
                os.killpg(pgid, signal.SIGINT)

                retcode = poll_timeout(15.0)
                if retcode is None:
                    rospy.logwarn("%s: escalating to SIGTERM", self)
                    os.killpg(pgid, signal.SIGTERM)

                    retcode = poll_timeout(2.0)
                    if retcode is None:
                        rospy.logerr("%s: escalating to SIGKILL, may still be running", self)
                        try:
                            os.killpg(pgid, signal.SIGKILL)
                        except OSError as ex:
                            rospy.logerr("%s: %s", self, ex)

                if retcode is not None:
                    rospy.loginfo("%s: process (pid %s) terminated, exit code: %s",
                                  self, pid, retcode)
            finally:
                self.process = None

    def spin_once(self):
        with self.lock:
            if self.process:
                poll_result = self.process.poll()
                if poll_result is not None:
                    rospy.loginfo("%s: process (pid %s) terminated, exit code: %s",
                                  self, self.process.pid, poll_result)
                    self.process = None


# NOTE: here was RosrunHandler and RoslaunchHandler
#       but roslaunch.scriptapi can't launch node from worker thread
#       so i decided to leave only ShellHandler.


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

            if 'service' in v:
                self._load_trigger(k, v)
            elif 'shell' in v:
                self._load_shell(k, v)
            else:
                rospy.logwarn("Skipping unknown section: %s", k)

        # check that events are known
        rospy.loginfo("Known events: %s", ', '.join(self.known_events))
        for h in self.handlers:
            for evt in h.events:
                if evt not in self.known_events:
                    rospy.logwarn("%s: unknown event: %s", h.name, evt)

        # config loaded, we may subscribe
        self.state_sub = rospy.Subscriber(
            mavros.get_topic('state'),
            State,
            self.mavros_state_cb)

    def _load_trigger(self, name, params):
        rospy.logdebug("Loading trigger: %s", name)

        def gen_cb(event):
            def cb(req):
                self(event)
                return TriggerResponse(success=True)    # try later to check success
            return cb

        self.known_events.append(name)
        self.triggers[name] = rospy.Service(params['service'], Trigger, gen_cb(name))
        rospy.loginfo("Trigger: %s (%s)", name, params['service'])

    def _load_shell(self, name, params):
        rospy.logdebug("Loading shell: %s", name)

        events, actions = self._get_evt_act(params)

        def expandpath(p):
            return os.path.expanduser(os.path.expandvars(p))

        args = params['shell']
        if not isinstance(args, list):
            args = shlex.split(args)

        command = expandpath(args[0])
        args = args[1:]

        logfile = params.get('logfile')
        if logfile:
            logfile = expandpath(logfile)

        rospy.loginfo("Shell: %s (%s)", name, ' '.join([command] + [repr(v) for v in args]))
        if logfile:
            rospy.loginfo("Log: %s -> %s", name, logfile)

        handler = ShellHandler(name, command, args, logfile, events, actions)
        self.handlers.append(handler)

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
            try:
                h(event)
            except Exception as ex:
                import traceback
                rospy.logerr("Event %s -> %s exception: %s", event, h, ex)
                rospy.logerr(traceback.format_exc())

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
    parser = argparse.ArgumentParser(
        description="This node launch/terminate processes on events.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default=mavros.DEFAULT_NAMESPACE)

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("event_launcher")
    mavros.set_namespace(args.mavros_ns)

    rospy.loginfo("Starting event launcher...")

    launcher = Launcher()
    launcher.spin()
