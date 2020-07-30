#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import argparse
import threading

import rospy
import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, MessageInterval

def do_mode(args):
    base_mode = 0
    custom_mode = ''

    if args.custom_mode is not None:
        custom_mode = args.custom_mode.upper()
    if args.base_mode is not None:
        base_mode = args.base_mode

    done_evt = threading.Event()
    def state_cb(state):
        print_if(args.verbose, "Current mode:", state.mode)
        if state.mode == custom_mode:
            print("Mode changed.")
            done_evt.set()

    if custom_mode != '' and not custom_mode.isdigit():
        # with correct custom mode we can wait until it changes
        sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
    else:
        done_evt.set()

    try:
        set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
        ret = set_mode(base_mode=base_mode, custom_mode=custom_mode)
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.mode_sent:
        fault("Request failed. Check mavros logs")

    if not done_evt.wait(5):
        fault("Timed out!")



def do_rate(args):
    set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
    def _test_set_rate(rate_arg, id):
        if rate_arg is not None:
            try:
                set_rate(stream_id=id, message_rate=rate_arg, on_off=(rate_arg != 0))
            except rospy.ServiceException as ex:
                fault(ex)

    _test_set_rate(args.all, StreamRateRequest.STREAM_ALL)
    _test_set_rate(args.raw_sensors, StreamRateRequest.STREAM_RAW_SENSORS)
    _test_set_rate(args.ext_status, StreamRateRequest.STREAM_EXTENDED_STATUS)
    _test_set_rate(args.rc_channels, StreamRateRequest.STREAM_RC_CHANNELS)
    _test_set_rate(args.raw_controller, StreamRateRequest.STREAM_RAW_CONTROLLER)
    _test_set_rate(args.position, StreamRateRequest.STREAM_POSITION)
    _test_set_rate(args.extra1, StreamRateRequest.STREAM_EXTRA1)
    _test_set_rate(args.extra2, StreamRateRequest.STREAM_EXTRA2)
    _test_set_rate(args.extra3, StreamRateRequest.STREAM_EXTRA3)

    if args.stream_id is not None:
        _test_set_rate(args.stream_id[1], args.stream_id[0])



def do_message_interval(args):

    if args.id is None:
        fault("id not specified")
        return

    if args.rate is None:
        fault("rate not specified")
        return

    try:
        set_message_interval = rospy.ServiceProxy(mavros.get_topic('set_message_interval'), MessageInterval)
        set_message_interval(message_id=args.id, message_rate=args.rate)
    except rospy.ServiceException as ex:
        fault(ex)



def main():
    parser = argparse.ArgumentParser(description="Change mode and rate on MAVLink device.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default=mavros.DEFAULT_NAMESPACE)
    parser.add_argument('-v', '--verbose', action='store_true', help="verbose output")
    parser.add_argument('--wait', action='store_true', help="Wait for establishing FCU connection")
    subarg = parser.add_subparsers()

    mode_args = subarg.add_parser('mode', help="Set mode", formatter_class=argparse.RawTextHelpFormatter)
    mode_args.set_defaults(func=do_mode)
    mode_group = mode_args.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('-b', '--base-mode', type=int, help="Base mode code")
    mode_group.add_argument('-c', '--custom-mode', type=str, help="Custom mode string (same as in ~/state topic)\
        \n\nNOTE: For PX4 Pro AUTO.LOITER, disable RC failsafe, which can be done by setting 'NAV_RCL_ACT' parameter to 0.")

    rate_args = subarg.add_parser('rate', help="Set stream rate")
    rate_args.set_defaults(func=do_rate)
    rate_args.add_argument('--all', type=int, metavar='rate', help="All streams")
    rate_args.add_argument('--raw-sensors', type=int, metavar='rate', help="raw sensors stream")
    rate_args.add_argument('--ext-status', type=int, metavar='rate', help="extended status stream")
    rate_args.add_argument('--rc-channels', type=int, metavar='rate', help="RC channels stream")
    rate_args.add_argument('--raw-controller', type=int, metavar='rate', help="raw conptoller stream")
    rate_args.add_argument('--position', type=int, metavar='rate', help="position stream")
    rate_args.add_argument('--extra1', type=int, metavar='rate', help="Extra 1 stream")
    rate_args.add_argument('--extra2', type=int, metavar='rate', help="Extra 2 stream")
    rate_args.add_argument('--extra3', type=int, metavar='rate', help="Extra 3 stream")
    rate_args.add_argument('--stream-id', type=int, nargs=2, metavar=('id', 'rate'), help="any stream")

    message_interval_args = subarg.add_parser('message_interval', help="Set message interval")
    message_interval_args.set_defaults(func=do_message_interval)
    message_interval_args.add_argument('--id', type=int, metavar='id', help="message id")
    message_interval_args.add_argument('--rate', type=float, metavar='rate', help="message rate")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mavsys", anonymous=True)
    mavros.set_namespace(args.mavros_ns)

    if args.wait:
        wait_fcu_connection()

    args.func(args)


if __name__ == '__main__':
    main()
