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
from mavros import mission as M

no_prettytable = False
try:
    from prettytable import PrettyTable
except ImportError:
    print("Waring: 'show' action disabled. install python-prettytable", file=sys.stderr)
    no_prettytable = True


def get_wp_file_io(args):
    return M.QGroundControlWP()


def _pull(args):
    try:
        ret = M.pull()
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed. Check mavros logs")

    print_if(args.verbose, "Waypoints received:", ret.wp_received)
    return ret


def do_pull(args):
    _pull(args)


def do_show(args):
    str_bool = lambda x: 'Yes' if x else 'No'
    str_frame = lambda f: M.FRAMES.get(f, 'UNK') + ' ({})'.format(f)
    str_command = lambda c: M.NAV_CMDS.get(c, 'UNK') + ' ({})'.format(c)

    done_evt = threading.Event()
    def _show_table(topic):
        pt = PrettyTable(('#', 'Curr', 'Auto',
                          'Frame', 'Command',
                          'P1', 'P2', 'P3', 'P4',
                          'X Lat', 'Y Long', 'Z Alt'))

        for seq, w in enumerate(topic.waypoints):
            pt.add_row((
                seq,
                str_bool(w.is_current),
                str_bool(w.autocontinue),
                str_frame(w.frame),
                str_command(w.command),
                w.param1,
                w.param2,
                w.param3,
                w.param4,
                w.x_lat,
                w.y_long,
                w.z_alt
            ))

        print(pt, file=sys.stdout)
        sys.stdout.flush()
        done_evt.set()

    if args.pull:
        _pull(args)

    # Waypoints topic is latched type, and it updates after pull
    sub = M.subscribe_waypoints(_show_table)
    if args.follow:
        rospy.spin()
    elif not done_evt.wait(30.0):
        fault("Something went wrong. Topic timed out.")


def do_load(args):
    wps = []
    wps_file = get_wp_file_io(args)
    with args.file:
        wps = [w for w in wps_file.read(args.file)]

    def _load_call(start, waypoint_list):
        try:
            ret = M.push(start_index=start, waypoints=waypoint_list)
        except rospy.ServiceException as ex:
            fault(ex)

        if not ret.success:
            fault("Request failed. Check mavros logs")

        print_if(args.verbose, "Waypoints transfered:", ret.wp_transfered)

    done_evt = threading.Event()
    def _fix_wp0(topic):
        if len(topic.waypoints) > 0:
            wps[0] = topic.waypoints[0]
            print_if(args.verbose, "HOME location: latitude:", wps[0].x_lat,
                     "longitude:", wps[0].y_long, "altitude:", wps[0].z_alt)
        else:
            print("Failed to get WP0! WP0 will be loaded from file.", file=sys.stderr)

        done_evt.set()

    if args.start_index > 0:
        # Push partial
        if args.start_index == args.end_index:
            args.end_index += 1

        end_index = args.end_index or len(wps)
        _load_call(args.start_index, wps[args.start_index:end_index])
    else:
        # Full push
        if not args.preserve_home:
            _load_call(0, wps)
        else:
            # Note: _load_call() emit publish on this topic, so callback only changes
            # waypoint 0, and signal done event.
            sub = M.subscribe_waypoints(_fix_wp0)
            if not done_evt.wait(30.0):
                fault("Something went wrong. Topic timed out.")
            else:
                sub.unregister()
                _load_call(0, wps)


def do_dump(args):
    done_evt = threading.Event()
    def _write_file(topic):
        wps_file = get_wp_file_io(args)
        with args.file:
            wps_file.write(args.file, topic.waypoints)
        done_evt.set()

    # Waypoints topic is latched type, and it updates after pull
    _pull(args)
    sub = M.subscribe_waypoints(_write_file)
    if not done_evt.wait(30.0):
        fault("Something went wrong. Topic timed out.")


def do_clear(args):
    try:
        ret = M.clear()
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed, Check mavros logs")
    else:
        print_if(args.verbose, "Waypoints cleared")


def do_set_current(args):
    try:
        ret = M.set_current(wp_seq=args.seq)
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed, Check mavros logs")
    else:
        print_if(args.verbose, "Set current done.")


def main():
    parser = argparse.ArgumentParser(description="Command line tool for manipulating missions on MAVLink device.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default=mavros.DEFAULT_NAMESPACE)
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    subarg = parser.add_subparsers()

    if not no_prettytable:
        show_args = subarg.add_parser('show', help="Show waypoints")
        show_args.add_argument('-f', '--follow', action='store_true', help="Watch and show new data")
        show_args.add_argument('-p', '--pull', action='store_true', help="Pull waypoints from FCU before show")
        show_args.set_defaults(func=do_show)

    load_args = subarg.add_parser('load', help="Load waypoints from file")
    load_args.set_defaults(func=do_load)
    load_args.add_argument('-p', '--preserve-home', action='store_true', help="Preserve home location (WP 0, APM only)")
    load_args.add_argument('-s', '--start-index', type=int, default=0, help="Waypoint start index for partial updating (APM only)")
    load_args.add_argument('-e', '--end-index', type=int, default=0, help="Waypoint end index for partial updating (APM only, default: last element in waypoint list)")
    load_args.add_argument('file', type=argparse.FileType('rb'), help="Input file (QGC/MP format)")

    pull_args = subarg.add_parser('pull', help="Pull waypoints from FCU")
    pull_args.set_defaults(func=do_pull)

    dump_args = subarg.add_parser('dump', help="Dump waypoints to file")
    dump_args.set_defaults(func=do_dump)
    dump_args.add_argument('file', type=argparse.FileType('wb'), help="Output file (QGC format)")

    clear_args = subarg.add_parser('clear', help="Clear waypoints on device")
    clear_args.set_defaults(func=do_clear)

    setcur_args = subarg.add_parser('setcur', help="Set current waypoints on device")
    setcur_args.add_argument('seq', type=int, help="Waypoint seq id")
    setcur_args.set_defaults(func=do_set_current)

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mavwp", anonymous=True)
    mavros.set_namespace(args.mavros_ns)

    args.func(args)


if __name__ == '__main__':
    main()
