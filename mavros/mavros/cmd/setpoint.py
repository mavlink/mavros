#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2016 Nuno Marques, Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import argparse
import math

import rospy
import mavros

from mavros import command, setpoint as sp
from mavros.utils import *
from mavros_msgs.srv import SetMode
from tf.transformations import quaternion_from_euler

_ONCE_DELAY = 3


def publish_once(args, pub, msg):
    pub.publish(msg)
    rospy.sleep(0.2)
    enable_offboard()

    # stick around long enough for others to grab
    timeout_t = rospy.get_time() + _ONCE_DELAY
    while not rospy.is_shutdown() and rospy.get_time() < timeout_t:
        rospy.sleep(0.2)
        print_if(pub.get_num_connections() < 1,
                 "Mavros not started, nobody subcsribes to", pub.name)


def enable_offboard():
    print("Requesting OFFBOARD mode...")

    try:
        set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
        ret = set_mode(custom_mode="OFFBOARD")
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.mode_sent:
        print("Request failed. Check mavros logs")
    else:
        print("OFFBOARD mode are set.")


def do_local_pos(args):
    pub = sp.get_pub_position_local(queue_size=10, latch=True)

    pos = sp.PoseStamped(header=sp.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
    pos.pose.position = sp.Point(x=args.position[0], y=args.position[1], z=args.position[2])

    yaw = math.radians(args.position[3]) if args.degrees else args.position[3]
    q = quaternion_from_euler(0, 0, yaw)
    pos.pose.orientation = sp.Quaternion(*q)

    publish_once(args, pub, pos)


def do_local_vel(args):
    pub = sp.get_pub_velocity_cmd_vel(queue_size=10, latch=True)

    vel = sp.TwistStamped(header=sp.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
    vel.twist.linear = sp.Vector3(x=args.velocity[0], y=args.velocity[1], z=args.velocity[2])
    vel.twist.angular = sp.Vector3(z=args.velocity[3])

    publish_once(args, pub, vel)


def do_local_accel(args):
    pub = sp.get_pub_accel_accel(queue_size=10, latch=True)

    vel = sp.Vector3Stamped(header=sp.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
    vel.vector = sp.Vector3(x=args.acceleration[0], y=args.acceleration[1], z=args.acceleration[2])

    publish_once(args, pub, vel)


def do_local_selector(args):
    if args.position is not None:
        do_local_pos(args)
    elif args.velocity is not None:
        do_local_vel(args)
    else:
        do_local_accel(args)


def main():
    parser = argparse.ArgumentParser(description="Command line tool for controlling the device by setpoints.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default=mavros.DEFAULT_NAMESPACE)
    parser.add_argument('-V', '--verbose', action='store_true', help="Verbose output")
    subarg = parser.add_subparsers()

    local_args = subarg.add_parser('local', help="Send local setpoint")
    local_args.add_argument('-d', '--degrees', action='store_true', help="Yaw angle in degrees")
    local_args.set_defaults(func=do_local_selector)
    local_group = local_args.add_mutually_exclusive_group(required=True)

    local_group.add_argument('-p', '--position', type=float, nargs=4,
                             metavar=('x', 'y', 'z', 'y_ang'), help="Local position & desired yaw angle")

    local_group.add_argument('-v', '--velocity', type=float, nargs=4,
                             metavar=('vx', 'vy', 'vz', 'y_rate'), help="Linear velocity & desired yaw rate")

    local_group.add_argument('-a', '--acceleration', type=float, nargs=3,
                             metavar=('afx', 'afy', 'afz'), help="Linear acceleration/force")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mavsetp", anonymous=True)
    mavros.set_namespace(args.mavros_ns)
    args.func(args)


if __name__ == '__main__':
    main()
