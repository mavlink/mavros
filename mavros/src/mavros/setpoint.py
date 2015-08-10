# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rospy
import mavros

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, \
        Vector3, Vector3Stamped, Point, Quaternion


def get_pub_accel_accel(**kvargs):
    """
    Returns publisher for :setpoint_accel: plugin, :accel: topic
    """
    return rospy.Publisher(mavros.get_topic('setpoint_accel', 'accel'), Vector3Stamped, **kvargs)


def get_pub_attitude_cmd_vel(**kvargs):
    """
    Returns publisher for :setpoint_attitude: plugin, :cmd_vel: topic
    """
    return rospy.Publisher(mavros.get_topic('setpoint_attitude', 'cmd_vel'), PoseStamped, **kvargs)


def get_pub_attitude_throttle(**kvargs):
    """
    Returns publisher for :setpoint_attitude: plugin, :cmd_vel: topic
    """
    return rospy.Publisher(mavros.get_topic('setpoint_attitude', 'att_throttle'), Float64, **kvargs)


def get_pub_attitude_pose(**kvargs):
    """
    Returns publisher for :setpoint_attitude: plugin, :attituse: topic
    """
    return rospy.Publisher(mavros.get_topic('setpoint_attitude', 'attitude'), PoseStamped, **kvargs)


def get_pub_attitude_posecov(**kvargs):
    """
    Returns publisher for :setpoint_attitude: plugin, :attituse: topic (with covariance)
    """
    raise DeprecationWarning("PoseWithCovarianceStamped subscriber removed.")


def get_pub_position_local(**kvargs):
    """
    Returns publisher for :setpoint_position: plugin, :local: topic
    """
    return rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, **kvargs)


def get_pub_velocity_cmd_vel(**kvargs):
    """
    Returns publisher for :setpoint_velocity: plugin, :cmd_vel: topic
    """
    return rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, **kvargs)


