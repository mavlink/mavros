# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rospy
import struct
from pymavlink import mavutil
from pymavlink.generator.mavcrc import x25crc
from mavros_msgs.msg import Mavlink
from std_msgs.msg import Header


def convert_to_bytes(msg):
    """
    Re-builds the MAVLink byte stream from mavros_msgs/Mavlink messages.
    """
    payload_octets = len(msg.payload64)
    msg_len = 6 + msg.len  # header + payload length
    if payload_octets < msg.len / 8:
        raise ValueError("Specified payload length is bigger than actual payload64")

    msgdata = bytearray(
        struct.pack(
            '<BBBBBB%dQ' % payload_octets,
            254, msg.len, msg.seq, msg.sysid, msg.compid, msg.msgid,
            *msg.payload64))

    if payload_octets != msg.len / 8:
        # message is shorter than payload octets
        msgdata = msgdata[:msg_len]

    if hasattr(mag, 'checksum'):
        # since #286 Mavlink.msg save original checksum, so recalculation not needed.
        crc16 = msg.checksum
    else:
        # from MAVLink.decode()
        message_type = mavutil.mavlink.mavlink_map[msg.msgid]
        crc_extra = message_type.crc_extra

        # calculate crc16
        crcbuf = msgdata[1:]
        crcbuf.append(crc_extra)
        crc16 = x25crc(crcbuf).crc

    # finalize
    msgdata += struct.pack('<H', crc16)
    return msgdata


def convert_to_payload64(payload_bytes):
    """
    Convert payload bytes to Mavlink.payload64
    """
    payload_bytes = bytearray(payload_bytes)
    payload_len = len(payload_bytes)
    payload_octets = payload_len / 8
    if payload_len % 8 > 0:
        payload_octets += 1
        payload_bytes += b'\0' * (8 - payload_len % 8)

    return struct.unpack('<%dQ' % payload_octets, payload_bytes)


def convert_to_rosmsg(mavmsg, stamp=None):
    """
    Convert pymavlink message to Mavlink.msg
    """
    if stamp is not None:
        header = Header(stamp=stamp)
    else:
        header = Header(stamp=rospy.get_rostime())

    return Mavlink(
        header=header,
        is_valid=True,
        len=len(mavmsg.get_payload()),
        seq=mavmsg.get_seq(),
        sysid=mavmsg.get_srcSystem(),
        compid=mavmsg.get_srcComponent(),
        msgid=mavmsg.get_msgId(),
        checksum=mavmsg.get_crc(),
        payload64=convert_to_payload64(mavmsg.get_payload())
    )
