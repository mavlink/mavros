# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import struct
from pymavlink import mavutil
from pymavlink.generator.mavcrc import x25crc
from mavros_msgs.msg import Mavlink


def convert_to_bytes(msg):
    """
    This function build wire byte stream from mavros_msgs/Mavlink
    """
    pay_len = len(msg.payload64)
    last_len = 0
    if pay_len < msg.len / 8:
        raise ValueError("Message length are bigger than payload64")

    if pay_len != msg.len / 8:
        # message are shorter than payload quads
        pay_len = msg.len / 8
        last_len = msg.len % 8

    msgdata = bytearray(
        struct.pack(
            '<BBBBBB%dQ' % pay_len,
            254, msg.len, msg.seq, msg.sysid, msg.compid, msg.msgid,
            *(msg.payload64[:pay_len])))
    if last_len:
        q = struct.unpack('8B', struct.pack('<Q', msg.payload64[pay_len]))
        msgdata += bytearray(q[:last_len])

    # from MAVLink.decode()
    type = mavutil.mavlink.mavlink_map[msg.msgid]
    crc_extra = type.crc_extra

    # calculate crc16
    crcbuf = msgdata[1:]
    crcbuf.append(crc_extra)
    crc16 = x25crc(crcbuf).crc

    # finalize
    msgdata += struct.pack('<H', crc16)
    return msgdata
