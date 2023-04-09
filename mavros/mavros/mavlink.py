# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import struct
import typing

import rclpy.time
from pymavlink import mavutil
from pymavlink.generator.mavcrc import x25crc  # noqa F401
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from mavros_msgs.msg import Mavlink

from .utils import system_now

MAVLink_message = typing.TypeVar("MAVLink_message")


def convert_to_bytes(msg: Mavlink) -> bytearray:
    """
    Re-builds the MAVLink byte stream from mavros_msgs/Mavlink messages.

    Support both v1.0 and v2.0.
    """
    payload_octets = len(msg.payload64)
    if payload_octets < msg.len / 8:
        raise ValueError("Specified payload length is bigger than actual payload64")

    if msg.magic == Mavlink.MAVLINK_V10:
        msg_len = 6 + msg.len  # header + payload length
        msgdata = bytearray(
            struct.pack(
                "<BBBBBB%dQ" % payload_octets,
                msg.magic,
                msg.len,
                msg.seq,
                msg.sysid,
                msg.compid,
                msg.msgid,
                *msg.payload64,
            )
        )
    else:  # MAVLINK_V20
        msg_len = 10 + msg.len  # header + payload length
        msgdata = bytearray(
            struct.pack(
                "<BBBBBBBBBB%dQ" % payload_octets,
                msg.magic,
                msg.len,
                msg.incompat_flags,
                msg.compat_flags,
                msg.seq,
                msg.sysid,
                msg.compid,
                msg.msgid & 0xFF,
                (msg.msgid >> 8) & 0xFF,
                (msg.msgid >> 16) & 0xFF,
                *msg.payload64,
            )
        )

    if payload_octets != msg.len / 8:
        # message is shorter than payload octets
        msgdata = msgdata[:msg_len]

    # finalize
    msgdata += struct.pack("<H", msg.checksum)

    if msg.magic == Mavlink.MAVLINK_V20:
        msgdata += bytearray(msg.signature)

    return msgdata


def convert_to_payload64(
    payload_bytes: typing.Union[bytes, bytearray]
) -> typing.List[int]:
    """Convert payload bytes to Mavlink.payload64."""
    payload_bytes = bytearray(payload_bytes)
    payload_len = len(payload_bytes)
    payload_octets = int(payload_len / 8)
    if payload_len % 8 > 0:
        payload_octets += 1
        payload_bytes += b"\0" * (8 - payload_len % 8)

    return struct.unpack(f"<{payload_octets}Q", payload_bytes)


def convert_to_rosmsg(
    mavmsg: MAVLink_message, stamp: typing.Optional[rclpy.time.Time] = None
) -> Mavlink:
    """
    Convert pymavlink message to Mavlink.msg.

    Currently supports both MAVLink v1.0 and v2.0,
    but without signing.
    """
    if stamp is not None:
        header = Header(stamp=stamp)
    else:
        stamp = Time()
        stamp.sec, stamp.nanosec = system_now().seconds_nanoseconds()
        header = Header(stamp=stamp)

    if mavutil.mavlink20():
        # XXX Need some api to retreive signature block.
        if mavmsg.get_signed():
            raise ValueError("Signed message can't be converted to rosmsg.")

        hdr = mavmsg.get_header()
        return Mavlink(
            header=header,
            framing_status=Mavlink.FRAMING_OK,
            magic=Mavlink.MAVLINK_V20,
            len=hdr.mlen,
            incompat_flags=hdr.incompat_flags,
            compat_flags=hdr.compat_flags,
            sysid=hdr.srcSystem,
            compid=hdr.srcComponent,
            msgid=hdr.msgId,
            checksum=mavmsg.get_crc(),
            payload64=convert_to_payload64(mavmsg.get_payload()),
            signature=None,  # FIXME #569
        )

    else:
        return Mavlink(
            header=header,
            framing_status=Mavlink.FRAMING_OK,
            magic=Mavlink.MAVLINK_V10,
            len=len(mavmsg.get_payload()),
            seq=mavmsg.get_seq(),
            sysid=mavmsg.get_srcSystem(),
            compid=mavmsg.get_srcComponent(),
            msgid=mavmsg.get_msgId(),
            checksum=mavmsg.get_crc(),
            payload64=convert_to_payload64(mavmsg.get_payload()),
        )
