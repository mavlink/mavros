#!/usr/bin/env python3
# libmavconn
# Copyright 2026, mavros contributors. Licensed as the rest of the package.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#
"""
End-to-end byte-stream tests for libmavconn, using pymavlink as reference.

Drive a real MAVConnUDP link through a small ctypes shim and check that the
bytes libmavconn emits match pymavlink's reference encoding, and that frames
crafted by pymavlink are decoded by libmavconn into the same message fields.

Skip cleanly when pymavlink (or the shim) is unavailable, so CI/buildfarm
without pip stays green.
"""

import ctypes
import importlib.util
import os
import socket
import struct
import time

import pytest

# Skip the whole module at collection time when pymavlink is unavailable, but
# still *collect* the tests so pytest exits 0. pytest.importorskip() at import
# time collects 0 tests, which makes pytest exit with code 5 ("no tests
# collected") and ament's run_test.py then reports the test as failed.
_HAVE_PYMAVLINK = importlib.util.find_spec("pymavlink") is not None
pytestmark = pytest.mark.skipif(
    not _HAVE_PYMAVLINK, reason="pymavlink is not installed (pip-only test dep)"
)

if _HAVE_PYMAVLINK:
    from pymavlink.dialects.v20 import common as mav_common


# --------------------------------------------------------------------------
# ctypes shim binding
# --------------------------------------------------------------------------

_SHIM_PATH = os.environ.get("MAVCONN_E2E_SHIM", "")


class RxMsg(ctypes.Structure):
    _fields_ = [
        ("msgid", ctypes.c_uint32),
        ("sysid", ctypes.c_uint8),
        ("compid", ctypes.c_uint8),
        ("seq", ctypes.c_uint8),
        ("len", ctypes.c_uint8),
        ("framing", ctypes.c_uint8),
        ("payload", ctypes.c_uint8 * 255),
    ]

    def payload_bytes(self):
        return bytes(self.payload[: self.len])


def _load_shim():
    if not _SHIM_PATH or not os.path.exists(_SHIM_PATH):
        pytest.skip(f"mavconn e2e shim not available (MAVCONN_E2E_SHIM={_SHIM_PATH!r})")
    lib = ctypes.CDLL(_SHIM_PATH)
    lib.mavconn_udp_create.restype = ctypes.c_void_p
    lib.mavconn_udp_create.argtypes = [
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint16,
        ctypes.c_char_p,
        ctypes.c_uint16,
    ]
    lib.mavconn_poll_rx.restype = ctypes.c_int
    lib.mavconn_poll_rx.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(RxMsg),
        ctypes.c_int,
    ]
    lib.mavconn_send_heartbeat.restype = ctypes.c_int
    lib.mavconn_send_heartbeat.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint32,
        ctypes.c_uint8,
    ]
    lib.mavconn_send_raw_bytes.restype = ctypes.c_int
    lib.mavconn_send_raw_bytes.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_uint32,
    ]
    lib.mavconn_setup_signing.restype = ctypes.c_int
    lib.mavconn_setup_signing.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint64,
    ]
    lib.mavconn_destroy.restype = None
    lib.mavconn_destroy.argtypes = [ctypes.c_void_p]
    return lib


# MAVLink framing values (mavlink_framing_t)
FRAMING_INCOMPLETE = 0
FRAMING_OK = 1
FRAMING_BAD_CRC = 2
FRAMING_BAD_SIGNATURE = 3

# A deterministic test signing key.
TEST_KEY = bytes(range(32))


def _mav(src_system=42, src_component=200, signing=None):
    """Create a pymavlink MAVLink codec context (no transport)."""
    m = mav_common.MAVLink(None, srcSystem=src_system, srcComponent=src_component)
    if signing is not None:
        m.signing.secret_key = signing["key"]
        m.signing.sign_outgoing = True
        m.signing.link_id = signing.get("link_id", 0)
        m.signing.timestamp = signing.get("timestamp", 0)
    return m


def _pack_heartbeat(mav, **fields):
    # NOTE: pass fields by keyword. pymavlink's positional ctor order is the
    # message *declaration* order, which is NOT the wire (sorted) order.
    msg = mav_common.MAVLink_heartbeat_message(
        custom_mode=fields.get("custom_mode", 0),
        type=fields.get("type", 18),  # MAV_TYPE_ONBOARD_CONTROLLER
        autopilot=fields.get("autopilot", 8),  # MAV_AUTOPILOT_INVALID
        base_mode=fields.get("base_mode", 192),  # MAV_MODE_MANUAL_ARMED
        system_status=fields.get("system_status", 4),  # MAV_STATE_ACTIVE
        mavlink_version=fields.get("mavlink_version", 3),
    )
    msg.pack(mav)
    return msg


def _recv_parsed(sock, mav, timeout=3.0):
    """Receive one UDP datagram and parse it with pymavlink."""
    sock.settimeout(timeout)
    data, _ = sock.recvfrom(4096)
    msgs = mav.parse_buffer(data)
    return data, msgs


@pytest.fixture()
def shim():
    lib = _load_shim()
    yield lib


@pytest.fixture()
def link(shim):
    """Yield a MAVConnUDP link bound to an ephemeral port; torn down after test."""
    bind_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bind_sock.bind(("127.0.0.1", 0))
    bind_port = bind_sock.getsockname()[1]
    bind_sock.close()

    remote_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote_sock.bind(("127.0.0.1", 0))
    remote_port = remote_sock.getsockname()[1]

    handle = shim.mavconn_udp_create(42, 200, bind_port, b"127.0.0.1", remote_port)
    try:
        # Let the link's io thread post its async receive before any test sends
        # a datagram. UDP delivers unreceived datagrams to no one, so without
        # this the first packet can be lost on a loaded/slow CI runner.
        time.sleep(0.2)
        yield shim, handle, bind_port, remote_sock, remote_port
    finally:
        shim.mavconn_destroy(handle)
        remote_sock.close()


# --------------------------------------------------------------------------
# TX: libmavconn -> pymavlink
# --------------------------------------------------------------------------


def test_tx_heartbeat_matches_pymavlink(link):
    shim, handle, bind_port, remote_sock, remote_port = link

    # libmavconn sends a heartbeat.
    rc = shim.mavconn_send_heartbeat(handle, 18, 8, 192, 0, 4)
    assert rc == 0

    # pymavlink reference frame with the same ids/fields. seq starts at 0.
    ref = _mav(src_system=42, src_component=200)
    ref_msg = _pack_heartbeat(
        ref,
        type=18,
        autopilot=8,
        base_mode=192,
        custom_mode=0,
        system_status=4,
        mavlink_version=3,
    )
    expected = bytes(ref_msg.get_msgbuf())

    data, msgs = _recv_parsed(remote_sock, _mav())
    assert data == expected, f"wire mismatch:\n got {data.hex()}\nwant {expected.hex()}"

    assert msgs and msgs[-1].get_msgId() == 0  # HEARTBEAT
    hb = msgs[-1]
    assert (hb.type, hb.autopilot, hb.base_mode, hb.system_status) == (18, 8, 192, 4)
    assert (hb.get_srcSystem(), hb.get_srcComponent()) == (42, 200)


# --------------------------------------------------------------------------
# RX: pymavlink -> libmavconn
# --------------------------------------------------------------------------


def test_rx_heartbeat_from_pymavlink(link):
    shim, handle, bind_port, remote_sock, remote_port = link

    # pymavlink crafts and sends a heartbeat to the libmavconn bind port.
    sender = _mav(src_system=7, src_component=8)
    msg = _pack_heartbeat(
        sender, type=2, autopilot=3, base_mode=64, custom_mode=12345, system_status=4
    )
    wire = bytes(msg.get_msgbuf())

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.sendto(wire, ("127.0.0.1", bind_port))
    out.close()

    got = RxMsg()
    rc = shim.mavconn_poll_rx(handle, ctypes.byref(got), 3000)
    assert rc == 1, "libmavconn did not receive the pymavlink frame"
    assert got.framing == FRAMING_OK
    assert got.msgid == 0  # HEARTBEAT
    assert (got.sysid, got.compid, got.seq) == (7, 8, 0)
    # payload layout: custom_mode(u32) type(u8) autopilot(u8) base_mode(u8)
    # system_status(u8) mavlink_version(u8)
    assert got.payload_bytes() == struct.pack("<IBBBBB", 12345, 2, 3, 64, 4, 3)


def test_rx_sys_status_from_pymavlink(link):
    """A non-trivial message with extension fields round-trips."""
    shim, handle, bind_port, remote_sock, remote_port = link

    sender = _mav(src_system=5, src_component=6)
    ss = mav_common.MAVLink_sys_status_message(
        onboard_control_sensors_present=0xA5A5A5A5,
        onboard_control_sensors_enabled=0x0F0F0F0F,
        onboard_control_sensors_health=0x12345678,
        load=500,
        voltage_battery=16800,
        current_battery=-1,
        battery_remaining=50,
        drop_rate_comm=0,
        errors_comm=0,
        errors_count1=10,
        errors_count2=11,
        errors_count3=12,
        errors_count4=13,
    )
    ss.pack(sender)
    wire = bytes(ss.get_msgbuf())

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.sendto(wire, ("127.0.0.1", bind_port))
    out.close()

    got = RxMsg()
    rc = shim.mavconn_poll_rx(handle, ctypes.byref(got), 3000)
    assert rc == 1
    assert got.framing == FRAMING_OK
    assert got.msgid == 1  # SYS_STATUS
    assert (got.sysid, got.compid) == (5, 6)
    # The payload must equal pymavlink's own payload bytes exactly.
    assert got.payload_bytes() == bytes(ss.get_payload())


# --------------------------------------------------------------------------
# Signing
# --------------------------------------------------------------------------


def test_tx_signed_heartbeat_verified_by_pymavlink(link):
    shim, handle, bind_port, remote_sock, remote_port = link

    key = (ctypes.c_uint8 * 32)(*TEST_KEY)
    rc = shim.mavconn_setup_signing(handle, key, 1, 7, 1000)  # link_id=7, ts=1000
    assert rc == 0

    rc = shim.mavconn_send_heartbeat(handle, 18, 8, 192, 0, 4)
    assert rc == 0

    # pymavlink verifier with the same key; its own clock only needs the
    # timestamp window to accept a new stream.
    verifier = _mav()
    verifier.signing.secret_key = TEST_KEY
    verifier.signing.sign_outgoing = False
    verifier.signing.timestamp = 1000

    data, msgs = _recv_parsed(remote_sock, verifier)
    # signed v2 frame: incompat_flags has the SIGNED bit set.
    assert data[0] == 0xFD
    assert data[2] & 0x01, "frame not marked signed"
    assert msgs and msgs[-1].get_msgId() == 0, f"signature rejected: {msgs}"
    assert verifier.signing.badsig_count == 0


def test_rx_signed_heartbeat_from_pymavlink(link):
    shim, handle, bind_port, remote_sock, remote_port = link

    key = (ctypes.c_uint8 * 32)(*TEST_KEY)
    rc = shim.mavconn_setup_signing(handle, key, 1, 3, 1000)
    assert rc == 0

    sender = _mav(
        src_system=9,
        src_component=10,
        signing={"key": TEST_KEY, "link_id": 3, "timestamp": 1000},
    )
    msg = _pack_heartbeat(
        sender, type=18, autopilot=8, base_mode=192, custom_mode=0, system_status=4
    )
    wire = bytes(msg.get_msgbuf())

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.sendto(wire, ("127.0.0.1", bind_port))
    out.close()

    got = RxMsg()
    rc = shim.mavconn_poll_rx(handle, ctypes.byref(got), 3000)
    assert rc == 1
    assert got.framing == FRAMING_OK
    assert got.msgid == 0
    assert (got.sysid, got.compid) == (9, 10)


def test_rx_bad_signature_rejected(link):
    shim, handle, bind_port, remote_sock, remote_port = link

    key = (ctypes.c_uint8 * 32)(*TEST_KEY)
    rc = shim.mavconn_setup_signing(handle, key, 1, 3, 1000)
    assert rc == 0

    # Sender signs with a *different* key.
    bad_key = bytes(b ^ 0xFF for b in TEST_KEY)
    sender = _mav(
        src_system=9,
        src_component=10,
        signing={"key": bad_key, "link_id": 3, "timestamp": 1000},
    )
    msg = _pack_heartbeat(
        sender, type=18, autopilot=8, base_mode=192, custom_mode=0, system_status=4
    )
    wire = bytes(msg.get_msgbuf())

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.sendto(wire, ("127.0.0.1", bind_port))
    out.close()

    got = RxMsg()
    rc = shim.mavconn_poll_rx(handle, ctypes.byref(got), 3000)
    assert rc == 1
    assert got.framing == FRAMING_BAD_SIGNATURE
