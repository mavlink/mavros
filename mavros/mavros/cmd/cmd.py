#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""
mav cmd command
"""

import sys
import threading

import click
from sensor_msgs.msg import NavSatFix

from mavros_msgs.srv import (CommandBool, CommandHome, CommandInt, CommandLong,
                             CommandTOL, CommandTriggerControl,
                             CommandTriggerInterval)

from . import cli, pass_client


def _check_ret(ctx, client, ret):
    if not ret.success:
        click.echo(f"Request failed. Check mavros logs. ACK: {ret.result}")
        ctx.exit(1)

    if client.verbose:
        if hasattr(ret, 'result'):
            click.echo(f"Command ACK: {ret.result}")
        else:
            click.echo("Request done.")


def bool2int(b: bool) -> int:
    """converts bool to 1 or 0

    I had an exception "TypeError: 'bool' object is not iterable"
    for code like int(confirmation).

    Why? Who knows..."""
    if b:
        return 1
    return 0


@cli.group()
@click.option("--wait",
              type=float,
              is_flag=False,
              flag_value=None,
              envvar="MAVCLI_WAIT",
              default=False,
              help="Wait for establishing FCU connection")
@pass_client
def cmd(client, wait):
    """Tool to send commands to MAVLink device."""

    if wait is not False:
        if client.verbose:
            click.echo("Waiting connection to the FCU...")

        client.system.wait_fcu_connection(wait)


@cmd.command()
@click.option("-c",
              "--confirmation",
              default=False,
              is_flag=True,
              help="Require confirmation")
@click.option("-b",
              "--broadcast",
              default=False,
              is_flag=True,
              help="Broadcast command")
@click.argument('command', type=int)
@click.argument('param1', type=float)
@click.argument('param2', type=float)
@click.argument('param3', type=float)
@click.argument('param4', type=float)
@click.argument('param5', type=float)
@click.argument('param6', type=float)
@click.argument('param7', type=float)
@pass_client
@click.pass_context
def long(ctx, client, confirmation, broadcast, command, param1, param2, param3,
         param4, param5, param6, param7):
    "Send any command (COMMAND_LONG)"

    req = CommandLong.Request(
        broadcast=broadcast,
        command=command,
        confirmation=bool2int(confirmation),
        param1=param1,
        param2=param2,
        param3=param3,
        param4=param4,
        param5=param5,
        param6=param6,
        param7=param7,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_long.call(req)
    _check_ret(ctx, client, ret)


@cmd.command()
@click.option("-c",
              "--current",
              default=False,
              is_flag=True,
              help="Is current?")
@click.option('-a',
              '--autocontinue',
              default=False,
              is_flag=True,
              help="Is autocontinue?")
@click.option('-f', '--frame', type=int, default=3, help="Frame Code")
@click.option("-b",
              "--broadcast",
              default=False,
              is_flag=True,
              help="Broadcast command")
@click.argument('command', type=int)
@click.argument('param1', type=float)
@click.argument('param2', type=float)
@click.argument('param3', type=float)
@click.argument('param4', type=float)
@click.argument('x', type=int)
@click.argument('y', type=int)
@click.argument('z', type=float)
@pass_client
@click.pass_context
def int(ctx, client, current, autocontinue, broadcast, frame, command, param1,
        param2, param3, param4, x, y, z):
    """Send any command (COMMAND_INT)"""

    req = CommandInt.Request(
        broadcast=broadcast,
        command=command,
        frame=frame,
        current=bool2int(current),
        autocontinue=bool2int(autocontinue),
        param1=param1,
        param2=param2,
        param3=param3,
        param4=param4,
        x=x,
        y=y,
        z=z,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_int.call(req)
    _check_ret(ctx, client, ret)


@cmd.command()
@click.option("-c",
              "--current-gps",
              is_flag=True,
              help="Use current GPS location (use 0 0 0 for location args)")
@click.argument('latitude', type=float, nargs=-1)
@click.argument('longitude', type=float, nargs=-1)
@click.argument('altitude', type=float, nargs=-1)
@pass_client
@click.pass_context
def set_home(ctx, client, current_gps, latitude, longitude, altitude):
    """Request change home position"""

    req = CommandHome.Request(
        current_gps=current_gps,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_set_home.call(req)
    _check_ret(ctx, client, ret)


@cmd.command()
@click.option("--min-pitch", type=float, required=True, help="Min pitch")
@click.option("--yaw", type=float, required=True, help="Desired Yaw")
@click.argument('latitude', type=float, nargs=-1)
@click.argument('longitude', type=float, nargs=-1)
@click.argument('altitude', type=float, nargs=-1)
@pass_client
@click.pass_context
def takeoff(ctx, client, min_pitch, yaw, current_gps, latitude, longitude,
            altitude):
    """Request takeoff"""

    req = CommandTOL.Request(
        min_pitch=min_pitch,
        yaw=yaw,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_takeoff.call(req)
    _check_ret(ctx, client, ret)


@cmd.command()
@click.option("--yaw", type=float, required=True, help="Desired Yaw")
@click.argument('latitude', type=float, nargs=-1)
@click.argument('longitude', type=float, nargs=-1)
@click.argument('altitude', type=float, nargs=-1)
@pass_client
@click.pass_context
def land(ctx, client, yaw, current_gps, latitude, longitude, altitude):
    """Request land"""

    req = CommandTOL.Request(
        min_pitch=0.0,
        yaw=yaw,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_land.call(req)
    _check_ret(ctx, client, ret)


def do_takeoff_cur_gps(args):
    done_evt = threading.Event()

    def fix_cb(fix):
        print("Taking-off from current coord: Lat:", fix.latitude, "Long:",
              fix.longitude)
        print_if(args.verbose, "With desired Altitude:", args.altitude, "Yaw:",
                 args.yaw, "Pitch angle:", args.min_pitch)

        try:
            ret = command.takeoff(min_pitch=args.min_pitch,
                                  yaw=args.yaw,
                                  latitude=fix.latitude,
                                  longitude=fix.longitude,
                                  altitude=args.altitude)
        except rospy.ServiceException as ex:
            fault(ex)

        _check_ret(args, ret)
        done_evt.set()

    topic = _find_gps_topic(args, "takeoff")
    if topic is None:
        fault("NavSatFix topic not exist")

    sub = rospy.Subscriber(topic, NavSatFix, fix_cb)
    if not done_evt.wait(10.0):
        fault("Something went wrong. Topic timed out.")


def do_land_cur_gps(args):
    done_evt = threading.Event()

    def fix_cb(fix):
        print("Landing on current coord: Lat:", fix.latitude, "Long:",
              fix.longitude)
        print_if(args.verbose, "With desired Altitude:", args.altitude, "Yaw:",
                 args.yaw)

        try:
            ret = command.land(min_pitch=0.0,
                               yaw=args.yaw,
                               latitude=fix.latitude,
                               longitude=fix.longitude,
                               altitude=args.altitude)
        except rospy.ServiceException as ex:
            fault(ex)

        _check_ret(args, ret)
        done_evt.set()

    topic = _find_gps_topic(args, "landing")
    if topic is None:
        fault("NavSatFix topic not exist")

    sub = rospy.Subscriber(topic, NavSatFix, fix_cb)
    if not done_evt.wait(10.0):
        fault("Something went wrong. Topic timed out.")


@cmd.command()
@click.option("-e",
              "--enable",
              "trigger_enable",
              flag_value=True,
              default=True,
              help="Enable camera trigger")
@click.option("-d",
              "--disable",
              "trigger_enable",
              flag_value=False,
              help="Disable camera trigger")
@click.option(
    '-c',
    '--cycle-time',
    default=0.0,
    type=float,
    help="Camera trigger cycle time. Zero to use current onboard value")
@pass_client
@click.pass_context
def trigger_control(ctx, client, trigger_enable, cycle_time):
    "Control onboard camera triggering system (PX4)"

    req = CommandTriggerControl.Request(
        trigger_enable=trigger_enable,
        cycle_time=cycle_time,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_trigger_control.call(req)
    _check_ret(ctx, client, ret)


def old_and_unused_main():

    # Note: arming service provided by mavsafety

    takeoff_cur_args = subarg.add_parser(
        'takeoffcur', help="Request takeoff from current GPS coordinates")
    takeoff_cur_args.set_defaults(func=do_takeoff_cur_gps)
    takeoff_cur_args.add_argument(
        '-a',
        '--any-gps',
        action="store_true",
        help="Try to find GPS topic (warn: could be dangerous!)")
    takeoff_cur_args.add_argument('min_pitch', type=float, help="Min pitch")
    takeoff_cur_args.add_argument('yaw', type=float, help="Desired Yaw")
    takeoff_cur_args.add_argument('altitude', type=float, help="Altitude")

    land_cur_args = subarg.add_parser(
        'landcur', help="Request land on current GPS coordinates")
    land_cur_args.set_defaults(func=do_land_cur_gps)
    land_cur_args.add_argument(
        '-a',
        '--any-gps',
        action="store_true",
        help="Try to find GPS topic (warn: could be dangerous!)")
    land_cur_args.add_argument('yaw', type=float, help="Desired Yaw")
    land_cur_args.add_argument('altitude', type=float, help="Altitude")
