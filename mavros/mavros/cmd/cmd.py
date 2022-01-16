# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""
mav cmd command.

Note: arming service provided by mav safety.
"""

import threading

import click
from sensor_msgs.msg import NavSatFix

from mavros_msgs.srv import (
    CommandHome,
    CommandInt,
    CommandLong,
    CommandTOL,
    CommandTriggerControl,
    CommandTriggerInterval,
)

from . import cli, pass_client
from .utils import bool2int, check_cmd_ret


@cli.group()
@pass_client
def cmd(client):
    """Tool to send commands to MAVLink device."""


@cmd.command()
@click.option(
    "-c", "--confirmation", default=False, is_flag=True, help="Require confirmation"
)
@click.option(
    "-b", "--broadcast", default=False, is_flag=True, help="Broadcast command"
)
@click.argument("command", type=int)
@click.argument("param1", type=float)
@click.argument("param2", type=float)
@click.argument("param3", type=float)
@click.argument("param4", type=float)
@click.argument("param5", type=float)
@click.argument("param6", type=float)
@click.argument("param7", type=float)
@pass_client
@click.pass_context
def long(
    ctx,
    client,
    confirmation,
    broadcast,
    command,
    param1,
    param2,
    param3,
    param4,
    param5,
    param6,
    param7,
):
    """Send any command (COMMAND_LONG)."""
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
    check_cmd_ret(ctx, client, ret)


@cmd.command()
@click.option("-c", "--current", default=False, is_flag=True, help="Is current?")
@click.option(
    "-a", "--autocontinue", default=False, is_flag=True, help="Is autocontinue?"
)
@click.option("-f", "--frame", type=int, default=3, help="Frame Code")
@click.option(
    "-b", "--broadcast", default=False, is_flag=True, help="Broadcast command"
)
@click.argument("command", type=int)
@click.argument("param1", type=float)
@click.argument("param2", type=float)
@click.argument("param3", type=float)
@click.argument("param4", type=float)
@click.argument("x", type=int)
@click.argument("y", type=int)
@click.argument("z", type=float)
@pass_client
@click.pass_context
def int(
    ctx,
    client,
    current,
    autocontinue,
    broadcast,
    frame,
    command,
    param1,
    param2,
    param3,
    param4,
    x,
    y,
    z,
):
    """Send any command (COMMAND_INT)."""
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
    check_cmd_ret(ctx, client, ret)


@cmd.command()
@click.option(
    "-c",
    "--current-gps",
    is_flag=True,
    help="Use current GPS location (use 0 0 0 for location args)",
)
@click.argument("latitude", type=float)
@click.argument("longitude", type=float)
@click.argument("altitude", type=float)
@pass_client
@click.pass_context
def set_home(ctx, client, current_gps, latitude, longitude, altitude):
    """Request change home position."""
    req = CommandHome.Request(
        current_gps=current_gps,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_set_home.call(req)
    check_cmd_ret(ctx, client, ret)


@cmd.command()
@click.option("--min-pitch", type=float, required=True, help="Min pitch")
@click.option("--yaw", type=float, required=True, help="Desired Yaw")
@click.argument("latitude", type=float)
@click.argument("longitude", type=float)
@click.argument("altitude", type=float)
@pass_client
@click.pass_context
def takeoff(ctx, client, min_pitch, yaw, latitude, longitude, altitude):
    """Request takeoff."""
    req = CommandTOL.Request(
        min_pitch=min_pitch,
        yaw=yaw,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_takeoff.call(req)
    check_cmd_ret(ctx, client, ret)


@cmd.command()
@click.option("--yaw", type=float, required=True, help="Desired Yaw")
@click.argument("latitude", type=float)
@click.argument("longitude", type=float)
@click.argument("altitude", type=float)
@pass_client
@click.pass_context
def land(ctx, client, yaw, latitude, longitude, altitude):
    """Request land."""
    req = CommandTOL.Request(
        min_pitch=0.0,
        yaw=yaw,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_land.call(req)
    check_cmd_ret(ctx, client, ret)


@cmd.command()
@click.option("--min-pitch", type=float, required=True, help="Min pitch")
@click.option("--yaw", type=float, required=True, help="Desired Yaw")
@click.argument("altitude", type=float)
@pass_client
@click.pass_context
def takeoff_cur(ctx, client, min_pitch, yaw, altitude):
    """Request takeoff from current GPS coordinates."""
    done_evt = threading.Event()

    def fix_cb(fix: NavSatFix):
        click.echo(
            f"Taking-off from current coord: "
            f"Lat: {fix.latitude}, Long: {fix.longitude}"
        )
        client.verbose_echo(
            f"With desired Altitude: {altitude}, "
            f"Yaw: {yaw}, Pitch angle: {min_pitch}"
        )

        req = CommandTOL.Request(
            min_pitch=min_pitch,
            yaw=yaw,
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=altitude,
        )

        client.verbose_echo(f"Calling: {req}")
        ret = client.command.cli_takeoff.call(req)
        check_cmd_ret(ctx, client, ret)

        done_evt.set()

    client.global_position.subscribe_fix(fix_cb)
    if not done_evt.wait(10.0):
        click.echo("Something went wrong. Topic timed out.")
        ctx.exit(1)


@cmd.command()
@click.option("--yaw", type=float, required=True, help="Desired Yaw")
@click.argument("altitude", type=float)
@pass_client
@click.pass_context
def land_cur(ctx, client, yaw, altitude):
    """Request land on current GPS coordinates."""
    done_evt = threading.Event()

    def fix_cb(fix: NavSatFix):
        click.echo(
            f"Landing on current coord: " f"Lat: {fix.latitude}, Long: {fix.longitude}"
        )
        client.verbose_echo(f"With desired Altitude: {altitude}, Yaw: {yaw}")

        req = CommandTOL.Request(
            min_pitch=0.0,
            yaw=yaw,
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=altitude,
        )

        client.verbose_echo(f"Calling: {req}")
        ret = client.command.cli_land.call(req)
        check_cmd_ret(ctx, client, ret)

        done_evt.set()

    client.global_position.subscribe_fix(fix_cb)
    if not done_evt.wait(10.0):
        click.echo("Something went wrong. Topic timed out.")
        ctx.exit(1)


@cmd.command()
@click.option(
    "-e",
    "--enable",
    "trigger_enable",
    flag_value=True,
    default=True,
    help="Enable camera trigger",
)
@click.option(
    "-d", "--disable", "trigger_enable", flag_value=False, help="Disable camera trigger"
)
@click.option(
    "-r", "--sequence-reset", is_flag=True, default=False, help="Reset trigger sequence"
)
@click.option(
    "-p", "--trigger-pause", is_flag=True, default=False, help="Pause triggering"
)
@pass_client
@click.pass_context
def trigger_control(ctx, client, trigger_enable, sequence_reset, trigger_pause):
    """Control onboard camera triggering system (PX4)."""
    req = CommandTriggerControl.Request(
        trigger_enable=trigger_enable,
        sequence_reset=sequence_reset,
        trigger_pause=trigger_pause,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_trigger_control.call(req)
    check_cmd_ret(ctx, client, ret)


@cmd.command()
@click.option(
    "-c",
    "--cycle-time",
    default=0.0,
    type=float,
    help="Camera trigger cycle time. Zero to use current onboard value",
)
@click.option(
    "-i",
    "--integration-time",
    default=0.0,
    type=float,
    help="Camera shutter intergration time. Zero to ignore",
)
@pass_client
@click.pass_context
def trigger_interval(ctx, client, cycle_time, integration_time):
    """Control onboard camera triggering system (PX4)."""
    req = CommandTriggerInterval.Request(
        cycle_time=cycle_time,
        integration_time=integration_time,
    )

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_trigger_interval.call(req)
    check_cmd_ret(ctx, client, ret)
