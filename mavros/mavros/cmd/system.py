# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""mav sys command."""

import threading
import typing

import click

from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval, SetMode, StreamRate

from . import cli, pass_client
from .utils import apply_options, fault_echo


@cli.group()
@pass_client
def sys(client):
    """Tool to change mode and rate on MAVLink device."""


@sys.command()
@click.option("-b", "--base-mode", type=int, default=0, help="Base mode code.")
@click.option(
    "-c",
    "--custom-mode",
    type=str,
    default="",
    help="Custom mode string (same as in ~/state topic)",
)
@pass_client
@click.pass_context
def mode(ctx, client, base_mode, custom_mode):
    """Set mode."""
    custom_mode = custom_mode.upper()

    done_evt = threading.Event()

    def state_cb(state: State):
        client.verbose_echo(f"Current mode: {state.mode}")
        if state.mode == custom_mode:
            click.echo("Mode changed.")
            done_evt.set()

    if custom_mode != "" and not custom_mode.isdigit():
        # with correct custom mode we can wait until it changes
        client.system.subscribe_state(state_cb)
    else:
        done_evt.set()

    req = SetMode.Request(base_mode=base_mode, custom_mode=custom_mode)
    ret = client.system.cli_set_mode.call(req)

    if not ret.mode_sent:
        fault_echo(ctx, "Request failed. Check mavros logs")

    if not done_evt.wait(5):
        fault_echo(ctx, "Timed out!")


def _add_rate_options(*options: typing.List[str]):
    opts = [
        click.option(
            f"--{option.lower().replace(' ', '-')}",
            type=int,
            metavar="RATE",
            default=None,
            help=f"{option} stream",
        )
        for option in options
    ]

    def wrap(f):
        return apply_options(f, *opts)

    return wrap


@sys.command()
@_add_rate_options(
    "All",
    "Raw sensors",
    "Ext status",
    "RC channels",
    "Raw controller",
    "Position",
    "Extra1",
    "Extra2",
    "Extra3",
)
@click.option(
    "--stream-id",
    type=int,
    nargs=2,
    metavar="ID, RATE",
    default=None,
    help="custom stream stream",
)
@pass_client
@click.pass_context
def rate(
    ctx,
    client,
    all,
    raw_sensors,
    ext_status,
    rc_channels,
    raw_controller,
    position,
    extra1,
    extra2,
    extra3,
    stream_id,
):
    """Set stream rate."""
    _ = 1  # yapf

    def set_rate(rate_arg: typing.Optional[int], id_: int):
        if rate_arg is None:
            return

        req = StreamRate.Request(
            stream_id=id_,
            message_rate=rate_arg,
            on_off=(rate_arg != 0),
        )
        client.system.cli_set_stream_rate.call(req)

    set_rate(all, StreamRate.Request.STREAM_ALL)
    set_rate(raw_sensors, StreamRate.Request.STREAM_RAW_SENSORS)
    set_rate(ext_status, StreamRate.Request.STREAM_EXTENDED_STATUS)
    set_rate(rc_channels, StreamRate.Request.STREAM_RC_CHANNELS)
    set_rate(raw_controller, StreamRate.Request.STREAM_RAW_CONTROLLER)
    set_rate(position, StreamRate.Request.STREAM_POSITION)
    set_rate(extra1, StreamRate.Request.STREAM_EXTRA1)
    set_rate(extra2, StreamRate.Request.STREAM_EXTRA2)
    set_rate(extra3, StreamRate.Request.STREAM_EXTRA3)

    if stream_id:
        set_rate(stream_id[1], stream_id[0])


@sys.command()
@click.option("--id", type=int, metavar="MSGID", required=True, help="message id")
@click.option("--rate", type=float, metavar="RATE", required=True, help="message rate")
@pass_client
def message_interval(client, id, rate):
    """Set message interval."""
    req = MessageInterval.Request(message_id=id, message_rate=rate)
    client.system.cli_set_message_interval.call(req)
