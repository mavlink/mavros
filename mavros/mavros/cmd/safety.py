#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""
mav safety command.

Allow user to Arm, Disarm and Kill vehicle motors.
"""

import click

from mavros_msgs.srv import CommandBool, CommandLong

from . import cli, pass_client
from .utils import check_cmd_ret


@cli.group()
@pass_client
def safety(client):
    """Tool to send safety commands to MAVLink device."""


def _arm(ctx, client, state: bool):
    req = CommandBool.Request(value=state)

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_arming.call(req)
    check_cmd_ret(ctx, client, ret)


@safety.command()
@pass_client
@click.pass_context
def arm(ctx, client):
    """Arm motors."""
    _arm(ctx, client, True)


@safety.command()
@pass_client
@click.pass_context
def disarm(ctx, client):
    """Disarm motors."""
    _arm(ctx, client, False)


@safety.command()
@pass_client
@click.pass_context
def kill(ctx, client):
    """Kill motors."""
    req = CommandLong.Request(command=400, param2=21196.0)

    client.verbose_echo(f"Calling: {req}")
    ret = client.command.cli_long.call(req)
    check_cmd_ret(ctx, client, ret)
