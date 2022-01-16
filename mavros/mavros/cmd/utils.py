# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""Utilities for cli."""

import functools
import typing

import click

try:
    import pymavlink.dialects.v20.common as common_dialect
except ImportError:
    common_dialect = None


def fault_echo(ctx, *args, **kwargs):
    kwargs["err"] = True
    click.echo(*args, **kwargs)
    ctx.exit(1)


def fault_secho(ctx, *args, **kwargs):
    kwargs["err"] = True
    click.secho(*args, **kwargs)
    ctx.exit(1)


def check_cmd_ret(ctx, client, ret):
    # NOTE(vooon): APM returns MAV_RESULT

    # https://mavlink.io/en/messages/common.html#MAV_CMD_ACK
    # ename = 'MAV_CMD_ACK'
    # https://mavlink.io/en/messages/common.html#MAV_RESULT
    ename = "MAV_RESULT"

    ackstr = ""
    if hasattr(ret, "result"):
        if common_dialect is not None:
            ack = common_dialect.enums[ename].get(
                ret.result, common_dialect.EnumEntry("unknown", "")
            )
            ackstr = f" ACK: {ret.result} ({ack.name})"
        else:
            ackstr = f" ACK: {ret.result}"

    if not ret.success:
        fault_echo(ctx, f"Request failed. Check mavros logs.{ackstr}")

    if client.verbose:
        if hasattr(ret, "result"):
            click.echo(f"Command done.{ackstr}")
        else:
            click.echo("Request done.")


def bool2int(b: bool) -> int:
    """
    Convert bool to 1 or 0.

    I had an exception "TypeError: 'bool' object is not iterable"
    for code like int(confirmation).

    Why? Who knows...
    """
    if b:
        return 1
    return 0


def apply_options(
    func: typing.Callable, *opts: typing.List[typing.Callable]
) -> typing.Callable:
    """Apply several click.option to the same function."""
    return functools.reduce(lambda x, opt: opt(x), reversed(opts), func)
