# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""mav param command."""

import typing

import click

from ..param import MavProxyParam, MissionPlannerParam, ParamFile, QGroundControlParam
from . import CliClient, cli, pass_client
from .utils import apply_options


@cli.group()
@pass_client
def param(client):
    """Tool to manipulate parameters of the MAVLink device."""


def _add_format_options(f):
    return apply_options(
        f,
        click.option(
            "-mp",
            "--mission-planner",
            "file_format",
            flag_value="mp",
            help="Select Mission Planner param file format",
        ),
        click.option(
            "-qgc",
            "--qgroundcontrol",
            "file_format",
            flag_value="qgc",
            help="Select QGroundControl param file format",
        ),
        click.option(
            "-mpx",
            "-mavpx",
            "--mavproxy",
            "file_format",
            flag_value="mpx",
            help="Select MAVProxy param file format",
        ),
    )


def get_param_file_io(
    client: CliClient, file_format: typing.Optional[str], file_: typing.TextIO
) -> ParamFile:
    if file_format == "mp":
        client.verbose_echo("MissionPlanner format")
        pf = MissionPlannerParam()

    elif file_format == "qgc":
        client.verbose_echo("QGroundControl format")
        pf = QGroundControlParam()

    elif file_format == "mpx":
        client.verbose_echo("MavProxy format")
        pf = MavProxyParam()

    else:
        if file_.name.endswith(".txt"):
            client.verbose_echo("Suggestion: QGroundControl format")
            pf = QGroundControlParam()
        else:
            client.verbose_echo("Suggestion: MissionPlanner format")
            pf = MissionPlannerParam()

    pf.tgt_system = client.uas_settings.target_system_id
    pf.tgt_component = client.uas_settings.target_component_id

    return pf


@param.command()
@_add_format_options
@click.argument("file_", type=click.File("r"), metavar="FILE")
@pass_client
def load(client, file_format, file_):
    """Load parameters from file."""
    param_file = get_param_file_io(client, file_format, file_)
    param_file.load(file_)

    client.param.values.update(param_file.parameters)

    client.verbose_echo(f"Prameters sent: {len(param_file.parameters)}")


@param.command()
@_add_format_options
@click.option(
    "-f",
    "--force",
    is_flag=True,
    default=False,
    help="Force pull params form FCU, update cache",
)
@click.argument("file_", type=click.File("w"), metavar="FILE")
@pass_client
def dump(client, file_format, force, file_):
    """Dump parameters to file."""
    # NOTE(vooon): hidden magic - create ParamDict and get ref
    values = client.param.values

    if force:
        client.param.call_pull(force=True)

    client.verbose_echo(f"Parameters received: {len(values)}")

    param_file = get_param_file_io(client, file_format, file_)
    param_file.parameters = values
    param_file.save(file_)


@param.command()
@click.argument("param_id", type=str)
@pass_client
def get(client, param_id):
    """Print one parameter value."""
    # XXX(vooon): ineffecient
    click.echo(f"{client.param.values[param_id].value}")


@param.command()
@click.argument("param_id", type=str)
@click.argument("value", type=str)
@pass_client
def set(client, param_id, value):
    """Set one parameter."""
    if "." in value:
        val = float(value)
    else:
        val = int(value)

    # XXX(vooon): ineffecient
    client.param.values[param_id] = val
    click.echo(f"{client.param.values[param_id].value}")
