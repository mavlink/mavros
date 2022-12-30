# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""MAVROS helper tools."""

import pathlib
import typing

import click
import rclpy

from .. import Client
from ..base import DEFAULT_NAMESPACE


class CliClient:
    def __init__(
        self,
        *,
        node_name: typing.Optional[str] = None,
        mavros_ns: str = DEFAULT_NAMESPACE,
        verbose: bool = False,
    ):
        self.cli = Client(node_name=node_name, mavros_ns=mavros_ns)
        self.verbose = verbose

    def __getattr__(self, key: str):
        try:
            return object.__getattribute__(self, key)
        except AttributeError:
            return getattr(self.cli, key)

    def __repr__(self) -> str:
        return f"<CliClient cli={self.cli!r}>"

    def verbose_echo(self, *args, **kwargs):
        if self.verbose:
            click.echo(*args, **kwargs)

    def verbose_secho(self, *args, **kwargs):
        if self.verbose:
            click.secho(*args, **kwargs)


pass_client = click.make_pass_decorator(CliClient)


def print_version(ctx, param_, value):
    if not value or ctx.resilient_parsing:
        return

    import xml.etree.ElementTree as ET  # nosemgrep

    import ament_index_python as aip

    share_dir = aip.get_package_share_directory("mavros")
    package_xml = pathlib.Path(share_dir) / "package.xml"

    tree = ET.parse(package_xml)
    versions = tree.getroot().findall("version")

    click.echo(f"MAVROS Version: {versions[0].text}")
    ctx.exit()


@click.group()
@click.option(
    "--node-name",
    type=str,
    default=None,
    envvar="MAVCLI_NODE_NAME",
    help="Set node name to cli's Node, default: random",
)
@click.option(
    "--mavros-ns",
    type=str,
    default=DEFAULT_NAMESPACE,
    envvar="MAVCLI_MAVROS_NS",
    help="Set namespace of mavros::UAS Node",
)
@click.option(
    "--verbose",
    type=bool,
    default=False,
    is_flag=True,
    envvar="MAVCLI_VERBOSE",
    help="Verbose output",
)
@click.option(
    "--wait-fcu",
    type=float,
    is_flag=False,
    flag_value=None,
    envvar="MAVCLI_WAIT_FCU",
    default=False,
    help="Wait for establishing FCU connection",
)
@click.option(
    "--version",
    is_flag=True,
    callback=print_version,
    expose_value=False,
    is_eager=True,
    help="Show program version and exit",
)
@click.pass_context
def cli(ctx, node_name, mavros_ns, verbose, wait_fcu):
    """MAVROS tools entry point."""
    spinner_thd = None

    def on_close():
        rclpy.shutdown()
        if spinner_thd:
            spinner_thd.join()

    rclpy.init()
    ctx.call_on_close(on_close)

    ctx.obj = CliClient(node_name=node_name, mavros_ns=mavros_ns, verbose=verbose)
    spinner_thd = ctx.obj.start_spinner()

    if wait_fcu or wait_fcu is None:
        ctx.obj.verbose_echo("Waiting connection to the FCU...")
        ctx.obj.system.wait_fcu_connection(wait_fcu)


from . import checkid, cmd, ftp, mission, param, safety, system  # NOQA
