# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""mav wp command."""

import threading
import typing

import click

from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import (
    WaypointClear,
    WaypointPull,
    WaypointPush,
    WaypointSetCurrent,
)

from ..mission import (
    FRAMES,
    NAV_CMDS,
    MissionPluginBase,
    PlanFile,
    QGroundControlPlan,
    QGroundControlWPL,
)
from . import CliClient, cli, pass_client
from .utils import apply_options, fault_echo

no_prettytable = False
try:
    from prettytable import PrettyTable
except ImportError:
    no_prettytable = True


@cli.group()
@pass_client
def wp(client):
    """Tool to manipulate missions on MAVLink device."""
    if no_prettytable:
        click.echo(
            "Waring: 'show' action disabled. Please install python3-prettytable",
            err=True,
        )


def _add_format_options(f):
    return apply_options(
        f,
        click.option(
            "-wpl",
            "--qgc-wpl",
            "file_format",
            flag_value="wpl",
            help="Select QGroundControl WPL mission file format (old CSV)",
        ),
        click.option(
            "-plan",
            "--qgc-plan",
            "file_format",
            flag_value="plan",
            help="Select QGroundControl Plan mission file format",
        ),
    )


def get_wp_file_io(
    client: CliClient, file_format: typing.Optional[str], file_: typing.TextIO
) -> PlanFile:
    if file_format == "wpl":
        return QGroundControlWPL()
    elif file_format == "plan":
        return QGroundControlPlan()
    else:
        if file_.name.endswith(".plan"):
            return QGroundControlPlan()
        else:
            return QGroundControlWPL()


def fmt_accessor(accessor: MissionPluginBase):
    return f"{accessor.__class__.__name__.replace('Plugin', '')}"


@wp.command()
@click.option(
    "--mission/--no-mission",
    "-m/-M",
    "pull_mission",
    default=True,
    help="Pull mission points",
)
@click.option(
    "--fence/--no-fence", "-f/-F", "pull_fence", default=True, help="Pull fence points"
)
@click.option(
    "--rally/--no-rally", "-r/-R", "pull_rally", default=True, help="Pull rally points"
)
@pass_client
@click.pass_context
def pull(ctx, client, pull_mission, pull_fence, pull_rally):
    """Pull mission from FCU."""
    _ = 1  # yapf

    def pull_if(cond: bool, accessor: MissionPluginBase):
        if not cond:
            return

        req = WaypointPull.Request()
        ret = accessor.cli_pull.call(req)

        if not ret.success:
            fault_echo(ctx, "Request failed. Check mavros logs")

        client.verbose_echo(f"{fmt_accessor(accessor)}(s) received: {ret.wp_received}")

    pull_if(pull_mission, client.waypoint)
    pull_if(pull_fence, client.geofence)
    pull_if(pull_rally, client.rallypoint)


@wp.command()
@click.option(
    "--mission", "accessor", flag_value="waypoint", default=True, help="Show mission"
)
@click.option("--fence", "accessor", flag_value="geofence", help="Show geo fence")
@click.option("--rally", "accessor", flag_value="rallypoint", help="Show rallypoints")
@click.option(
    "-p",
    "--pull",
    "pull_flag",
    is_flag=True,
    default=False,
    help="Pull points from FCU before show",
)
@click.option(
    "-f", "--follow", is_flag=True, default=False, help="Watch and show new data"
)
@pass_client
@click.pass_context
def show(ctx, client, accessor, pull_flag, follow):
    """
    Show current points.

    You can select type by setting flag:
      --mission - mission (default),
      --fence - geofence,
      --rally - rallypoints
    """
    if no_prettytable:
        fault_echo(ctx, "Show command require prettytable module!")

    def str_bool(x: bool) -> str:
        return "Yes" if x else "No"

    def str_frame(f: int) -> str:
        return f"{FRAMES.get(f, 'UNK')} ({f})"

    def str_command(c: int) -> str:
        return f"{NAV_CMDS.get(c, 'UNK')} ({c})"

    done_evt = threading.Event()

    def show_table(topic: WaypointList):
        pt = PrettyTable(
            (
                "#",
                "Curr",
                "Auto",
                "Frame",
                "Command",
                "P1",
                "P2",
                "P3",
                "P4",
                "X Lat",
                "Y Long",
                "Z Alt",
            )
        )

        for seq, w in enumerate(topic.waypoints):
            pt.add_row(
                (
                    seq,
                    str_bool(w.is_current),
                    str_bool(w.autocontinue),
                    str_frame(w.frame),
                    str_command(w.command),
                    w.param1,
                    w.param2,
                    w.param3,
                    w.param4,
                    w.x_lat,
                    w.y_long,
                    w.z_alt,
                )
            )

        click.echo(pt)
        if not follow:
            done_evt.set()

    if pull_flag:
        ctx.invoke(
            pull,
            pull_mission=accessor == "waypoint",
            pull_fence=accessor == "geofence",
            pull_rally=accessor == "rallypoint",
        )

    # Waypoints topic is latched type, and it updates after pull
    sub = getattr(client, accessor).subscribe_points(show_table)

    if follow:
        done_evt.wait()
    elif not done_evt.wait(30.0):
        fault_echo(ctx, "Something went wrong. Topic timed out.")

    del sub


@wp.command()
@_add_format_options
@click.option(
    "-p",
    "--preserve-home",
    is_flag=True,
    default=False,
    help="Preserve home location (WP 0, APM only)",
)
@click.option(
    "-s",
    "--start-index",
    type=int,
    default=0,
    help="Waypoint start index for partial update (APM only)",
)
@click.option(
    "-e",
    "--end-index",
    type=int,
    default=0,
    help="Waypoint end index for partial update " "(APM only, default: last element)",
)
@click.option(
    "--no-mission", "-M", is_flag=True, default=False, help="Don't load mission points"
)
@click.option(
    "--no-fence", "-F", is_flag=True, default=False, help="Don't load fence points"
)
@click.option(
    "--no-rally", "-R", is_flag=True, default=False, help="Don't load rally points"
)
@click.argument("file_", metavar="FILE", type=click.File("r"))
@pass_client
@click.pass_context
def load(
    ctx,
    client,
    file_format,
    preserve_home,
    start_index,
    end_index,
    no_mission,
    no_fence,
    no_rally,
    file_,
):
    """Load mission from file."""
    mission_file = get_wp_file_io(client, file_format, file_)
    mission_file.load(file_)

    def call_push(
        *,
        accessor: MissionPluginBase,
        points: typing.Optional[typing.List[Waypoint]],
        start_index: int = 0,
        no_send: bool = False,
    ):
        if points is None or no_send:
            return

        req = WaypointPush.Request(
            waypoints=points.waypoints,
            start_index=start_index,
        )
        ret = accessor.cli_push(req)

        if not ret.success:
            fault_echo("Request failed. Check mavros logs")

        client.verbose_echo(
            f"{fmt_accessor(accessor)}(s) transfered: {ret.wp_transfered}"
        )

    done_evt = threading.Event()

    def fix_wp0(topic: WaypointList):
        if len(topic.waypoints) > 0:
            wp0 = topic.waypoints[0]
            mission_file.mission[0] = wp0
            client.verbose_echo(
                f"HOME location: latitude: {wp0.x_lat}, "
                f"longitude: {wp0.y_long}, altitude: {wp0.z_alt}"
            )
        else:
            click.echo("Failed to get WP0! WP0 will be loaded from file.", err=True)

        done_evt.set()

    if start_index > 0:
        # Push partial
        if start_index == end_index:
            end_index += 1

        end_index = end_index or len(mission_file.mission)

        call_push(
            accessor=client.waypoint,
            points=mission_file.mission[start_index:end_index],
            start_index=start_index,
        )
        return

    if preserve_home:
        client.waypoint.subscribe_points(fix_wp0)
        if not done_evt.wait(30.0):
            fault_echo("Something went wrong. Topic timed out.")

    call_push(
        accessor=client.waypoint,
        points=mission_file.mission,
        start_index=0,
        no_send=no_mission,
    )
    call_push(
        accessor=client.geofence,
        points=mission_file.fence,
        start_index=0,
        no_send=no_fence,
    )
    call_push(
        accessor=client.rallypoint,
        points=mission_file.geofence,
        start_index=0,
        no_send=no_rally,
    )


@wp.command()
@_add_format_options
@click.option(
    "--no-mission", "-M", is_flag=True, default=False, help="Don't dump mission points"
)
@click.option(
    "--no-fence", "-F", is_flag=True, default=False, help="Don't dump fence points"
)
@click.option(
    "--no-rally", "-R", is_flag=True, default=False, help="Don't dump rally points"
)
@click.argument("file_", metavar="FILE", type=click.File("w"))
@pass_client
@click.pass_context
def dump(ctx, client, file_format, no_mission, no_fence, no_rally, file_):
    """Dump mission to file."""
    mission_file = get_wp_file_io(client, file_format, file_)

    fetch_mission = not no_mission
    fetch_fence = isinstance(mission_file, QGroundControlPlan) and not no_fence
    fetch_rally = isinstance(mission_file, QGroundControlPlan) and not no_rally

    ctx.invoke(
        pull,
        pull_mission=fetch_mission,
        pull_fence=fetch_fence,
        pull_rally=fetch_rally,
    )

    if fetch_mission:
        mission_file.mission = client.waypoint.points

    if fetch_fence:
        mission_file.fence = client.geofence.points

    if fetch_rally:
        mission_file.rally = client.rallypoint.points

    mission_file.save(file_)


@wp.command()
@click.option(
    "--mission/--no-mission",
    "-m/-M",
    "clear_mission",
    default=True,
    help="Clear mission points",
)
@click.option(
    "--fence/--no-fence",
    "-f/-F",
    "clear_fence",
    default=True,
    help="Clear fence points",
)
@click.option(
    "--rally/--no-rally",
    "-r/-R",
    "clear_rally",
    default=True,
    help="Clear rally points",
)
@pass_client
@click.pass_context
def clear(ctx, client, clear_mission, clear_fence, clear_rally):
    """Clear mission from FCU."""
    _ = 1  # yapf

    def clear_if(cond: bool, accessor: MissionPluginBase):
        if not cond:
            return

        req = WaypointClear.Request()
        ret = accessor.cli_clear.call(req)

        if not ret.success:
            fault_echo(ctx, "Request failed. Check mavros logs")

        client.verbose_echo(f"{fmt_accessor(accessor)}(s) cleared")

    clear_if(clear_mission, client.waypoint)
    clear_if(clear_fence, client.geofence)
    clear_if(clear_rally, client.rallypoint)


@wp.command()
@click.argument("seq", type=int)
@pass_client
@click.pass_context
def setcur(ctx, client, seq):
    """Set current mission item."""
    req = WaypointSetCurrent.Request(wp_seq=seq)
    ret = client.waypoint.cli_set_current.call(req)

    if not ret.success:
        fault_echo("Request failed, Check mavros logs")

    client.verbose_echo("Set current done.")
