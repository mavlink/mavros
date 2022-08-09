# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""
Check ID diagnostic.

This script listens to devices connected to mavros and
checks against system & component id mismatch errors.
"""

import threading
import typing

import click
import rclpy.qos

from mavros_msgs.msg import Mavlink

from . import CliClient, cli, pass_client
from .utils import common_dialect

ROUTER_QOS = rclpy.qos.QoSProfile(
    depth=1000,
    reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
)


class Checker:
    def __init__(self, *, client: CliClient, follow: bool, watch_time: float):
        # dict of sets: (sysid, compid) -> set[msgid...]
        self.message_sources = {}
        self.messages_received = 0
        self.reports = 0
        self.client = client
        self.follow = follow
        self.event = threading.Event()

        self.tgt_ids = client.uas_settings.target_ids
        uas_url = client.uas_settings.uas_url
        source_topic = f"{uas_url}/mavlink_source"

        click.secho(
            f"Router topic: {source_topic}, target: {self.fmt_ids(self.tgt_ids)}",
            fg="cyan",
        )

        self.source_sub = client.create_subscription(
            Mavlink, source_topic, self.mavlink_source_cb, ROUTER_QOS
        )
        self.timer = client.create_timer(watch_time, self.timer_cb)

    def fmt_ids(self, ids: typing.Tuple[int]) -> str:
        return f"{'.'.join(f'{v}' for v in ids)}"

    def mavlink_source_cb(self, msg: Mavlink):
        self.client.verbose_secho(f"Msg: {msg}", fg="magenta")

        ids = (msg.sysid, msg.compid)
        if ids in self.message_sources:
            self.message_sources[ids].add(msg.msgid)
        else:
            self.message_sources[ids] = set((msg.msgid,))

        self.messages_received += 1

    def timer_cb(self):
        if self.reports > 0:
            click.echo("-" * 80)

        self.reports += 1
        str_tgt_ids = self.fmt_ids(self.tgt_ids)

        if self.tgt_ids in self.message_sources:
            click.secho(f"OK. I got messages from {str_tgt_ids}.", fg="green")
        else:
            click.secho(
                f"ERROR. I got {len(self.message_sources)} addresses, "
                f"but not your target {str_tgt_ids}",
                fg="red",
            )

        click.secho("---", fg="cyan")
        click.secho(
            f"Received {self.messages_received}, from {len(self.message_sources)} addresses",
            fg="cyan",
        )
        click.secho("address   list of messages", fg="cyan")
        for address, messages in self.message_sources.items():

            def fmt_msgid(msgid: int) -> str:
                if common_dialect is None:
                    return f"{msgid}"

                msg = common_dialect.mavlink_map.get(msgid)
                if msg is None:
                    return f"{msgid}"
                else:
                    msg_name = ""
                    # Since pymavlink version 2.4.32 `name` is renamed to `msgname`.
                    # We want to stay compatible with prior versions of pymavlink.
                    if hasattr(msg, "msgname"):
                        msg_name = msg.msgname
                    else:
                        msg_name = msg.name
                    return f"{msgid} ({msg_name})"

            str_ids = self.fmt_ids(address)
            click.secho(
                f"{str_ids:>7s}   {', '.join(fmt_msgid(msgid) for msgid in messages)}",
                fg="white",
            )

        if not self.follow:
            self.event.set()


@cli.command()
@click.option("-f", "--follow", is_flag=True, help="do not exit after first report.")
@click.option("--watch-time", type=float, default=15.0, help="watch period")
@pass_client
def checkid(client, follow, watch_time):
    """Tool to verify target address and list messages coming to mavros UAS."""
    checker = Checker(client=client, follow=follow, watch_time=watch_time)
    checker.event.wait()
