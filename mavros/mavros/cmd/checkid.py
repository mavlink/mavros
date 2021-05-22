# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""
This script listens to devices connected to mavros and
checks against system & component id mismatch errors.
"""

import threading

import click
import rclpy

from mavros_msgs.msg import Mavlink

from ..utils import call_get_parameters
from . import CliClient, cli, pass_client


class Checker:
    def __init__(self, *, client: CliClient, follow: bool, watch_time: float):
        # dict of sets: (sysid, compid) -> set[msgid...]
        self.message_sources = {}
        self.messages_received = 0
        self.reports = 0
        self.client = client
        self.follow = follow
        self.event = threading.Event()

        params = call_get_parameters(
            node=client,
            names=['target_system_id', 'target_component_id', 'uas_url'])

        self.tgt_ids = (
            params['target_system_id'].value,
            params['target_component_id'].value,
        )
        uas_url = params['uas_url'].value
        source_topic = f"{uas_url}/mavlink_source"

        click.secho(
            f"Router topic: {source_topic}, target: {'.'.join(self.tgt_ids)}",
            fg='cyan')

        self.source_sub = client.create_subscription(Mavlink, source_topic,
                                                     1000,
                                                     self.mavlink_source_cb)
        self.timer = client.create_timer(watch_time, self.timer_cb)

    def mavlink_source_cb(self, msg: Mavlink):
        ids = (msg.sysid, msg.compid)
        if ids in self.message_sources:
            self.message_sources[ids].add(msg.msgid)
        else:
            self.message_sources[ids] = set((msg.msgid, ))

        self.messages_received += 1

    def timer_cb(self):
        if self.reports > 0:
            click.echo('-' * 80)

        self.reports += 1
        str_tgt_ids = f"{'.'.join(self.tgt_ids)}"

        if self.tgt_ids in self.message_sources:
            click.secho(f"OK. I got messages from {str_tgt_ids}.", fg='green')
        else:
            click.secho(
                f"ERROR. I got {len(self.message_sources)} addresses, but not your target {str_tgt_ids}",
                fg='red')

        click.secho("---", fg='cyan')
        click.secho(
            f"Received {self.messages_received}, from {len(self.message_sources)} addresses",
            fg='cyan')
        click.secho("address   list of messages", fg='cyan')
        for address, messages in self.message_sources.items():

            def fmt_msgid(msgid: int) -> str:
                # TODO(vooon): try to load pymavlink
                return f"{msgid}"

            str_ids = f"{'.'.join(address)}"
            click.secho(
                f"{str_ids: 6s}   {', '.join(fmt_msgid(msgid) for msgid in messages)}",
                fg='cyan')

        if not self.follow:
            self.event.set()


@cli
@click.option("-f",
              "--follow",
              is_flag=True,
              help="do not exit after first report.")
@click.option("--watch-time", type=float, default=15.0, help="watch period")
@pass_client
def checkid(client, follow, watch_time):
    """
    This script listens to devices connected to mavros and
    checks against system & component id mismatch errors.
    """

    checker = Checker(client=client, follow=follow, watch_time=watch_time)
    checker.event.wait()
