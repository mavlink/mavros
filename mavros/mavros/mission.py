# -*- python -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import csv
import threading
import typing
from collections import OrderedDict

import rclpy

from mavros_msgs.msg import CommandCode, Waypoint, WaypointList
from mavros_msgs.srv import (WaypointClear, WaypointPull, WaypointPush,
                             WaypointSetCurrent)

from .base import (SERVICE_WAIT_TIMEOUT, STATE_QOS, PluginModule,
                   ServiceWaitTimeout, SubscriptionCallable, cached_property)

FRAMES = {
    Waypoint.FRAME_GLOBAL: 'GAA',
    Waypoint.FRAME_GLOBAL_REL_ALT: 'GRA',
    Waypoint.FRAME_LOCAL_ENU: 'LOC-ENU',
    Waypoint.FRAME_LOCAL_NED: 'LOC-NED',
    Waypoint.FRAME_MISSION: 'MIS'
}

NAV_CMDS = {
    CommandCode.NAV_LAND: 'LAND',
    CommandCode.NAV_LOITER_TIME: 'LOITER-TIME',
    CommandCode.NAV_LOITER_TURNS: 'LOITER-TURNS',
    CommandCode.NAV_LOITER_UNLIM: 'LOITER-UNLIM',
    CommandCode.NAV_RETURN_TO_LAUNCH: 'RTL',
    CommandCode.NAV_TAKEOFF: 'TAKEOFF',
    CommandCode.NAV_WAYPOINT: 'WAYPOINT',
    CommandCode.CONDITION_DELAY: 'COND-DELAY',
    CommandCode.CONDITION_CHANGE_ALT: 'COND-CHANGE-ALT',
    CommandCode.CONDITION_DISTANCE: 'COND-DISTANCE',
    CommandCode.CONDITION_YAW: 'COND-YAW',
    CommandCode.CONDITION_GATE: 'COND-GATE',
    CommandCode.DO_JUMP: 'DO-JUMP',
    CommandCode.DO_CHANGE_SPEED: 'DO-CHANGE-SPEED',
    CommandCode.DO_SET_RELAY: 'DO-SET-RELAY',
    CommandCode.DO_REPEAT_RELAY: 'DO-REPEAT-RELAY',
    CommandCode.DO_SET_SERVO: 'DO-SET-SERVO',
    CommandCode.DO_REPEAT_SERVO: 'DO-REPEAT-SERVO',
    CommandCode.DO_SET_ROI: 'DO-SET-ROI',
    CommandCode.NAV_FENCE_RETURN_POINT: 'FENCE-RETURN',
    CommandCode.NAV_FENCE_POLYGON_VERTEX_INCLUSION: 'FENCE-VERTEX-INC',
    CommandCode.NAV_FENCE_POLYGON_VERTEX_EXCLUSION: 'FENCE-VERTEX-EXC',
    CommandCode.NAV_FENCE_CIRCLE_INCLUSION: 'FENCE-CIRCLE-INC',
    CommandCode.NAV_FENCE_CIRCLE_EXCLUSION: 'FENCE-CIRCLE-EXC',
    CommandCode.NAV_RALLY_POINT: 'RALLY',
}


class PlanFile:
    """Base class for waypoint file parsers."""

    mission: typing.Optional[typing.List[Waypoint]] = None
    fence: typing.Optional[typing.List[Waypoint]] = None
    rally: typing.Optional[typing.List[Waypoint]] = None

    def load(self, file_: typing.TextIO):
        """Return a iterable of waypoints."""
        raise NotImplementedError

    def save(self, file_: typing.TextIO):
        """Write waypoints to file."""
        raise NotImplementedError


class QGroundControlWPL(PlanFile):
    """Parse QGC waypoint file."""

    file_header = 'QGC WPL 120'
    known_versions = (110, 120)

    fields_map = OrderedDict(
        is_current=lambda x: bool(int(x)),
        frame=int,
        command=float,
        param1=float,
        param2=float,
        param3=float,
        param4=float,
        x_lat=float,
        y_long=float,
        z_alt=float,
        autocontinue=lambda x: bool(int(x)),
    )

    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE

    def _parse_wpl_file(self, file_: typing.TextIO):
        got_header = False
        dict_reader = csv.DictReader(file_,
                                     self.fields_map.keys(),
                                     restkey='_more',
                                     restval='_less',
                                     dialect=self.CSVDialect)
        for data in dict_reader:
            if not got_header:
                qgc, wpl, ver = data['_less'].split(' ', 3)
                ver = int(ver)
                if qgc == 'QGC' and wpl == 'WPL' \
                        and ver in self.known_versions:
                    got_header = True

            else:
                yield Waypoint(**{
                    self.fields_map[k](v)
                    for k, v in data if k in self.fields_map
                })

    def load(self, file_: typing.TextIO):
        self.mission = list(self._parse_wpl_file(file_))
        self.fence = None
        self.rally = None

    def save(self, file_: typing.TextIO):
        assert self.mission is not None, "empty mission"
        assert self.fence is None, "WPL do not support geofences"
        assert self.rally is None, "WPL do not support rallypoints"

        writer = csv.writer(file_, self.CSVDialect)
        writer.writerow((self.file_header, ))
        for seq, w in enumerate(self.waypoints):
            row = (seq, ) + (getattr(w, k) for k in self.fields_map.keys())
            writer.writerow(row)


class QGroundControlPlan(PlanFile):
    pass  # TODO(vooon): implement me!


class MissionPluginBase(PluginModule):

    _plugin_ns = 'mission'
    _plugin_list_topic = 'waypoints'

    _points: typing.List[Waypoint] = []
    _points_sub = None

    @cached_property
    def cli_pull(self) -> rclpy.node.Client:
        return self.create_client(WaypointPull, (self._plugin_ns, 'pull'))

    @cached_property
    def cli_push(self) -> rclpy.node.Client:
        return self.create_client(WaypointPush, (self._plugin_ns, 'push'))

    @cached_property
    def cli_clear(self) -> rclpy.node.Client:
        return self.create_client(WaypointClear, (self._plugin_ns, 'clear'))

    def subscribe_points(
        self,
        callback: SubscriptionCallable,
        qos_profile: rclpy.qos.QoSProfile = STATE_QOS
    ) -> rclpy.node.Subscription:
        """Subscribe to points list (waypoints, fences, rallypoints)."""
        return self.create_subscription(
            WaypointList, (self._plugin_ns, self._plugin_list_topic), callback,
            qos_profile)

    @property
    def points(self) -> typing.List[Waypoint]:
        """Subscribe and return points cache."""
        if self._points_sub is not None:
            return self._points

        done_evt = threading.Event()

        def handler(ml: WaypointList):
            self._points = ml.waypoints
            done_evt.set()

        self._points_sub = self.subscribe_points(handler)
        if not done_evt.wait(SERVICE_WAIT_TIMEOUT):
            raise ServiceWaitTimeout(
                f"timeout waiting for {self._points_sub.topic_name}")

        return self._points


class WaypointPlugin(MissionPluginBase):
    """Interface to waypoint plugin."""

    @cached_property
    def cli_set_current(self) -> rclpy.node.Client:
        return self._node.create_client(WaypointSetCurrent,
                                        (self._plugin_ns, 'set_current'))


class GeofencePlugin(MissionPluginBase):
    """Interface to geofence plugin."""

    _plugin_ns = 'geofence'
    _plugin_list_topic = 'fences'


class RallypointPlugin(MissionPluginBase):
    """Interface to rallypoint plugin."""

    _plugin_ns = 'rallypoint'
    _plugin_list_topic = 'rallypoints'
