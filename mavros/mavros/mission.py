# -*- python -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import csv
import typing
from collections import OrderedDict

import rclpy
from mavros_msgs.msg import CommandCode, Waypoint, WaypointList
from mavros_msgs.srv import (WaypointClear, WaypointPull, WaypointPush,
                             WaypointSetCurrent)

from .base import STATE_QOS, PluginModule, cached_property

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
    # Maybe later i will add this enum to message
    112: 'COND-DELAY',
    113: 'COND-CHANGE-ALT',
    114: 'COND-DISTANCE',
    115: 'COND-YAW',
    177: 'DO-JUMP',
    178: 'DO-CHANGE-SPEED',
    181: 'DO-SET-RELAY',
    182: 'DO-REPEAT-RELAY',
    183: 'DO-SET-SERVO',
    184: 'DO-REPEAT-SERVO',
    201: 'DO-SET-ROI',
}


class PlanFile:
    """Base class for waypoint file parsers"""

    mission: typing.Optional[typing.List[Waypoint]] = None
    fence: typing.Optional[typing.List[Waypoint]] = None
    rally: typing.Optional[typing.List[Waypoint]] = None

    def load(self, file_: typing.TextIO):
        """Returns a iterable of waypoints"""
        raise NotImplementedError

    def save(self, file_: typing.TextIO):
        """Writes waypoints to file"""
        raise NotImplementedError


class QGroundControlWPL(PlanFile):
    """Parse QGC waypoint file"""

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

    @cached_property
    def pull(self) -> rclpy.node.Client:
        return self._node.create_client(
            WaypointPull, self._node.get_topic(self._plugin_ns, 'pull'))

    @cached_property
    def push(self) -> rclpy.node.Client:
        return self._node.create_client(
            WaypointPush, self._node.get_topic(self._plugin_ns, 'push'))

    @cached_property
    def clear(self) -> rclpy.node.Client:
        return self._node.create_client(
            WaypointClear, self._node.get_topic(self._plugin_ns, 'clear'))

    def subscribe_points(
            self,
            callback: rclpy.Callable,
            qos: rclpy.QoSProfile = STATE_QOS) -> rclpy.Subscription:
        """
        Subscribe to points list (waypoints, fences, rallypoints)
        """
        return self._node.create_subscription(
            WaypointList,
            self._node.get_topic(self._plugin_ns, self._plugin_list_topic),
            callback, qos)


class WaypointPlugin(MissionPluginBase):
    """
    Interface to waypoint plugin
    """
    @cached_property
    def set_current(self) -> rclpy.node.Client:
        return self._node.create_client(
            WaypointSetCurrent,
            self._node.get_topic(self._plugin_ns, 'set_current'))


class GeofencePlugin(MissionPluginBase):
    """
    Interface to geofence plugin
    """

    _plugin_ns = 'geofence'
    _plugin_list_topic = 'fences'


class RallypointPlugin(MissionPluginBase):
    """
    Interface to rallypoint plugin
    """

    _plugin_ns = 'rallypoint'
    _plugin_list_topic = 'rallypoints'
