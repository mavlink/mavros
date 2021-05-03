# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

from . import command, mission, system
from .base import BaseNode, cached_property


class Client(BaseNode):
    """
    Client provides some convinient methods to work with MAVROS API
    """
    @cached_property
    def system(self) -> command.SystemPlugin:
        return system.SystemPlugin(self)

    @cached_property
    def command(self) -> command.CommandPlugin:
        return command.CommandPlugin(self)

    @cached_property
    def waypoint(self) -> mission.WaypointPlugin:
        return mission.WaypointPlugin(self)

    @cached_property
    def geofence(self) -> mission.GeofencePlugin:
        return mission.GeofencePlugin(self)

    @cached_property
    def rallypoint(self) -> mission.RallypointPlugin:
        return mission.RallypointPlugin(self)
