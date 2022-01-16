# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

from . import (
    command,
    ftp,
    global_position,
    local_position,
    mission,
    param,
    setpoint,
    system,
)
from .base import BaseNode, cached_property


class Client(BaseNode):
    """
    Client provides some convinient methods to work with MAVROS API.

    Plugin interfaces are lazy constructed.

    NOTE: Client is a ROS2 Node.
    """

    @cached_property
    def system(self) -> system.SystemPlugin:
        return system.SystemPlugin(self)

    @cached_property
    def command(self) -> command.CommandPlugin:
        return command.CommandPlugin(self)

    @cached_property
    def param(self) -> param.ParamPlugin:
        return param.ParamPlugin(self)

    @cached_property
    def waypoint(self) -> mission.WaypointPlugin:
        return mission.WaypointPlugin(self)

    @cached_property
    def geofence(self) -> mission.GeofencePlugin:
        return mission.GeofencePlugin(self)

    @cached_property
    def rallypoint(self) -> mission.RallypointPlugin:
        return mission.RallypointPlugin(self)

    @cached_property
    def setpoint_accel(self) -> setpoint.SetpointAccelPlugin:
        return setpoint.SetpointAccelPlugin(self)

    @cached_property
    def setpoint_attitude(self) -> setpoint.SetpointAttitudePlugin:
        return setpoint.SetpointAttitudePlugin(self)

    @cached_property
    def setpoint_position(self) -> setpoint.SetpointPositionPlugin:
        return setpoint.SetpointPositionPlugin(self)

    @cached_property
    def setpoint_raw(self) -> setpoint.SetpointRawPlugin:
        return setpoint.SetpointRawPlugin(self)

    @cached_property
    def setpoint_trajectory(self) -> setpoint.SetpointTrajectoryPlugin:
        return setpoint.SetpointTrajectoryPlugin(self)

    @cached_property
    def setpoint_velocity(self) -> setpoint.SetpointVelocityPlugin:
        return setpoint.SetpointVelocityPlugin(self)

    @cached_property
    def ftp(self) -> ftp.FTPPlugin:
        return ftp.FTPPlugin(self)

    @cached_property
    def global_position(self) -> global_position.GlobalPositionPlugin:
        return global_position.GlobalPositionPlugin(self)

    @cached_property
    def local_position(self) -> local_position.LocalPositionPlugin:
        return local_position.LocalPositionPlugin(self)
