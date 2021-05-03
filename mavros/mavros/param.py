# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import csv
import datetime
import typing

from mavros_msgs.msg import ParamEvent
from mavros_msgs.srv import ParamPull, ParamSetV2
from rcl_interfaces import GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter

from .base import PluginModule, SubscriptionCallable, cached_property


class ParamFile:
    """Base class for param file parsers"""

    parameters: typing.Optional[typing.Dict[Parameter]] = None
    stamp: typing.Optional[datetime.datetime] = None
    tgt_system = 1
    tgt_component = 1

    def load(self, file_: typing.TextIO):
        """Returns a iterable of Parameters"""
        raise NotImplementedError

    def save(self, file_: typing.TextIO):
        """Writes Parameters to file"""
        raise NotImplementedError


class MavProxyParam(ParamFile):
    """Parse MavProxy param files"""
    class CSVDialect(csv.Dialect):
        delimiter = ' '
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE

    def _parse_param_file(self, file_: typing.TextIO):
        to_numeric = lambda x: float(x) if '.' in x else int(x)

        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue  # skip comments

            if len(data) != 2:
                raise ValueError("wrong field count")

            yield Parameter(data[0].strip(), value=to_numeric(data[1]))

    def load(self, file_: typing.TextIO):
        self.parameters = {p.name: p for p in self._parse_param_file(file_)}

    def save(self, file_: typing.TextIO):
        if self.stamp is None:
            self.stamp = datetime.datetime.now()

        writer = csv.writer(file_, self.CSVDialect)
        file_.writerow((f"""#NOTE: {self.stamp.strftime("%d.%m.%Y %T")}""", ))
        for p in self.parameters:
            writer.writerow((p.name, p.value))


class MissionPlannerParam(MavProxyParam):
    """Parse MissionPlanner param files"""
    class CSVDialect(csv.Dialect):
        delimiter = ','
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE


class QGroundControlParam(ParamFile):
    """Parse QGC param files"""
    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\n'
        quoting = csv.QUOTE_NONE

    def _parse_param_file(self, file_: typing.TextIO):
        to_numeric = lambda x: float(x) if '.' in x else int(x)

        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue  # skip comments

            if len(data) != 5:
                raise ValueError("wrong field count")

            yield Parameter(data[2].strip(), value=to_numeric(data[3]))

    def load(self, file_: typing.TextIO):
        self.parameters = {p.name: p for p in self._parse_param_file(file_)}

    def save(self, file_: typing.TextIO):
        def to_type(x):
            if isinstance(x, float):
                return 9  # REAL32
            elif isinstance(x, int):
                return 6  # INT32
            else:
                raise ValueError(f"unknown type: {type(x):r}")

        if self.stamp is None:
            self.stamp = datetime.datetime.now()

        writer = csv.writer(file_, self.CSVDialect)
        writer.writerow(
            (f"""# NOTE: {self.stamp.strftime("%d.%m.%Y %T")}""", ))
        writer.writerow((
            f"# Onboard parameters saved by mavparam for ({self.tgt_system}.{self.tgt_component})",
        ))
        writer.writerow(
            ("# MAV ID", "COMPONENT ID", "PARAM NAME", "VALUE", "(TYPE)"))
        for p in self.parameters:
            writer.writerow((
                self.tgt_system,
                self.tgt_component,
                p.name,
                p.value,
                to_type(p.value),
            ))


class ParamPlugin(PluginModule):
    """
    Parameter plugin client
    """
    @cached_property
    def list_parameters(self) -> rclpy.node.Client:
        return self.create_client(ListParameters, ('param', 'list_parameters'))

    @cached_property
    def get_parameters(self) -> rclpy.node.Client:
        return self.create_client(GetParameters, ('param', 'get_parameters'))

    @cached_property
    def set_parameters(self) -> rclpy.node.Client:
        return self.create_client(SetParameters, ('param', 'set_parameters'))

    @cached_property
    def pull(self) -> rclpy.node.Client:
        return self.create_client(ParamPull, ('param', 'pull'))

    @cached_property
    def set(self) -> rclpy.node.Client:
        return self.create_client(ParamSetV2, ('param', 'set'))

    def subscribe_events(
        self,
        callback: SubscriptionCallable,
        qos_profile: rclpy.qos.QoSProfile = rclpy.qos.QoSPresetProfiles.
        PARAMETERS
    ) -> rclpy.node.Subscription:
        """
        Subscribe to parameter events
        """
        return self.create_subscription(ParamEvent, ('param', 'event'),
                                        callback, qos_profile)
