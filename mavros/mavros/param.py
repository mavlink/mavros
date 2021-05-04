# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import csv
import datetime
import typing

import rclpy
from mavros_msgs.msg import ParamEvent
from mavros_msgs.srv import ParamPull, ParamSetV2
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter

from .base import (PluginModule, ServiceWaitTimeout, SubscriptionCallable,
                   cached_property)

TIMEOUT = 5.0


class ParamFile:
    """Base class for param file parsers"""

    parameters: typing.Optional[typing.Dict[str, Parameter]] = None
    stamp: typing.Optional[datetime.datetime] = None
    tgt_system: int = 1
    tgt_component: int = 1

    def load(self, file_: typing.TextIO) -> 'ParamFile':
        """Returns a iterable of Parameters"""
        raise NotImplementedError

    def save(self, file_: typing.TextIO):
        """Writes Parameters to file"""
        raise NotImplementedError


class MavProxyParam(ParamFile):
    """Parse MavProxy parm files"""
    class CSVDialect(csv.Dialect):
        delimiter = ' '
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE
        escapechar = ''

    def _parse_param_file(self, file_: typing.TextIO):
        def to_numeric(x):
            return float(x) if '.' in x else int(x)

        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue  # skip comments

            if len(data) != 2:
                raise ValueError("wrong field count")

            yield Parameter(data[0].strip(), value=to_numeric(data[1]))

    def load(self, file_: typing.TextIO) -> ParamFile:
        self.parameters = {p.name: p for p in self._parse_param_file(file_)}
        return self

    def save(self, file_: typing.TextIO):
        if self.stamp is None:
            self.stamp = datetime.datetime.now()

        writer = csv.writer(file_, self.CSVDialect)
        file_.write(
            f"""#NOTE: {self.stamp.strftime("%d.%m.%Y %T")}{self.CSVDialect.lineterminator}"""
        )
        for k, p in self.parameters.items():
            writer.writerow((p.name, p.value))


class MissionPlannerParam(MavProxyParam):
    """Parse MissionPlanner param files"""
    class CSVDialect(csv.Dialect):
        delimiter = ','
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE
        escapechar = ''


class QGroundControlParam(ParamFile):
    """Parse QGC param files"""
    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\n'
        quoting = csv.QUOTE_NONE
        escapechar = ''

    def _parse_param_file(self, file_: typing.TextIO):
        def to_numeric(x):
            return float(x) if '.' in x else int(x)

        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue  # skip comments

            if len(data) != 5:
                raise ValueError("wrong field count")

            yield Parameter(data[2].strip(), value=to_numeric(data[3]))

    def load(self, file_: typing.TextIO) -> ParamFile:
        self.parameters = {p.name: p for p in self._parse_param_file(file_)}
        return self

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
        for k, p in self.parameters.items():
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

    timeout_sec: float = 5.0
    _parameters = None
    _event_sub = None

    @cached_property
    def list_parameters(self) -> rclpy.node.Client:
        """ListParameters service client"""
        return self.create_client(ListParameters, ('param', 'list_parameters'))

    @cached_property
    def get_parameters(self) -> rclpy.node.Client:
        """GetParameters service client"""
        return self.create_client(GetParameters, ('param', 'get_parameters'))

    @cached_property
    def set_parameters(self) -> rclpy.node.Client:
        """SetParameters service client"""
        return self.create_client(SetParameters, ('param', 'set_parameters'))

    @cached_property
    def pull(self) -> rclpy.node.Client:
        """ParamPull service client"""
        return self.create_client(ParamPull, ('param', 'pull'))

    @cached_property
    def set(self) -> rclpy.node.Client:
        """ParamSetV2 service client"""
        return self.create_client(ParamSetV2, ('param', 'set'))

    def subscribe_events(
        self,
        callback: SubscriptionCallable,
        qos_profile: rclpy.qos.QoSProfile = rclpy.qos.QoSPresetProfiles.
        PARAMETERS
    ) -> rclpy.node.Subscription:
        """Subscribe to parameter events"""
        return self.create_subscription(ParamEvent, ('param', 'event'),
                                        callback, qos_profile)

    def call_pull(self, *, force_pull: bool = False) -> ParamPull.Response:
        """Do a call to ParamPull service"""
        lg = self._node.get_logger()

        ready = self.pull.wait_for_service(timeout_sec=self.timeout_sec)
        if not ready:
            raise ServiceWaitTimeout()

        req = ParamPull.Request()
        req.force_pull = force_pull

        future = self.pull.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)

        resp = future.result()
        lg.debug(f"pull result: {resp}")

        return resp

    @property
    def values(self) -> 'ParamDict':
        """Provides current state of parameters and allows to change them."""
        if self._parameters is not None:
            return self._parameters

        pm = ParamDict()
        pm._pm = self

        # 1. subscribe for parameter updates
        self._event_sub = self.subscribe_events(pm._event_handler)
        self._parameters = pm

        # 2. pull parameters, if it isn't yet done
        #    we'll get bunch of events
        self.call_pull()

        # 3. if to little events come, request whole list
        if len(pm) < 10:
            names = call_list_parameters(node=self._node,
                                         client=self.list_parameters)
            for k, v in call_get_parameters(node=self._node,
                                            client=self.get_parameters,
                                            names=names).items():
                pm.setdefault(k, v)

        return pm


class ParamDict(dict):
    """
    ParamDict class holds states of parameters
    and allow to upload new items
    """
    class NoSet:
        """Wrapper to mark values we do not want to send set request for"""
        value: Parameter

        def __init__(self, p):
            self.value = p

    _pm: 'ParamPlugin' = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, key: str) -> Parameter:
        return super().__getitem__(key)

    def __setitem__(self, key: str, value):
        if self._set_item(key, value):
            call_set_parameters_check_and_raise(node=self._pm._node,
                                                client=self._pm.set_parameters,
                                                parameters=[value])

    def _set_item(self, key: str, value) -> bool:
        is_no_set = False
        if isinstance(value, ParamDict.NoSet):
            is_no_set = True
            value = value.value

        if isinstance(value, Parameter):
            pass
        elif isinstance(value, ParameterValue):
            value = parameter_from_parameter_value(key, value)
        elif isinstance(value, ParameterMsg):
            value = Parameter.from_parameter_msg(value)
        else:
            value = Parameter(name=key, value=value)

        assert key == value.name

        do_call_set = not is_no_set and self.get(key,
                                                 Parameter(name=key)) != value
        super().__setitem__(key, value)
        return do_call_set

    def __getattr__(self, key: str):
        try:
            return object.__getattribute__(self, key)
        except AttributeError:
            try:
                return self[key]
            except KeyError:
                raise AttributeError(key)

    def __setattr__(self, key: str, value):
        try:
            object.__getattribute__(self, key)
        except AttributeError:
            try:
                self[key] = value
            except Exception as ex:
                raise AttributeError(f"{key}: {ex}")
        else:
            object.__setattr__(self, key, value)

    def __delattr__(self, key: str):
        try:
            object.__getattribute__(self, key)
        except AttributeError:
            try:
                del self[key]
            except KeyError:
                raise AttributeError(key)
        else:
            object.__delattr__(self, key)

    def update(self, *args, **kwargs):
        keys_to_set = []
        for k, v in dict(*args, **kwargs).items():
            if self._set_item(k, v):
                keys_to_set.append(k)

        if keys_to_set:
            call_set_parameters_check_and_raise(
                node=self._pm._node,
                client=self._pm.set_parameters,
                parameters=[self[k] for k in keys_to_set])

    def setdefault(self, key: str, value=None):
        if key not in self:
            self[key] = ParamDict.NoSet(value)

    def _event_handler(self, msg: ParamEvent):
        self[msg.param_id] = parameter_from_parameter_value(
            msg.param_id, msg.value)


def call_list_parameters(*,
                         node: rclpy.node.Node,
                         node_name: typing.Optional[str] = None,
                         client: typing.Optional[rclpy.node.Client] = None,
                         prefixes: typing.List[str] = []) -> typing.List[str]:
    lg = node.get_logger()

    if client is None:
        assert node_name is not None
        client = node.create_client(ListParameters,
                                    f"{node_name}/list_parameters")

    ready = client.wait_for_service(timeout_sec=TIMEOUT)
    if not ready:
        lg.error("wait for service time out")
        raise ServiceWaitTimeout()

    req = ListParameters.Request()
    req.prefixes = prefixes

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    resp = future.result()
    lg.debug(f"list result: {resp}")

    return resp.result.names


def call_get_parameters(
        *,
        node: rclpy.node.Node,
        node_name: typing.Optional[str] = None,
        client: typing.Optional[rclpy.node.Client] = None,
        names: typing.List[str] = []) -> typing.Dict[str, ParameterValue]:
    lg = node.get_logger()

    if client is None:
        assert node_name is not None
        client = node.create_client(GetParameters,
                                    f"{node_name}/get_parameters")

    ready = client.wait_for_service(timeout_sec=TIMEOUT)
    if not ready:
        lg.error("wait for service time out")
        raise ServiceWaitTimeout()

    req = GetParameters.Request()
    req.names = names

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    resp = future.result()
    lg.debug(f"get result: {resp}")

    return dict(zip(names, resp.values))


def call_set_parameters(
    *,
    node: rclpy.node.Node,
    node_name: typing.Optional[str] = None,
    client: typing.Optional[rclpy.node.Client] = None,
    parameters: typing.List[Parameter] = []
) -> typing.Dict[str, SetParametersResult]:
    lg = node.get_logger()

    if client is None:
        assert node_name is not None
        client = node.create_client(SetParameters,
                                    f"{node_name}/set_parameters")

    ready = client.wait_for_service(timeout_sec=TIMEOUT)
    if not ready:
        lg.error("wait for service time out")
        raise ServiceWaitTimeout()

    req = SetParameters.Request()
    req.parameters = [p.to_parameter_msg() for p in parameters]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    resp = future.result()
    lg.debug(f"set result: {resp}")

    return dict(zip((p.name for p in parameters), resp.results))


def call_set_parameters_check_and_raise(**kwargs):
    results = call_set_parameters(**kwargs)
    msg = ';'.join(f"{k}: {r.reason}" for k, r in results.items()
                   if not r.successful)
    if msg:
        raise ValueError(msg)


def parameter_from_parameter_value(
        name: str, parameter_value: ParameterValue) -> Parameter:
    pmsg = ParameterMsg(name=name, value=parameter_value)
    return Parameter.from_parameter_msg(pmsg)
