# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import functools
import typing

import rclpy
import rclpy.clock
import rclpy.time
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter

from .base import ServiceWaitTimeout

TIMEOUT = 5.0


@functools.cache
def system_clock() -> rclpy.clock.Clock:
    return rclpy.clock.Clock()


def system_now() -> rclpy.time.Time:
    return system_clock().now()


def wait_for_service(client: rclpy.node.Client,
                     lg: typing.Optional[typing.Any]):
    ready = client.wait_for_service(timeout_sec=TIMEOUT)
    if not ready:
        if lg:
            lg.error("wait for service time out")
        raise ServiceWaitTimeout()


def parameter_from_parameter_value(
        name: str, parameter_value: ParameterValue) -> Parameter:
    pmsg = ParameterMsg(name=name, value=parameter_value)
    return Parameter.from_parameter_msg(pmsg)


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

    wait_for_service(client, lg)

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

    wait_for_service(client, lg)

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

    wait_for_service(client, lg)

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
