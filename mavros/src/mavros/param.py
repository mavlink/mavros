# -*- python -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import csv
import time
import rospy

from mavros.srv import ParamPull, ParamPush, ParamGet, ParamSet


class Parameter(object):
    """Class representing one parameter"""
    def __init__(self, param_id, param_value=0):
        self.param_id = param_id
        self.param_value = param_value

    def __repr__(self):
        return "<Parameter '{}': {}>".format(self.param_id, self.param_value)


class ParamFile(object):
    """Base class for param file parsers"""
    def __init__(self, args):
        pass

    def read(self, file_):
        """Returns a iterable of Parameters"""
        raise NotImplementedError

    def write(self, file_, parametes):
        """Writes Parameters to file"""
        raise NotImplementedError


class MissionPlannerParam(ParamFile):
    """Parse MissionPlanner param files"""

    class CSVDialect(csv.Dialect):
        delimiter = ','
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE

    def read(self, file_):
        to_numeric = lambda x: float(x) if '.' in x else int(x)

        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue # skip comments

            if len(data) != 2:
                raise ValueError("wrong field count")

            yield Parameter(data[0].strip(), to_numeric(data[1]));

    def write(self, file_, parameters):
        writer = csv.writer(file_, self.CSVDialect)
        writer.writerow(("#NOTE: " + time.strftime("%d.%m.%Y %T") ,))
        for p in parameters:
            writer.writerow((p.param_id, p.param_value))


class QGroundControlParam(ParamFile):
    """Parse QGC param files"""

    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\n'
        quoting = csv.QUOTE_NONE

    def __init__(self, args):
        self.mavros_ns = args.mavros_ns

    def read(self, file_):
        to_numeric = lambda x: float(x) if '.' in x else int(x)

        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue # skip comments

            if len(data) != 5:
                raise ValueError("wrong field count")

            yield Parameter(data[2].strip(), to_numeric(data[3]));

    def write(self, file_, parameters):
        def to_type(x):
            if isinstance(x, float):
                return 9 # REAL32
            elif isinstance(x, int):
                return 6 # INT32
            else:
                raise ValueError("unknown type: " + repr(type(x)))

        sysid = rospy.get_param(self.mavros_ns + "/target_system_id", 1)
        compid = rospy.get_param(self.mavros_ns + "/target_component_id", 1)

        writer = csv.writer(file_, self.CSVDialect)
        writer.writerow(("# NOTE: " + time.strftime("%d.%m.%Y %T"), ))
        writer.writerow(("# Onboard parameters saved by mavparam for ({}, {})".format(sysid, compid), ))
        writer.writerow(("# MAV ID" , "COMPONENT ID", "PARAM NAME", "VALUE", "(TYPE)"))
        for p in parameters:
            writer.writerow((sysid, compid, p.param_id, p.param_value, to_type(p.param_value), )) # XXX


def param_ret_value(ret):
    if ret.integer != 0:
        return ret.integer
    elif ret.real != 0.0:
        return ret.real
    else:
        return 0


def param_get(param_id, ns="/mavros"):
    try:
        get_cl = rospy.ServiceProxy(ns + "/param/get", ParamGet)
        ret = get_cl(param_id=param_id)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    if not ret.success:
        raise IOError("Request failed.")

    return param_ret_value(ret)


def param_set(param_id, value, ns="/mavros"):
    if isinstance(value, float):
        val_f = value
        val_i = 0
    else:
        val_f = 0.0
        val_i = value

    try:
        set_cl = rospy.ServiceProxy(ns + "/param/set", ParamSet)
        ret = set_cl(param_id=param_id,
                     integer=val_i,
                     real=val_f
                     )
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    if not ret.success:
        raise IOError("Request failed.")

    return param_ret_value(ret)


def param_get_all(force_pull=False, ns="/mavros"):
    try:
        pull_cl = rospy.ServiceProxy(ns + "/param/pull", ParamPull)
        ret = pull_cl(force_pull=force_pull)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    if not ret.success:
        raise IOError("Request failed.")

    params = rospy.get_param(ns + "/param")

    return (ret.param_received,
            sorted((Parameter(k, v) for k, v in params.iteritems()),
                   cmp=lambda x, y: cmp(x.param_id, y.param_id))
            )

def param_set_list(param_list, ns="/mavros"):
    # 1. load parameters to parameter server
    for p in param_list:
        rospy.set_param(ns + "/param/" + p.param_id, p.param_value)

    # 2. request push all
    try:
        push_cl = rospy.ServiceProxy(ns + "/param/push", ParamPush)
        ret = push_cl()
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    if not ret.success:
        raise IOError("Request failed.")

    return ret.param_transfered
