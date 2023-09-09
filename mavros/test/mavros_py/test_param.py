# -*- coding: utf-8 -*-

import datetime
import io
import pathlib
from unittest.mock import MagicMock, patch

import pytest

from mavros.param import (
    MavProxyParam,
    MissionPlannerParam,
    ParamDict,
    Parameter,
    QGroundControlParam,
)


def test_ParamDict_get():
    pm = ParamDict()
    pm._pm = MagicMock()

    tv1 = Parameter("TEST1", value=1)
    tv2 = Parameter("TEST2", value=2.0)

    pm.setdefault("TEST1", tv1)
    pm.setdefault("TEST2", tv2.value)

    assert pm["TEST1"].value == 1
    assert pm.TEST2.value == 2.0

    with pytest.raises(KeyError):
        pm["NO_KEY"]

    with pytest.raises(AttributeError):
        pm.NO_ATTR


def test_ParamDict_set():
    pm = ParamDict()
    pm._pm = MagicMock()
    pm._pm._node = MagicMock()
    pm._pm.cli_set_parameters = MagicMock(return_value=None)

    tv1 = Parameter("TEST1", value=1)
    tv2 = Parameter("TEST2", value=2.0)

    with patch("mavros.utils.call_set_parameters", MagicMock(return_value={})) as csp:
        pm["TEST1"] = tv1

        csp.assert_called_once_with(
            node=pm._pm._node, client=pm._pm.cli_set_parameters, parameters=[tv1]
        )

    with patch("mavros.utils.call_set_parameters", MagicMock(return_value={})) as csp:
        pm.TEST2 = tv2

        csp.assert_called_once_with(
            node=pm._pm._node, client=pm._pm.cli_set_parameters, parameters=[tv2]
        )

    with patch("mavros.utils.call_set_parameters", MagicMock(return_value={})) as csp:
        pm.TEST_B = True
        pm.TEST_I = 3
        pm.TEST_F = 4.0

        csp.assert_called()
        assert pm.TEST_B.value is True
        assert pm.TEST_I.value == 3
        assert pm.TEST_F.value == 4.0

    with patch("mavros.utils.call_set_parameters", MagicMock(return_value={})) as csp:
        pm.setdefault("TEST_D", 5)

        csp.assert_not_called()
        assert pm.TEST_D.value == 5


def test_ParamDict_del():
    pm = ParamDict()
    pm._pm = MagicMock()

    pm.setdefault("TEST1", 1)
    pm.setdefault("TEST2", 2.0)

    del pm["TEST1"]
    del pm.TEST2

    with pytest.raises(KeyError):
        del pm["NO_KEY"]

    with pytest.raises(AttributeError):
        del pm.NO_ATTR


@pytest.mark.parametrize(
    "file_class,file_name,expected_len",
    [
        (
            MavProxyParam,
            "mavproxy.parm",
            1296,
        ),
        (
            MissionPlannerParam,
            "missionplanner.parm",
            353,
        ),
        (
            QGroundControlParam,
            "qgroundcontrol.params",
            1296,
        ),
    ],
)
def test_ParamFile_load(file_class, file_name, expected_len):
    file_path = pathlib.Path(__file__).parent / "testdata" / file_name

    with file_path.open() as file_:
        pf = file_class().load(file_)

    assert expected_len == len(pf.parameters)


SAVE_PARAMS = {
    "TEST_I": Parameter("TEST_I", value=100),
    "TEST_F": Parameter("TEST_F", value=1e3),
}

SAVE_STAMP = datetime.datetime.now()


@pytest.mark.parametrize(
    "file_class,expected_output",
    [
        (
            MavProxyParam,
            f"""\
#NOTE: {SAVE_STAMP.strftime("%d.%m.%Y %T")}\r\n\
TEST_I 100\r\n\
TEST_F 1000.0\r\n\
""",
        ),
        (
            MissionPlannerParam,
            f"""\
#NOTE: {SAVE_STAMP.strftime("%d.%m.%Y %T")}\r\n\
TEST_I,100\r\n\
TEST_F,1000.0\r\n\
""",
        ),
        (
            QGroundControlParam,
            f"""\
# NOTE: {SAVE_STAMP.strftime("%d.%m.%Y %T")}\n\
# Onboard parameters saved by mavparam for (2.1)\n\
# MAV ID\tCOMPONENT ID\tPARAM NAME\tVALUE\t(TYPE)\n\
2\t1\tTEST_I\t100\t6\n\
2\t1\tTEST_F\t1000.0\t9\n\
""",
        ),
    ],
)
def test_ParamFile_save(file_class, expected_output):
    pf = file_class()
    pf.parameters = SAVE_PARAMS
    pf.tgt_system = 2

    out = io.StringIO()
    pf.save(out)

    assert expected_output == out.getvalue()
