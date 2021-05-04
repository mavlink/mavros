# -*- coding: utf-8 -*-

from unittest.mock import MagicMock, patch

import pytest

from mavros.param import Parameter, ParamPlugin, ParamDict


def test_ParamDict_get():
    pm = ParamDict()
    pm._pm = MagicMock()

    tv1 = Parameter('TEST1', value=1)
    tv2 = Parameter('TEST2', value=2.0)

    pm.setdefault('TEST1', tv1)
    pm.setdefault('TEST2', tv2.value)

    assert pm['TEST1'].value == 1
    assert pm.TEST2.value == 2.0

    with pytest.raises(KeyError):
        pm['NO_KEY']

    with pytest.raises(AttributeError):
        pm.NO_ATTR


def test_ParamDict_set():
    pm = ParamDict()
    pm._pm = MagicMock()
    pm._pm._node = MagicMock()
    pm._pm.set_parameters = MagicMock(return_value=None)

    tv1 = Parameter('TEST1', value=1)
    tv2 = Parameter('TEST2', value=2.0)

    with patch('mavros.param.call_set_parameters',
               MagicMock(return_value=None)) as csp:
        pm['TEST1'] = tv1

        csp.assert_called_once_with(node=pm._pm._node,
                                    client=pm._pm.set_parameters,
                                    parameters=[tv1])

    with patch('mavros.param.call_set_parameters',
               MagicMock(return_value=None)) as csp:
        pm.TEST2 = tv2

        csp.assert_called_once_with(node=pm._pm._node,
                                    client=pm._pm.set_parameters,
                                    parameters=[tv2])

    with patch('mavros.param.call_set_parameters',
               MagicMock(return_value=None)) as csp:

        pm.TEST_B = True
        pm.TEST_I = 3
        pm.TEST_F = 4.0

        csp.assert_called()
        assert pm.TEST_B.value == True
        assert pm.TEST_I.value == 3
        assert pm.TEST_F.value == 4.0


def test_ParamDict_del():
    pm = ParamDict()
    pm._pm = MagicMock()

    pm.setdefault('TEST1', 1)
    pm.setdefault('TEST2', 2.0)

    del pm['TEST1']
    del pm.TEST2

    with pytest.raises(KeyError):
        del pm['NO_KEY']

    with pytest.raises(AttributeError):
        del pm.NO_ATTR
