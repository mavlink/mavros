# -*- coding: utf-8 -*-

import pathlib

import pytest

from mavros.mission import QGroundControlWPL

# from mavros.mission import QGroundControlPlan


@pytest.mark.parametrize(
    "file_class,file_name,expected_mission_len,expected_fence_len,expected_rally_len",
    [
        (
            QGroundControlWPL,
            "CMAC-circuit.txt",
            8,
            0,
            0,
        ),
        # ( # XXX TODO(vooon): implement me!
        #     QGroundControlPlan,
        #     "simple.plan",
        #     0,
        #     0,
        #     0,
        # ),
    ],
)
def test_PlanFile_load(
    file_class, file_name, expected_mission_len, expected_fence_len, expected_rally_len
):
    file_path = pathlib.Path(__file__).parent / "testdata" / file_name

    with file_path.open() as file_:
        pf = file_class().load(file_)

    assert expected_mission_len == len(pf.mission or [])
    assert expected_fence_len == len(pf.fence or [])
    assert expected_rally_len == len(pf.rally or [])
