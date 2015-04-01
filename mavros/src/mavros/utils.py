# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import os
import sys


def print_if(cond, *args, **kvargs):
    if cond:
        print(*args, **kvargs)


def fault(*args, **kvargs):
    kvargs['file'] = sys.stderr
    print(*args, **kvargs)
    sys.exit(1)
