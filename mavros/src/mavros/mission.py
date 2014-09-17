# -*- python -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

import csv
import time

from mavros.msg import Waypoint, WaypointList
from mavros.srv import WaypointPull, WaypointPush, WaypointClear, \
    WaypointSetCurrent, WaypointGOTO


FRAMES = {
    Waypoint.FRAME_GLOBAL: 'GAA',
    Waypoint.FRAME_GLOBAL_REL_ALT: 'GRA',
    Waypoint.FRAME_LOCAL_ENU: 'LOC-ENU',
    Waypoint.FRAME_LOCAL_NED: 'LOC-NED',
    Waypoint.FRAME_MISSION: 'MIS'
}

NAV_CMDS = {
    Waypoint.NAV_LAND: 'LAND',
    Waypoint.NAV_LOITER_TIME: 'LOITER-TIME',
    Waypoint.NAV_LOITER_TURNS: 'LOITER-TURNS',
    Waypoint.NAV_LOITER_UNLIM: 'LOITER-UNLIM',
    Waypoint.NAV_RETURN_TO_LAUNCH: 'RTL',
    Waypoint.NAV_TAKEOFF: 'TAKEOFF',
    Waypoint.NAV_WAYPOINT: 'WAYPOINT',
    # Maybe later i will add this enum to message
    112: 'COND-DELAY',
    113: 'COND-CHANGE-ALT',
    114: 'COND-DISTANCE',
    115: 'COND-YAW',
    177: 'DO-JUMP',
    178: 'DO-CHANGE-SPEED',
    181: 'DO-SET-RELAY',
    182: 'DO-REPEAT-RELAY',
    183: 'DO-SET-SERVO',
    184: 'DO-REPEAT-SERVO',
    201: 'DO-SET-ROI',
}


class WaypointFile(object):
    """Base class for waypoint file parsers"""
    def read(self, file_):
        """Returns a iterable of waypoints"""
        raise NotImplementedError

    def write(self, file_, waypoints):
        """Writes waypoints to file"""
        raise NotImplementedError


class QGroundControlWP(WaypointFile):
    """Parse QGC waypoint file"""

    file_header = 'QGC WPL 120'
    known_versions = (110, 120)

    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE

    def read(self, file_):
        got_header = False
        for data in csv.reader(file_, self.CSVDialect):
            if data[0].startswith('#'):
                continue; # skip comments (i think in next format version they add this)

            if not got_header:
                qgc, wpl, ver = data[0].split(' ', 3)
                ver = int(ver)
                if qgc == 'QGC' and wpl == 'WPL' and ver in self.known_versions:
                    got_header = True

            else:
                yield Waypoint(
                    is_current = bool(int(data[1])),
                    frame = int(data[2]),
                    command = int(data[3]),
                    param1 = float(data[4]),
                    param2 = float(data[5]),
                    param3 = float(data[6]),
                    param4 = float(data[7]),
                    x_lat = float(data[8]),
                    y_long = float(data[9]),
                    z_alt = float(data[10]),
                    autocontinue = bool(int(data[11]))
                )

    def write(self, file_, waypoints):
        writer = csv.writer(file_, self.CSVDialect)
        writer.writerow((self.file_header ,))
        for seq, w in enumerate(waypoints):
            writer.writerow((
                seq,
                int(w.is_current),
                w.frame,
                w.command,
                w.param1,
                w.param2,
                w.param3,
                w.param4,
                w.x_lat,
                w.y_long,
                w.z_alt,
                int(w.autocontinue)
            ))
