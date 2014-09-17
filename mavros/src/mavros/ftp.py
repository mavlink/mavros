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

import os
import rospy
from std_srvs.srv import Empty
from mavros.msg import FileEntry
from mavros.srv import FileOpen, FileClose, FileRead, FileList, FileOpenRequest, \
    FileMakeDir, FileRemoveDir, FileRemove, FileWrite


class FTPFile(object):
    def __init__(self, name, mode, ns="/mavros"):
        self.name = None
        self.mode = mode
        self.mavros_ns = ns
        self.open(name, mode)

    def __del__(self):
        self.close()

    def _check_raise_errno(self, ret):
        if not ret.success:
            raise IOError(os.strerror(ret.r_errno))

    def open(self, path, mode):
        """
        Supported modes:
            - 'w': write binary
            - 'r': read binary
            - 'cw': create excl & write
        """
        if mode == 'w' or mode == 'wb':
            m = FileOpenRequest.MODE_WRITE
        elif mode == 'r' or mode == 'rb':
            m = FileOpenRequest.MODE_READ
        elif mode == 'cw':
            m = FileOpenRequest.MODE_CREATE
        else:
            raise ValueError("Unknown open mode: {}".format(m))

        try:
            open_cl = rospy.ServiceProxy(self.mavros_ns + "/ftp/open", FileOpen)
            ret = open_cl(file_path=path, mode=m)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        self._check_raise_errno(ret)

        self._read_cl = rospy.ServiceProxy(self.mavros_ns + "/ftp/read", FileRead)
        self._write_cl = rospy.ServiceProxy(self.mavros_ns + "/ftp/write", FileWrite)

        self.name = path
        self.mode = mode
        self.size = ret.size
        self.offset = 0

    def close(self):
        if self.closed:
            return

        try:
            close_cl = rospy.ServiceProxy(self.mavros_ns + "/ftp/close", FileClose)
            ret = close_cl(file_path=self.name)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        self.name = None
        self._check_raise_errno(ret)

    def read(self, size=1):
        try:
            ret = self._read_cl(file_path=self.name, offset=self.offset, size=size)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        self._check_raise_errno(ret)
        self.offset += len(ret.data)
        return bytearray(ret.data)

    def write(self, bin_data):
        data_len = len(bin_data)
        try:
            ret = self._write_cl(file_path=self.name, offset=self.offset, data=bin_data)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        self._check_raise_errno(ret)
        self.offset += data_len
        if self.offset > self.size:
            self.size = self.offset

    def tell(self):
        return self.offset

    def seek(self, offset, whence=os.SEEK_SET):
        if whence is os.SEEK_SET:
            self.offset = offset
        elif whence is os.SEEK_END:
            self.offset = offset + self.size
        elif whence is os.SEEK_CUR:
            self.offset += offset
        else:
            raise ValueError("Unknown whence")

    def truncate(self, size=0):
        raise NotImplementedError

    @property
    def closed(self):
        return self.name is None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
