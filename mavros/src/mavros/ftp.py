# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2015 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

__all__ = (
    'FTPFile',
    'open',
    'listdir',
    'unlink',
    'mkdir',
    'rmdir',
    'rename',
    'checksum',
    'reset_server'
)

import os
import rospy
import mavros

from std_srvs.srv import Empty
from mavros_msgs.msg import FileEntry
from mavros_msgs.srv import FileOpen, FileClose, FileRead, FileList, FileOpenRequest, \
    FileMakeDir, FileRemoveDir, FileRemove, FileWrite, FileTruncate, FileRename, \
    FileChecksum


def _get_proxy(service, type):
    return rospy.ServiceProxy(mavros.get_topic('ftp', service), type)


def _check_raise_errno(ret):
        if not ret.success:
            raise IOError(ret.r_errno, os.strerror(ret.r_errno))


class FTPFile(object):
    """
    FCU file object.
    Note that current PX4 firmware only support two connections simultaneously.
    """

    def __init__(self, name, mode):
        self.name = None
        self.mode = mode
        self.open(name, mode)

    def __del__(self):
        self.close()

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

        open_ = _get_proxy('open', FileOpen)
        try:
            ret = open_(file_path=path, mode=m)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        _check_raise_errno(ret)

        self._read = _get_proxy('read', FileRead)
        self._write = _get_proxy('write', FileWrite)

        self.name = path
        self.mode = mode
        self.size = ret.size
        self.offset = 0

    def close(self):
        if self.closed:
            return

        close_ = _get_proxy('close', FileClose)
        try:
            ret = close_(file_path=self.name)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        self.name = None
        _check_raise_errno(ret)

    def read(self, size=1):
        try:
            ret = self._read(file_path=self.name, offset=self.offset, size=size)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        _check_raise_errno(ret)
        self.offset += len(ret.data)
        return bytearray(ret.data)

    def write(self, bin_data):
        data_len = len(bin_data)
        try:
            ret = self._write(file_path=self.name, offset=self.offset, data=bin_data)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        _check_raise_errno(ret)
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
        truncate_ = _get_proxy('truncate', FileTruncate)
        try:
            ret = truncate_(file_path=self.name, length=size)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        _check_raise_errno(ret)

    @property
    def closed(self):
        return self.name is None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def open(path, mode):
    """Open file on FCU"""
    return FTPFile(path, mode)


def listdir(path):
    """List directory :path: contents"""
    try:
        list_ = _get_proxy('list', FileList)
        ret = list_(dir_path=path)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    _check_raise_errno(ret)
    return ret.list


def unlink(path):
    """Remove :path: file"""
    remove = _get_proxy('remove', FileRemove)
    try:
        ret = remove(file_path=path)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    _check_raise_errno(ret)


def mkdir(path):
    """Create directory :path:"""
    mkdir_ = _get_proxy('mkdir', FileMakeDir)
    try:
        ret = mkdir_(dir_path=path)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    _check_raise_errno(ret)


def rmdir(path):
    """Remove directory :path:"""
    rmdir_ = _get_proxy('rmdir', FileRemoveDir)
    try:
        ret = rmdir_(dir_path=path)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    _check_raise_errno(ret)


def rename(old_path, new_path):
    """Rename :old_path: to :new_path:"""
    rename_ = _get_proxy('rename', FileRename)
    try:
        ret = rename_(old_path=old_path, new_path=new_path)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    _check_raise_errno(ret)


def checksum(path):
    """Calculate CRC32 for :path:"""
    checksum_ = _get_proxy('checksum', FileChecksum)
    try:
        ret = checksum_(file_path=path)
    except rospy.ServiceException as ex:
        raise IOError(str(ex))

    _check_raise_errno(ret)
    return ret.crc32


def reset_server():
    reset = _get_proxy('reset', Empty)
    try:
        reset()
    except rospy.ServiceException as ex:
        raise IOError(str(ex))
