# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2015,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import os
import typing

import rclpy
from std_srvs.srv import Empty

from mavros_msgs.msg import FileEntry
from mavros_msgs.srv import (
    FileChecksum,
    FileClose,
    FileList,
    FileMakeDir,
    FileOpen,
    FileRead,
    FileRemove,
    FileRemoveDir,
    FileRename,
    FileTruncate,
    FileWrite,
)

from .base import PluginModule, cached_property


def _check_raise_errno(ret):
    if not ret.success:
        raise IOError(ret.r_errno, os.strerror(ret.r_errno))


class FTPFile:
    """
    FCU file object.

    Note that current PX4 firmware only support two connections simultaneously.
    """

    _fm: "FTPPlugin"

    def __init__(self, *, fm, name, mode):
        self._fm = fm
        self.name = None
        self.mode = mode
        self.open(name, mode)

    def __del__(self):
        self.close()

    def open(self, path: str, mode: str):
        """
        Call open.

        Supported modes:
            - 'w': write binary
            - 'r': read binary
            - 'cw': create excl & write
        """
        if mode == "w" or mode == "wb":
            m = FileOpen.Request.MODE_WRITE
        elif mode == "r" or mode == "rb":
            m = FileOpen.Request.MODE_READ
        elif mode == "cw":
            m = FileOpen.Request.MODE_CREATE
        else:
            raise ValueError("Unknown open mode: {}".format(m))

        req = FileOpen.Request(
            file_path=path,
            mode=m,
        )

        ret = self._fm.cli_open.call(req)
        _check_raise_errno(ret)

        self.name = path
        self.mode = mode
        self.size = ret.size
        self.offset = 0

    def close(self):
        if self.closed:
            return

        req = FileClose.Request(file_path=self.name)
        ret = self._fm.cli_close(req)
        self.name = None
        _check_raise_errno(ret)

    def read(self, size: int = 1) -> bytearray:
        req = FileRead.Request(file_path=self.name, offset=self.offset, size=size)
        ret = self._fm.cli_read.call(req)
        _check_raise_errno(ret)
        self.offset += len(ret.data)
        return bytearray(ret.data)

    def write(self, bin_data: typing.Union[bytes, bytearray]):
        data_len = len(bin_data)

        req = FileWrite.Request(file_path=self.name, offset=self.offset, data=bin_data)
        ret = self._fm.cli_write.call(req)
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

    def truncate(self, size: int = 0):
        req = FileTruncate.Request(file_path=self.name, length=size)
        ret = self._fm.cli_truncate.call(req)
        _check_raise_errno(ret)

    @property
    def closed(self):
        return self.name is None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class FTPPlugin(PluginModule):
    """FTP plugin interface."""

    @cached_property
    def cli_open(self) -> rclpy.node.Client:
        return self.create_client(FileOpen, ("ftp", "open"))

    @cached_property
    def cli_close(self) -> rclpy.node.Client:
        return self.create_client(FileClose, ("ftp", "close"))

    @cached_property
    def cli_read(self) -> rclpy.node.Client:
        return self.create_client(FileRead, ("ftp", "read"))

    @cached_property
    def cli_write(self) -> rclpy.node.Client:
        return self.create_client(FileWrite, ("ftp", "write"))

    @cached_property
    def cli_truncate(self) -> rclpy.node.Client:
        return self.create_client(FileTruncate, ("ftp", "truncate"))

    @cached_property
    def cli_listdir(self) -> rclpy.node.Client:
        return self.create_client(FileList, ("ftp", "list"))

    @cached_property
    def cli_unlink(self) -> rclpy.node.Client:
        return self.create_client(FileRemove, ("ftp", "remove"))

    @cached_property
    def cli_mkdir(self) -> rclpy.node.Client:
        return self.create_client(FileMakeDir, ("ftp", "mkdir"))

    @cached_property
    def cli_rmdir(self) -> rclpy.node.Client:
        return self.create_client(FileRemoveDir, ("ftp", "rmdir"))

    @cached_property
    def cli_rename(self) -> rclpy.node.Client:
        return self.create_client(FileRename, ("ftp", "rename"))

    @cached_property
    def cli_checksum(self) -> rclpy.node.Client:
        return self.create_client(FileChecksum, ("ftp", "checksum"))

    @cached_property
    def cli_reset(self) -> rclpy.node.Client:
        return self.create_client(Empty, ("ftp", "reset"))

    def open(self, path: str, mode: str = "r") -> FTPFile:
        return FTPFile(fm=self, name=path, mode=mode)

    def listdir(self, dir_path: str) -> typing.List[FileEntry]:
        req = FileList.Request(dir_path=dir_path)
        ret = self.cli_listdir.call(req)
        _check_raise_errno(ret)
        return ret.list

    def unlink(self, path: str):
        req = FileRemove.Request(file_path=path)
        ret = self.cli_unlink.call(req)
        _check_raise_errno(ret)

    def mkdir(self, path: str):
        req = FileMakeDir.Request(dir_path=path)
        ret = self.cli_mkdir.call(req)
        _check_raise_errno(ret)

    def rmdir(self, path: str):
        req = FileRemoveDir.Request(dir_path=path)
        ret = self.cli_rmdir.call(req)
        _check_raise_errno(ret)

    def rename(self, old_path: str, new_path: str):
        req = FileRename.Request(old_path=old_path, new_path=new_path)
        ret = self.cli_rename.call(req)
        _check_raise_errno(ret)

    def checksum(self, path: str) -> int:
        req = FileChecksum.Request(file_path=path)
        ret = self.cli_checksum.call(req)
        _check_raise_errno(ret)
        return ret.crc32

    def reset_server(
        self,
    ):
        req = Empty.Request()
        self.cli_reset.call(req)
