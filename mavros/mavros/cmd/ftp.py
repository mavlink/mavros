# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2014,2021 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
"""mav ftp command."""

import os
import pathlib
import typing

import click

from mavros_msgs.msg import FileEntry

from ..nuttx_crc32 import nuttx_crc32
from . import cli, pass_client
from .utils import fault_echo

# optimized transfer size for FTP message payload
# XXX: bug in ftp.cpp cause a doubling request of last package.
# -1 fixes that.
FTP_PAGE_SIZE = 239 * 18 - 1
FTP_PWD_FILE = pathlib.Path("/tmp/.mavftp_pwd")


class ProgressBar:
    """Wrapper class for hiding file transfer brogressbar construction."""

    def __init__(self, quiet: bool, label: str, maxval: int):
        if quiet or maxval == 0:
            if maxval == 0:
                click.echo("Can't show progressbar for unknown file size", err=True)
            self.pbar = None
            return

        self.pbar = click.progressbar(
            label=label,
            length=maxval,
            show_percent=True,
            show_eta=True,
        )

    def update(self, value: int):
        if self.pbar:
            self.pbar.update(value)

    def __enter__(self):
        if self.pbar:
            self.pbar.__enter__()

        return self

    def __exit__(self, type, value, traceback):
        if self.pbar:
            self.pbar.__exit__(type, value, traceback)


@cli.group()
@pass_client
def ftp(client):
    """File manipulation tool for MAVLink-FTP."""


def resolve_path(path: typing.Union[None, str, pathlib.Path] = None) -> pathlib.Path:
    """Resolve FTP path using PWD file."""
    if FTP_PWD_FILE.exists():
        with FTP_PWD_FILE.open("r") as fd:
            pwd = fd.readline()
    else:
        # default home location is root directory
        pwd = os.environ.get("MAVFTP_HOME", "/")

    pwd = pathlib.Path(pwd)

    if not path:
        return pwd.resolve()  # no path - PWD location
    elif path.startswith("/"):
        return pathlib.Path(path).resolve()  # absolute path
    else:
        return (pwd / path).resolve()


@ftp.command("cd")
@click.argument("path", type=click.Path(exists=False), nargs=1, required=False)
@pass_client
@click.pass_context
def change_directory(ctx, client, path):
    """Change directory."""
    if path:
        path = resolve_path(path)
    if path and not path.is_absolute():
        fault_echo(ctx, f"Path is not absolute: {path}")

    if path:
        with FTP_PWD_FILE.open("w") as fd:
            fd.write(str(pathlib.Path(path).resolve()))
    else:
        if FTP_PWD_FILE.exists():
            FTP_PWD_FILE.unlink()


@ftp.command("ls")
@click.argument("path", type=click.Path(exists=False), nargs=1, required=False)
@pass_client
@click.pass_context
def list(ctx, client, path):
    """List files and directories."""
    path = resolve_path(path)
    for ent in client.ftp.listdir(str(path)):
        isdir = ent.type == FileEntry.TYPE_DIRECTORY
        click.echo(f"{ent.name}{isdir and '/' or ''}\t{ent.size}")


@ftp.command()
@click.argument("path", type=click.Path(exists=False), nargs=1, required=False)
@pass_client
@click.pass_context
def cat(ctx, client, path):
    """Cat file from FCU."""
    ctx.invoke(
        download,
        src=path,
        dest=click.open_file("-", "wb"),
        progressbar=True,
        verify=False,
    )


@ftp.command("rm")
@click.argument(
    "path", type=click.Path(exists=False, file_okay=True), nargs=1, required=True
)
@pass_client
@click.pass_context
def remove(ctx, client, path):
    """Remove file."""
    path = resolve_path(path)
    client.ftp.unlink(str(path))


@ftp.command()
@pass_client
def reset(client):
    """Reset ftp server."""
    client.ftp.reset_server()


@ftp.command()
@click.argument(
    "path", type=click.Path(exists=False, dir_okay=True), nargs=1, required=True
)
@pass_client
@click.pass_context
def mkdir(ctx, client, path):
    """Create directory."""
    path = resolve_path(path)
    client.ftp.mkdir(str(path))


@ftp.command()
@click.argument(
    "path", type=click.Path(exists=False, dir_okay=True), nargs=1, required=True
)
@pass_client
@click.pass_context
def rmdir(ctx, client, path):
    """Remove directory."""
    path = resolve_path(path)
    client.ftp.rmdir(str(path))


@ftp.command()
@click.option(
    "--progressbar/--no-progressbar", " /-q", default=True, help="show progress bar"
)
@click.option("--verify/--no-verify", " /-v", default=True, help="perform verify step")
@click.argument(
    "src", type=click.Path(exists=False, file_okay=True), nargs=1, required=True
)
@click.argument("dest", type=click.File("wb"), required=False)
@pass_client
@click.pass_context
def download(ctx, client, src, dest, progressbar, verify):
    """Download file."""
    local_crc = 0
    src = resolve_path(src)

    if not dest:
        # if file argument is not set, use $PWD/basename
        dest = click.open_file(pathlib.Path(src.name).name, "wb")

    client.verbose_echo(f"Downloading from {src} to {dest.name}", err=True)

    with dest as to_fd, click.ftp.open(src, "r") as from_fd, ProgressBar(
        not progressbar, "Downloading:", from_fd.size
    ) as bar:
        while True:
            buf = from_fd.read(FTP_PAGE_SIZE)
            if len(buf) == 0:
                break

            local_crc = nuttx_crc32(buf, local_crc)
            to_fd.write(buf)
            bar.update(from_fd.tell())

    if verify:
        click.verbose_echo("Verifying...", err=True)
        remote_crc = click.ftp.checksum(str(src))
        if local_crc != remote_crc:
            fault_echo(f"Verification failed: 0x{local_crc:08x} != 0x{remote_crc:08x}")


@ftp.command()
@click.option(
    "--progressbar/--no-progressbar", " /-q", default=True, help="show progress bar"
)
@click.option("--verify/--no-verify", " /-v", default=True, help="perform verify step")
@click.option(
    "--overwrite/--no-overwrite",
    " /-W",
    default=True,
    help="is it allowed to overwrite file",
)
@click.argument("src", type=click.File("rb"), nargs=1, required=True)
@click.argument("dest", type=click.Path(exists=False, file_okay=True), required=False)
@pass_client
@click.pass_context
def upload(ctx, client, src, dest, progressbar, verify, overwrite):
    """Upload file."""
    mode = "cw" if not overwrite else "w"
    local_crc = 0

    if dest:
        dest = resolve_path(dest)
    else:
        dest = resolve_path(pathlib.Path(src.name).name)

    client.verbose_echo(f"Uploading from {src} to {dest}", err=True)

    # for stdin it is 0
    from_size = os.fstat(src.fileno()).st_size

    with src as from_fd, client.ftp.open(str(dest), mode) as to_fd, ProgressBar(
        not progressbar, "Uploading:", from_size
    ) as bar:
        while True:
            buf = from_fd.read(FTP_PAGE_SIZE)
            if len(buf) == 0:
                break

            local_crc = nuttx_crc32(buf, local_crc)
            to_fd.write(buf)
            bar.update(to_fd.tell())

    if verify:
        client.verbose_echo("Verifying...", err=True)
        remote_crc = client.ftp.checksum(str(dest))
        if local_crc != remote_crc:
            fault_echo(f"Verification failed: 0x{local_crc:08x} != 0x{remote_crc:08x}")


@ftp.command()
@click.argument("local", type=click.File("rb"), nargs=1, required=True)
@click.argument("remote", type=click.Path(exists=False, file_okay=True), required=False)
@pass_client
@click.pass_context
def verify(ctx, client, local, remote):
    """Verify files."""
    local_crc = 0

    if remote:
        remote = resolve_path(remote)
    else:
        remote = resolve_path(pathlib.Path(local.name).name)

    client.verbose_echo(f"Verifying {local} and {remote}", err=True)

    with local as fd:
        while True:
            buf = fd.read(4096 * 32)  # use 128k block for CRC32 calculation
            if len(buf) == 0:
                break

            local_crc = nuttx_crc32(buf, local_crc)

    remote_crc = client.ftp.checksum(str(remote))

    client.verbose_echo("CRC32 for local and remote files:")
    client.verbose_echo(f"0x{local_crc:08x}  {local.name}")
    client.verbose_echo(f"0x{remote_crc:08x}  {remote.name}")

    if local_crc != remote_crc:
        fault_echo(ctx, f"{local.name}: FAULT")
    else:
        click.echo(f"{local.name}: OK")
