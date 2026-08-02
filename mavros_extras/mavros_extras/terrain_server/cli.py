#!/usr/bin/env python3
"""
CLI entry-point for the terrain tile server.

Usage::

    ros2 run mavros_extras terrain_server                       # run ROS node
    ros2 run mavros_extras terrain_server node                  # explicit
    ros2 run mavros_extras terrain_server preload LAT1 LON1 LAT2 LON2
    ros2 run mavros_extras terrain_server update
    ros2 run mavros_extras terrain_server validate
"""

from __future__ import annotations

import logging
import sys

import click

from .srtm import SRTM1_SIZE, SRTM3_SIZE, SrtmManager


def _make_manager(
    terrain_data_path: str,
    srtm_data_url: str,
    srtm_source: str,
    max_cache_tiles: int,
    proxy: str,
) -> SrtmManager:
    """Create a SrtmManager from CLI options (for cache subcommands)."""
    return SrtmManager(
        terrain_data_path=terrain_data_path,
        auto_download=True,
        srtm_data_url=srtm_data_url,
        srtm_source=srtm_source,
        max_cache_tiles=max_cache_tiles,
        proxy=proxy,
        download_workers=4,
    )


@click.group(
    invoke_without_command=True,
    context_settings={'ignore_unknown_options': True, 'allow_extra_args': True},
)
@click.pass_context
def cli(ctx: click.Context) -> None:
    """Terrain tile server and cache management (MAVLink TERRAIN protocol)."""
    if ctx.invoked_subcommand is None:
        # Pass through any extra args (e.g. --ros-args -p key:=val) to the node
        from .node import main as node_main

        node_main(args=ctx.args)


@cli.command()
def node() -> None:
    """Run the ROS 2 terrain server node."""
    from .node import main as node_main

    node_main()


@cli.command()
@click.argument('lat1', type=int)
@click.argument('lon1', type=int)
@click.argument('lat2', type=int)
@click.argument('lon2', type=int)
@click.option('--terrain-data-path', default='', help='Directory for .hgt tile cache.')
@click.option(
    '--srtm-data-url',
    default='https://terrain.ardupilot.org',
    help='SRTM tile download base URL (http:// for Squid cacheability).',
)
@click.option('--srtm-source', default='SRTM3', help='SRTM dataset name.')
@click.option('--max-cache-tiles', type=int, default=64, help='LRU cache size.')
@click.option('--proxy', default='', help='HTTP/HTTPS proxy (host:port).')
def preload(
    lat1: int,
    lon1: int,
    lat2: int,
    lon2: int,
    terrain_data_path: str,
    srtm_data_url: str,
    srtm_source: str,
    max_cache_tiles: int,
    proxy: str,
) -> None:
    """Download all SRTM tiles in a bounding box."""
    mgr = _make_manager(terrain_data_path, srtm_data_url, srtm_source, max_cache_tiles, proxy)
    la1, la2 = sorted((lat1, lat2))
    lo1, lo2 = sorted((lon1, lon2))

    tiles_total = (la2 - la1 + 1) * (lo2 - lo1 + 1)
    click.echo(f'Preloading {tiles_total} tiles for bbox lat=[{la1}, {la2}] lon=[{lo1}, {lo2}]')

    downloaded = 0
    skipped = 0
    failed = 0
    for lat in range(la1, la2 + 1):
        for lon in range(lo1, lo2 + 1):
            if mgr.is_tile_available(lat, lon):
                skipped += 1
                continue
            if mgr.preload_tile(lat, lon):
                downloaded += 1
                click.echo(f'  OK: ({lat:+03d}, {lon:+04d})')
            else:
                failed += 1
                click.echo(f'  FAIL: ({lat:+03d}, {lon:+04d})', err=True)

    click.echo(f'\nDone: {downloaded} downloaded, {skipped} already cached, {failed} failed')


@cli.command()
@click.option('--terrain-data-path', default='', help='Directory for .hgt tile cache.')
@click.option(
    '--srtm-data-url',
    default='https://terrain.ardupilot.org',
    help='SRTM tile download base URL (http:// for Squid cacheability).',
)
@click.option('--srtm-source', default='SRTM3', help='SRTM dataset name.')
@click.option('--max-cache-tiles', type=int, default=64, help='LRU cache size.')
@click.option('--proxy', default='', help='HTTP/HTTPS proxy (host:port).')
def update(
    terrain_data_path: str,
    srtm_data_url: str,
    srtm_source: str,
    max_cache_tiles: int,
    proxy: str,
) -> None:
    """Re-download missing or corrupted tiles."""
    import os
    from pathlib import Path

    mgr = _make_manager(terrain_data_path, srtm_data_url, srtm_source, max_cache_tiles, proxy)

    data_path = terrain_data_path
    if not data_path:
        home = Path(os.environ.get('HOME', '/tmp'))
        data_path = str(home / '.cache' / 'mavros' / 'terrain' / srtm_source)

    root = Path(data_path)
    if not root.exists():
        click.echo(f'No terrain data directory: {data_path}', err=True)
        return

    hgt_files = list(root.rglob('*.hgt'))
    click.echo(f'Checking {len(hgt_files)} tiles in {data_path}')

    invalid = 0
    re_downloaded = 0
    for hgt in hgt_files:
        size = hgt.stat().st_size
        if size != SRTM1_SIZE and size != SRTM3_SIZE:
            invalid += 1
            click.echo(
                f'  INVALID: {hgt.name} ({size} bytes), re-downloading...',
                err=True,
            )
            hgt.unlink()
            name = hgt.name
            lat = int(name[1:3]) * (-1 if name[0] == 'S' else 1)
            lon = int(name[4:7]) * (-1 if name[3] == 'W' else 1)
            if mgr.preload_tile(lat, lon):
                re_downloaded += 1

    click.echo(f'\nDone: {invalid} invalid, {re_downloaded} re-downloaded')


@cli.command()
@click.option('--terrain-data-path', default='', help='Directory for .hgt tile cache.')
@click.option('--srtm-source', default='SRTM3', help='SRTM dataset name.')
def validate(terrain_data_path: str, srtm_source: str) -> None:
    """Verify all cached tiles have correct sizes."""
    import os
    from pathlib import Path

    data_path = terrain_data_path
    if not data_path:
        home = Path(os.environ.get('HOME', '/tmp'))
        data_path = str(home / '.cache' / 'mavros' / 'terrain' / srtm_source)

    root = Path(data_path)
    if not root.exists():
        click.echo(f'No terrain data directory: {data_path}', err=True)
        return

    hgt_files = list(root.rglob('*.hgt'))
    click.echo(f'Validating {len(hgt_files)} tiles in {data_path}')

    valid = 0
    invalid = 0
    for hgt in hgt_files:
        size = hgt.stat().st_size
        if size == SRTM1_SIZE or size == SRTM3_SIZE:
            valid += 1
        else:
            invalid += 1
            click.echo(f'  INVALID: {hgt.name} ({size} bytes)', err=True)

    click.echo(f'\nResult: {valid} valid, {invalid} invalid')
    if invalid > 0:
        click.echo('Run "terrain_server update" to fix invalid tiles.')
        sys.exit(1)


def main(args=None) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )

    if args is None:
        args = sys.argv[1:]

    # If no subcommand or starts with ROS args, run the node directly
    # (bypass click so rclpy can parse --ros-args -p key:=val)
    known_commands = {'node', 'preload', 'update', 'validate', '--help', '-h'}
    if not args or args[0] not in known_commands:
        from .node import main as node_main

        node_main(args=args)
        return

    cli(args=args, standalone_mode=True)


if __name__ == '__main__':
    main()
