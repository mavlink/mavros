"""
Manage SRTM tiles: download, cache, parse, and look up elevation.

Handles .hgt files from the Shuttle Radar Topography Mission dataset.
Supports SRTM1 (1 arc-second, 3601x3601) and SRTM3 (3 arc-second, 1201x1201).

Written by Zeke Sarosi <zeke.sarosi@gmail.com>
Inspired by the Pymavlink implementation
"""

from __future__ import annotations

import array
import io
import logging
import math
import os
import struct
import threading
import urllib.error
import urllib.request
import zipfile
from collections import OrderedDict
from pathlib import Path

logger = logging.getLogger(__name__)

SRTM1_SIDE = 3601
SRTM3_SIDE = 1201
SRTM_VOID = -32768

RADIUS_OF_EARTH = 6378100.0

# MAVLink TERRAIN_REQUEST grid layout (common.xml §TERRAIN_REQUEST):
#   The terrain around a position is divided into a GRID_ROWS × GRID_COLS
#   array of blocks. Each block is TILE_DIM × TILE_DIM elevation samples.
GRID_COLS = 8
GRID_ROWS = 7
TILE_DIM = 4

_CONTINENTS = (
    'Africa/',
    'Australia/',
    'Eurasia/',
    'Islands/',
    'North_America/',
    'South_America/',
)

try:
    from mavros_extras.srtm_continent_map import lookup_continent as _lookup_continent
except ImportError:
    _lookup_continent = None


class SrtmTile:
    """Single loaded SRTM .hgt tile with compact int16 storage."""

    __slots__ = ('data', 'side')

    def __init__(self, data: array.array, side: int):
        self.data = data
        self.side = side

    def __repr__(self) -> str:
        return f'SrtmTile(side={self.side})'


class SrtmManager:
    """Thread-safe SRTM tile manager with LRU cache and optional auto-download."""

    def __init__(
        self,
        terrain_data_path: str = '',
        auto_download: bool = False,
        download_host: str = 'terrain.ardupilot.org',
        srtm_source: str = 'SRTM3',
        max_cache_tiles: int = 64,
    ):
        self._terrain_data_path = terrain_data_path
        self._auto_download = auto_download
        self._download_host = download_host
        self._srtm_source = srtm_source
        self._max_cache_tiles = max_cache_tiles

        self._cache: OrderedDict[int, SrtmTile | None] = OrderedDict()
        self._file_index: dict[str, Path] = {}
        self._download_failed: set[int] = set()
        self._lock = threading.Lock()

        if not self._terrain_data_path and self._auto_download:
            home = os.environ.get('HOME', '/tmp')
            self._terrain_data_path = os.path.join(
                home, '.cache', 'mavros', 'terrain', self._srtm_source
            )
            logger.info('Auto-download cache: %s', self._terrain_data_path)

        if self._terrain_data_path:
            Path(self._terrain_data_path).mkdir(parents=True, exist_ok=True)
            self._build_file_index()

    def _build_file_index(self) -> None:
        root = Path(self._terrain_data_path)
        if not root.exists():
            return
        count = 0
        for hgt in root.rglob('*.hgt'):
            self._file_index[hgt.name] = hgt
            count += 1
        logger.info('Indexed %d .hgt files in %s', count, self._terrain_data_path)

    # ------------------------------------------------------------------ keys

    @staticmethod
    def _tile_key(lat: int, lon: int) -> int:
        """Return a unique integer key for a 1-degree tile corner."""
        return (lat + 90) * 360 + (lon + 180)

    @staticmethod
    def _tile_filename(lat: int, lon: int) -> str:
        """Return the standard .hgt filename for a tile (e.g. N47E011.hgt)."""
        ns = 'N' if lat >= 0 else 'S'
        ew = 'E' if lon >= 0 else 'W'
        return f'{ns}{abs(lat):02d}{ew}{abs(lon):03d}.hgt'

    # ------------------------------------------------------------------ load

    def _load_tile(self, lat: int, lon: int) -> SrtmTile | None:
        if not self._terrain_data_path:
            return None

        filename = self._tile_filename(lat, lon)
        filepath = self._file_index.get(filename)

        if filepath is None:
            filepath = Path(self._terrain_data_path) / filename
            if not filepath.exists():
                return None
            self._file_index[filename] = filepath

        file_size = filepath.stat().st_size
        expected_1 = SRTM1_SIDE * SRTM1_SIDE * 2
        expected_3 = SRTM3_SIDE * SRTM3_SIDE * 2

        if file_size == expected_1:
            side = SRTM1_SIDE
        elif file_size == expected_3:
            side = SRTM3_SIDE
        else:
            logger.warning(
                'Unexpected file size for %s: %d bytes (expected %d or %d)',
                filename,
                file_size,
                expected_3,
                expected_1,
            )
            return None

        raw = filepath.read_bytes()
        n = side * side
        data = array.array('h', struct.unpack(f'>{n}h', raw))

        logger.info('Loaded tile %s (%d×%d)', filename, side, side)
        return SrtmTile(data, side)

    # ------------------------------------------------------------------ download

    def _download_tile(self, lat: int, lon: int) -> bool:
        """
        Download a tile zip from the ArduPilot SRTM mirror.

        Use the continent lookup table for a direct download when available,
        falling back to trying all continent directories sequentially.
        Only the expected .hgt file is extracted to prevent zip-slip attacks.
        """
        filename = self._tile_filename(lat, lon)
        hgt_path = Path(self._terrain_data_path) / filename
        if hgt_path.exists():
            return True

        zip_name = filename + '.zip'
        base_url = f'https://{self._download_host}/{self._srtm_source}'

        continents: tuple[str, ...] | list[str]
        if _lookup_continent is not None:
            known = _lookup_continent(lat, lon)
            if known is None:
                logger.info('No SRTM coverage for %s (continent map)', filename)
                return False
            continents = (known,)
        else:
            continents = _CONTINENTS

        zip_bytes: bytes | None = None
        for continent in continents:
            url = f'{base_url}/{continent}{zip_name}'
            if not url.startswith(('https://', 'http://')):
                raise ValueError(f'Refusing non-HTTP URL: {url}')
            try:
                logger.info('Downloading %s', url)
                # URL scheme is validated above; host is a trusted configuration parameter.
                with urllib.request.urlopen(  # nosemgrep: dynamic-urllib-use-detected
                    urllib.request.Request(url), timeout=60
                ) as resp:
                    zip_bytes = resp.read()
                break
            except urllib.error.URLError:
                continue

        if zip_bytes is None:
            logger.warning('Tile %s not found on server', filename)
            return False

        try:
            with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
                if filename not in zf.namelist():
                    logger.warning('Archive for %s does not contain the expected .hgt', filename)
                    return False
                zf.extract(filename, self._terrain_data_path)
        except zipfile.BadZipFile:
            logger.error('Corrupt zip for %s', filename)
            return False

        if hgt_path.exists():
            logger.info('Downloaded: %s', filename)
            with self._lock:
                self._file_index[filename] = hgt_path
            return True

        logger.warning('Extraction produced no .hgt for %s', filename)
        return False

    # ------------------------------------------------------------------ cache

    def _get_tile(self, lat: int, lon: int) -> SrtmTile | None:
        """Return a tile from cache, loading or downloading as needed."""
        key = self._tile_key(lat, lon)

        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
                return self._cache[key]

        tile = self._load_tile(lat, lon)

        if tile is None and self._auto_download:
            if key not in self._download_failed:
                if self._download_tile(lat, lon):
                    tile = self._load_tile(lat, lon)
                else:
                    self._download_failed.add(key)

        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
                return self._cache[key]
            self._cache[key] = tile
            while len(self._cache) > self._max_cache_tiles:
                self._cache.popitem(last=False)

        return tile

    # ------------------------------------------------------------------ elevation

    def lookup_elevation(self, lat_deg: float, lon_deg: float) -> float | None:
        """
        Interpolate elevation at a WGS-84 coordinate using bilinear weights.

        When one or more of the four surrounding grid cells are void,
        the available corners are averaged with renormalized bilinear weights
        instead of falling back to a single arbitrary sample.
        """
        tlat = math.floor(lat_deg)
        tlon = math.floor(lon_deg)

        tile = self._get_tile(tlat, tlon)
        if tile is None:
            return None

        frac_lat = lat_deg - tlat
        frac_lon = lon_deg - tlon

        row_f = (1.0 - frac_lat) * (tile.side - 1)
        col_f = frac_lon * (tile.side - 1)

        r0 = max(0, min(int(math.floor(row_f)), tile.side - 2))
        c0 = max(0, min(int(math.floor(col_f)), tile.side - 2))

        fr = row_f - r0
        fc = col_f - c0

        s = tile.side
        v00 = tile.data[r0 * s + c0]
        v01 = tile.data[r0 * s + c0 + 1]
        v10 = tile.data[(r0 + 1) * s + c0]
        v11 = tile.data[(r0 + 1) * s + c0 + 1]

        corners = (
            (v00, (1 - fc) * (1 - fr)),
            (v01, fc * (1 - fr)),
            (v10, (1 - fc) * fr),
            (v11, fc * fr),
        )
        valid = [(float(v), w) for v, w in corners if v != SRTM_VOID]

        if not valid:
            return None
        if len(valid) == 4:
            return sum(v * w for v, w in valid)

        total_w = sum(w for _, w in valid)
        return sum(v * w for v, w in valid) / total_w


# ---------------------------------------------------------------------- geodesic


def gps_newpos(
    lat_deg: float, lon_deg: float, bearing_deg: float, distance_m: float
) -> tuple[float, float]:
    """
    Compute a new position along a rhumb line.

    Match MAVProxy ``mp_util.gps_newpos`` for consistency with ArduPilot.
    """
    if distance_m == 0.0:
        return (lat_deg, lon_deg)

    lat1 = max(-math.pi / 2 + 1e-15, min(math.pi / 2 - 1e-15, math.radians(lat_deg)))
    lon1 = math.radians(lon_deg)
    tc = -math.radians(bearing_deg)
    d = distance_m / RADIUS_OF_EARTH

    lat = lat1 + d * math.cos(tc)
    lat = max(-math.pi / 2 + 1e-15, min(math.pi / 2 - 1e-15, lat))

    if abs(lat - lat1) < 1e-15:
        q = math.cos(lat1)
    else:
        dphi = math.log(math.tan(lat / 2 + math.pi / 4) / math.tan(lat1 / 2 + math.pi / 4))
        q = (lat - lat1) / dphi

    dlon = -d * math.sin(tc) / q
    lon = math.fmod(lon1 + dlon + math.pi, 2 * math.pi) - math.pi

    return (math.degrees(lat), math.degrees(lon))


def gps_offset(
    lat_deg: float, lon_deg: float, east_m: float, north_m: float
) -> tuple[float, float]:
    """Offset a position by east/north meters."""
    bearing = math.degrees(math.atan2(east_m, north_m))
    distance = math.hypot(east_m, north_m)
    return gps_newpos(lat_deg, lon_deg, bearing, distance)


# ---------------------------------------------------------------------- grid protocol


def compute_terrain_data_block(
    mgr: SrtmManager,
    lat_e7: int,
    lon_e7: int,
    grid_spacing: int,
    bit: int,
) -> list[int] | None:
    """
    Compute the 16 elevation values for one 4x4 terrain block.

    Parameter names match the MAVLink TERRAIN_DATA message fields.
    Return a list of 16 int16 elevations (row-major within the block),
    or None if any sample is unavailable.
    """
    base_lat = lat_e7 / 1e7
    base_lon = lon_e7 / 1e7
    spacing = float(grid_spacing)
    bit_spacing = spacing * TILE_DIM

    col = bit % GRID_COLS
    row = bit // GRID_COLS

    tile_lat, tile_lon = gps_offset(base_lat, base_lon, bit_spacing * col, bit_spacing * row)

    data: list[int] = []
    for i in range(TILE_DIM * TILE_DIM):
        y = i % TILE_DIM
        x = i // TILE_DIM

        pt_lat, pt_lon = gps_offset(tile_lat, tile_lon, spacing * y, spacing * x)
        elev = mgr.lookup_elevation(pt_lat, pt_lon)
        if elev is None:
            return None
        data.append(int(round(elev)))

    return data
