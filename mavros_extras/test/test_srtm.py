"""Unit tests for mavros_extras.terrain_server.srtm module."""

from __future__ import annotations

import array
import struct

from mavros_extras.terrain_server.srtm import (
    compute_terrain_data_block,
    gps_newpos,
    gps_offset,
    GRID_COLS,
    GRID_ROWS,
    SRTM1_SIDE,
    SRTM3_SIDE,
    SRTM_VOID,
    SrtmManager,
    SrtmTile,
    TILE_DIM,
)
import pytest


# ------------------------------------------------------------------ SrtmTile


class TestSrtmTile:
    """Test SrtmTile data class."""

    def test_repr(self):
        tile = SrtmTile(array.array('h', [0]), 1201)
        assert 'SrtmTile(side=1201)' == repr(tile)

    def test_slots(self):
        tile = SrtmTile(array.array('h', [42]), 3601)
        assert tile.side == 3601
        assert tile.data[0] == 42


# ------------------------------------------------------------------ tile key / filename


class TestTileKeyFilename:
    """Test tile key and filename generation."""

    def test_key_unique(self):
        keys = {SrtmManager._tile_key(lat, lon) for lat in (-90, 0, 89) for lon in (-180, 0, 179)}
        assert len(keys) == 9

    def test_key_deterministic(self):
        assert SrtmManager._tile_key(47, 11) == SrtmManager._tile_key(47, 11)

    @pytest.mark.parametrize(
        ('lat', 'lon', 'expected'),
        [
            (47, 11, 'N47E011.hgt'),
            (-34, -58, 'S34W058.hgt'),
            (0, 0, 'N00E000.hgt'),
            (-1, -1, 'S01W001.hgt'),
            (89, 179, 'N89E179.hgt'),
            (-90, -180, 'S90W180.hgt'),
        ],
    )
    def test_tile_filename(self, lat, lon, expected):
        assert SrtmManager._tile_filename(lat, lon) == expected


# ------------------------------------------------------------------ synthetic tile helpers


def _make_flat_tile(side: int, elevation: int = 500) -> bytes:
    """Create a synthetic .hgt file with uniform elevation."""
    n = side * side
    return struct.pack(f'>{n}h', *([elevation] * n))


def _make_gradient_tile(side: int) -> bytes:
    """Create a tile where elevation = row index (south-to-north gradient)."""
    data = []
    for row in range(side):
        data.extend([row] * side)
    return struct.pack(f'>{side * side}h', *data)


def _make_void_tile(side: int) -> bytes:
    """Create a tile filled with SRTM void markers."""
    n = side * side
    return struct.pack(f'>{n}h', *([SRTM_VOID] * n))


# ------------------------------------------------------------------ SrtmManager load


class TestSrtmManagerLoad:
    """Test tile loading from disk."""

    def test_load_srtm3_flat(self, tmp_path):
        hgt = tmp_path / 'N47E011.hgt'
        hgt.write_bytes(_make_flat_tile(SRTM3_SIDE, 800))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        elev = mgr.lookup_elevation(47.5, 11.5)
        assert elev is not None
        assert abs(elev - 800.0) < 1e-6

    def test_load_srtm1_flat(self, tmp_path):
        hgt = tmp_path / 'N47E011.hgt'
        hgt.write_bytes(_make_flat_tile(SRTM1_SIDE, 1200))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        elev = mgr.lookup_elevation(47.5, 11.5)
        assert elev is not None
        assert abs(elev - 1200.0) < 1e-6

    def test_load_missing_tile(self, tmp_path):
        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        assert mgr.lookup_elevation(47.5, 11.5) is None

    def test_load_bad_file_size(self, tmp_path):
        hgt = tmp_path / 'N47E011.hgt'
        hgt.write_bytes(b'\x00' * 100)

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        assert mgr.lookup_elevation(47.5, 11.5) is None

    def test_no_data_path(self):
        mgr = SrtmManager(terrain_data_path='')
        assert mgr.lookup_elevation(47.5, 11.5) is None


# ------------------------------------------------------------------ elevation lookup


class TestElevationLookup:
    """Test bilinear elevation interpolation."""

    def test_corner_exact(self, tmp_path):
        """Elevation at exact SW corner of the tile."""
        hgt = tmp_path / 'N00E000.hgt'
        hgt.write_bytes(_make_flat_tile(SRTM3_SIDE, 100))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        elev = mgr.lookup_elevation(0.0, 0.0)
        assert elev is not None
        assert abs(elev - 100.0) < 1e-6

    def test_bilinear_interpolation(self, tmp_path):
        """With a gradient tile, mid-row should interpolate."""
        hgt = tmp_path / 'N00E000.hgt'
        hgt.write_bytes(_make_gradient_tile(SRTM3_SIDE))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        elev = mgr.lookup_elevation(0.5, 0.5)
        assert elev is not None
        expected_row = (1.0 - 0.5) * (SRTM3_SIDE - 1)
        assert abs(elev - expected_row) < 1.0

    def test_void_returns_none(self, tmp_path):
        """All-void tile should return None."""
        hgt = tmp_path / 'N10E020.hgt'
        hgt.write_bytes(_make_void_tile(SRTM3_SIDE))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        assert mgr.lookup_elevation(10.5, 20.5) is None


# ------------------------------------------------------------------ LRU cache


class TestLRUCache:
    """Test LRU tile cache behavior."""

    def test_cache_eviction(self, tmp_path):
        for lat in range(5):
            hgt = tmp_path / f'N{lat:02d}E000.hgt'
            hgt.write_bytes(_make_flat_tile(SRTM3_SIDE, lat * 100))

        mgr = SrtmManager(terrain_data_path=str(tmp_path), max_cache_tiles=2)

        assert mgr.lookup_elevation(0.5, 0.5) is not None
        assert mgr.lookup_elevation(1.5, 0.5) is not None
        assert mgr.lookup_elevation(2.5, 0.5) is not None

        assert len(mgr._cache) <= 2

    def test_cache_hit_returns_same(self, tmp_path):
        hgt = tmp_path / 'N05E005.hgt'
        hgt.write_bytes(_make_flat_tile(SRTM3_SIDE, 333))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        e1 = mgr.lookup_elevation(5.5, 5.5)
        e2 = mgr.lookup_elevation(5.5, 5.5)
        assert e1 == e2


# ------------------------------------------------------------------ gps_newpos / gps_offset


class TestGeodesic:
    """Test rhumb-line geodesic helpers."""

    def test_zero_distance(self):
        lat, lon = gps_newpos(47.0, 11.0, 90.0, 0.0)
        assert lat == 47.0
        assert lon == 11.0

    def test_north(self):
        lat, lon = gps_newpos(0.0, 0.0, 0.0, 111_320.0)
        assert lat == pytest.approx(1.0, abs=0.02)
        assert lon == pytest.approx(0.0, abs=1e-6)

    def test_east(self):
        lat, lon = gps_newpos(0.0, 0.0, 90.0, 111_320.0)
        assert lat == pytest.approx(0.0, abs=0.02)
        assert lon == pytest.approx(1.0, abs=0.02)

    def test_south(self):
        lat, lon = gps_newpos(10.0, 0.0, 180.0, 111_320.0)
        assert lat == pytest.approx(9.0, abs=0.02)

    def test_symmetry(self):
        lat1, lon1 = gps_newpos(45.0, 10.0, 45.0, 10000.0)
        lat2, lon2 = gps_newpos(lat1, lon1, 225.0, 10000.0)
        assert lat2 == pytest.approx(45.0, abs=0.001)
        assert lon2 == pytest.approx(10.0, abs=0.001)

    def test_gps_offset_north(self):
        lat, lon = gps_offset(0.0, 0.0, 0.0, 111_320.0)
        assert lat == pytest.approx(1.0, abs=0.02)

    def test_gps_offset_east(self):
        lat, lon = gps_offset(0.0, 0.0, 111_320.0, 0.0)
        assert lon == pytest.approx(1.0, abs=0.02)


# ------------------------------------------------------------------ compute_terrain_data_block


class TestComputeTerrainDataBlock:
    """Test MAVLink terrain data block computation."""

    def test_flat_tile_block(self, tmp_path):
        hgt = tmp_path / 'N47E011.hgt'
        hgt.write_bytes(_make_flat_tile(SRTM3_SIDE, 600))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        block = compute_terrain_data_block(
            mgr, lat_e7=470_000_000, lon_e7=110_000_000, grid_spacing=100, bit=0
        )
        assert block is not None
        assert len(block) == TILE_DIM * TILE_DIM
        assert all(v == 600 for v in block)

    def test_missing_tile_returns_none(self, tmp_path):
        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        block = compute_terrain_data_block(
            mgr, lat_e7=470_000_000, lon_e7=110_000_000, grid_spacing=100, bit=0
        )
        assert block is None

    def test_block_size(self, tmp_path):
        hgt = tmp_path / 'N47E011.hgt'
        hgt.write_bytes(_make_flat_tile(SRTM3_SIDE, 400))

        mgr = SrtmManager(terrain_data_path=str(tmp_path))
        for bit in range(min(4, GRID_COLS * GRID_ROWS)):
            block = compute_terrain_data_block(
                mgr, lat_e7=470_000_000, lon_e7=110_000_000, grid_spacing=30, bit=bit
            )
            if block is not None:
                assert len(block) == 16

    def test_grid_constants(self):
        assert GRID_COLS == 8
        assert GRID_ROWS == 7
        assert TILE_DIM == 4


# ------------------------------------------------------------------ continent map


class TestContinentMap:
    """Test continent lookup table."""

    def test_import(self):
        from mavros_extras.terrain_server.srtm_continent_map import lookup_continent

        assert callable(lookup_continent)

    def test_known_tiles(self):
        from mavros_extras.terrain_server.srtm_continent_map import lookup_continent

        result = lookup_continent(47, 11)
        assert result is not None
        assert 'Eurasia' in result

    def test_out_of_range(self):
        from mavros_extras.terrain_server.srtm_continent_map import lookup_continent

        assert lookup_continent(90, 0) is None
        assert lookup_continent(0, 360) is None

    def test_ocean_tile(self):
        from mavros_extras.terrain_server.srtm_continent_map import lookup_continent

        result = lookup_continent(0, -30)
        assert result is None
