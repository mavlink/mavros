"""
Terrain tile server for MAVLink TERRAIN protocol.

Subscribes to TERRAIN_REQUEST from the MAVROS terrain plugin, looks up
SRTM elevation data, and publishes TERRAIN_DATA blocks back.
Also provides a TERRAIN_CHECK service for point elevation queries.
"""

from __future__ import annotations

from .node import main, TerrainServerNode

__all__ = ['TerrainServerNode', 'main']
