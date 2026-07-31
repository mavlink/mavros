#!/usr/bin/env python3
"""
ROS 2 terrain server node.

Subscribes to TERRAIN_REQUEST from the MAVROS terrain plugin, looks up
SRTM elevation data, and publishes TERRAIN_DATA blocks back.
Also provides a TERRAIN_CHECK service for point elevation queries.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading

from mavros.base import BaseNode
from mavros_msgs.msg import TerrainData, TerrainRequest
from mavros_msgs.srv import TerrainCheck
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.qos import QoSProfile

from .srtm import (
    compute_terrain_data_block,
    GRID_COLS,
    GRID_ROWS,
    SrtmManager,
)


@dataclass
class _PendingRequest:
    """Tracks which 4x4 blocks have been requested vs already sent."""

    lat_deg: float
    lon_deg: float
    grid_spacing: int
    mask: int
    sent_mask: int = 0

    @property
    def remaining(self) -> int:
        """Bitmask of blocks still needed."""
        return self.mask & ~self.sent_mask


class TerrainServerNode(BaseNode):
    """Serves SRTM elevation data in response to MAVLink terrain requests."""

    def __init__(self) -> None:
        super().__init__('terrain_server_node')

        self.declare_parameter(
            'terrain_data_path',
            '',
            ParameterDescriptor(
                description=(
                    'Directory for .hgt tile cache. '
                    'Empty defaults to ~/.cache/mavros/terrain/<srtm_source>/'
                )
            ),
        )
        self.declare_parameter(
            'auto_download',
            False,
            ParameterDescriptor(
                description='Automatically download missing SRTM tiles in background threads.'
            ),
        )
        self.declare_parameter(
            'srtm_data_url',
            'https://terrain.ardupilot.org',
            ParameterDescriptor(
                description=(
                    'SRTM tile download base URL. Use http:// for Squid/nginx cacheability.'
                )
            ),
        )
        self.declare_parameter(
            'srtm_source',
            'SRTM3',
            ParameterDescriptor(
                description='SRTM dataset name (SRTM3 = 3 arc-second, SRTM1 = 1 arc-second).'
            ),
        )
        self.declare_parameter(
            'send_rate_hz',
            5.0,
            ParameterDescriptor(
                description='Rate (Hz) for sending TERRAIN_DATA blocks to the FCU.'
            ),
        )
        self.declare_parameter(
            'max_cache_tiles',
            64,
            ParameterDescriptor(
                description='Maximum number of tiles to keep in the in-memory LRU cache.'
            ),
        )
        self.declare_parameter(
            'proxy',
            '',
            ParameterDescriptor(
                description='HTTP/HTTPS proxy for tile downloads (host:port). Empty uses env vars.'
            ),
        )

        terrain_data_path = self.get_parameter('terrain_data_path').value
        auto_download = self.get_parameter('auto_download').value
        srtm_data_url = self.get_parameter('srtm_data_url').value
        srtm_source = self.get_parameter('srtm_source').value
        rate_hz = self.get_parameter('send_rate_hz').value
        max_cache_tiles = self.get_parameter('max_cache_tiles').value
        proxy = self.get_parameter('proxy').value

        if rate_hz <= 0.0:
            rate_hz = 5.0

        self._mgr = SrtmManager(
            terrain_data_path=terrain_data_path,
            auto_download=auto_download,
            srtm_data_url=srtm_data_url,
            srtm_source=srtm_source,
            max_cache_tiles=max_cache_tiles,
            proxy=proxy,
        )

        self._pending: deque[_PendingRequest] = deque()
        self._lock = threading.Lock()

        self._blocks_served = 0
        self._requests_received = 0

        self._data_pub = self.create_publisher(
            TerrainData,
            self.get_topic('terrain', 'data'),
            QoSProfile(depth=64),
        )

        self.create_subscription(
            TerrainRequest,
            self.get_topic('terrain', 'request'),
            self._on_request,
            QoSProfile(depth=10),
        )

        self.create_service(
            TerrainCheck,
            self.get_topic('terrain', 'check'),
            self._on_check,
        )

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._on_send_tick)

        self.get_logger().info(
            f'Terrain server ready  path={terrain_data_path or "(none)"}'
            f'  auto_download={auto_download}  rate={rate_hz:.1f} Hz'
            f'  proxy={proxy or "(env)"}'
        )

    # ---------------------------------------------------------------- callbacks

    def _on_request(self, msg: TerrainRequest) -> None:
        lat_deg = msg.latitude
        lon_deg = msg.longitude

        with self._lock:
            for req in self._pending:
                if (
                    req.lat_deg == lat_deg
                    and req.lon_deg == lon_deg
                    and req.grid_spacing == msg.grid_spacing
                ):
                    req.mask = msg.mask
                    req.sent_mask &= msg.mask
                    self.get_logger().debug(
                        f'Updated pending request lat={lat_deg:.7f}'
                        f' lon={lon_deg:.7f} mask=0x{msg.mask:016x}'
                    )
                    return

            self._pending.append(_PendingRequest(lat_deg, lon_deg, msg.grid_spacing, msg.mask))
            self._requests_received += 1
            count = self._requests_received

        self.get_logger().info(
            f'TERRAIN_REQUEST #{count} lat={lat_deg:.7f} lon={lon_deg:.7f}'
            f' spacing={msg.grid_spacing} mask=0x{msg.mask:016x}'
        )

    def _on_check(
        self,
        request: TerrainCheck.Request,
        response: TerrainCheck.Response,
    ) -> TerrainCheck.Response:
        elev = self._mgr.lookup_elevation(request.latitude, request.longitude)
        if elev is None:
            response.success = False
            response.terrain_height = 0.0
            self.get_logger().debug(
                f'Check: no data at ({request.latitude:.7f}, {request.longitude:.7f})'
            )
        else:
            response.success = True
            response.terrain_height = float(elev)
        return response

    # ---------------------------------------------------------------- timer

    def _on_send_tick(self) -> None:
        with self._lock:
            while self._pending and self._pending[0].remaining == 0:
                self._pending.popleft()

            if not self._pending:
                return

            req = self._pending[0]
            needed = req.remaining
            lat_deg = req.lat_deg
            lon_deg = req.lon_deg
            grid_spacing = req.grid_spacing

        # Convert to degE7 for compute_terrain_data_block (MAVLink wire format)
        lat_e7 = round(lat_deg * 1e7)
        lon_e7 = round(lon_deg * 1e7)

        for bit in range(GRID_COLS * GRID_ROWS):
            if not (needed & (1 << bit)):
                continue

            data = compute_terrain_data_block(self._mgr, lat_e7, lon_e7, grid_spacing, bit)
            if data is None:
                # Tile not available yet (may be downloading in background)
                continue

            msg = TerrainData()
            msg.latitude = lat_deg
            msg.longitude = lon_deg
            msg.grid_spacing = grid_spacing
            msg.gridbit = bit
            msg.data = data

            self._data_pub.publish(msg)

            with self._lock:
                req.sent_mask |= 1 << bit
                self._blocks_served += 1
                if req.remaining == 0:
                    self.get_logger().info(
                        f'Completed terrain request lat={lat_deg:.7f}'
                        f' lon={lon_deg:.7f}'
                        f' ({self._blocks_served} blocks served total)'
                    )

            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
