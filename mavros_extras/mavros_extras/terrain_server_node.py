#!/usr/bin/env python3
"""ROS 2 terrain server node.

Subscribes to terrain requests from the MAVROS terrain plugin,
looks up SRTM elevation data, and publishes terrain data blocks back.
Also provides a service for point elevation queries.
"""

from __future__ import annotations

import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from mavros_msgs.msg import TerrainData, TerrainRequest
from mavros_msgs.srv import TerrainCheck

from mavros_extras.srtm import (
    GRID_COLS,
    GRID_ROWS,
    SrtmManager,
    compute_terrain_data_block,
)


class _PendingRequest:
    """Tracks which 4x4 blocks have been requested vs already sent."""

    __slots__ = ("lat", "lon", "grid_spacing", "mask", "sent_mask")

    def __init__(
        self, lat: int, lon: int, grid_spacing: int, mask: int
    ) -> None:
        self.lat = lat
        self.lon = lon
        self.grid_spacing = grid_spacing
        self.mask = mask
        self.sent_mask = 0

    @property
    def remaining(self) -> int:
        """Bitmask of blocks still needed."""
        return self.mask & ~self.sent_mask


class TerrainServerNode(Node):
    """Serves SRTM elevation data in response to MAVLink terrain requests."""

    def __init__(self) -> None:
        super().__init__("terrain_server_node")

        self.declare_parameter("terrain_data_path", "")
        self.declare_parameter("auto_download", False)
        self.declare_parameter("download_host", "terrain.ardupilot.org")
        self.declare_parameter("srtm_source", "SRTM3")
        self.declare_parameter("send_rate_hz", 5.0)
        self.declare_parameter("max_cache_tiles", 64)

        terrain_data_path = self.get_parameter("terrain_data_path").value
        auto_download = self.get_parameter("auto_download").value
        download_host = self.get_parameter("download_host").value
        srtm_source = self.get_parameter("srtm_source").value
        rate_hz = self.get_parameter("send_rate_hz").value
        max_cache_tiles = self.get_parameter("max_cache_tiles").value

        if rate_hz <= 0.0:
            rate_hz = 5.0

        self._mgr = SrtmManager(
            terrain_data_path=terrain_data_path,
            auto_download=auto_download,
            download_host=download_host,
            srtm_source=srtm_source,
            max_cache_tiles=max_cache_tiles,
        )

        self._pending: deque[_PendingRequest] = deque()
        self._lock = threading.Lock()

        self._blocks_served = 0
        self._requests_received = 0

        self._data_pub = self.create_publisher(
            TerrainData,
            "/mavros/terrain/data",
            QoSProfile(depth=64),
        )

        self.create_subscription(
            TerrainRequest,
            "/mavros/terrain/request",
            self._on_request,
            QoSProfile(depth=10),
        )

        self.create_service(
            TerrainCheck,
            "/mavros/terrain/check",
            self._on_check,
        )

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._on_send_tick)

        self.get_logger().info(
            f"Terrain server ready  path={terrain_data_path or '(none)'}"
            f"  auto_download={auto_download}  rate={rate_hz:.1f} Hz"
        )

    # ---------------------------------------------------------------- callbacks

    def _on_request(self, msg: TerrainRequest) -> None:
        lat_deg = msg.lat / 1e7
        lon_deg = msg.lon / 1e7

        with self._lock:
            for req in self._pending:
                if (
                    req.lat == msg.lat
                    and req.lon == msg.lon
                    and req.grid_spacing == msg.grid_spacing
                ):
                    req.mask = msg.mask
                    req.sent_mask &= msg.mask
                    self.get_logger().debug(
                        f"Updated pending request lat={lat_deg:.7f}"
                        f" lon={lon_deg:.7f} mask=0x{msg.mask:016x}"
                    )
                    return

            self._pending.append(
                _PendingRequest(msg.lat, msg.lon, msg.grid_spacing, msg.mask)
            )
            self._requests_received += 1
            count = self._requests_received

        self.get_logger().info(
            f"TERRAIN_REQUEST #{count} lat={lat_deg:.7f} lon={lon_deg:.7f}"
            f" spacing={msg.grid_spacing} mask=0x{msg.mask:016x}"
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
                f"Check: no data at ({request.latitude:.7f}, {request.longitude:.7f})"
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

        for bit in range(GRID_COLS * GRID_ROWS):
            if not (needed & (1 << bit)):
                continue

            data = compute_terrain_data_block(
                self._mgr, req.lat, req.lon, req.grid_spacing, bit
            )
            if data is None:
                self.get_logger().debug(
                    f"No elevation data for bit {bit}"
                    f" at ({req.lat / 1e7:.7f}, {req.lon / 1e7:.7f})"
                )
                continue

            msg = TerrainData()
            msg.lat = req.lat
            msg.lon = req.lon
            msg.grid_spacing = req.grid_spacing
            msg.gridbit = bit
            msg.data = data

            self._data_pub.publish(msg)

            with self._lock:
                req.sent_mask |= 1 << bit
                self._blocks_served += 1
                if req.remaining == 0:
                    self.get_logger().info(
                        f"Completed terrain request lat={req.lat / 1e7:.7f}"
                        f" lon={req.lon / 1e7:.7f}"
                        f" ({self._blocks_served} blocks served total)"
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


if __name__ == "__main__":
    main()
