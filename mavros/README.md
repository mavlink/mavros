MAVROS
======

MAVLink extendable communication node for ROS2.

The ROS API (topics, services, parameters) of every plugin is documented in the
[plugin reference](https://mavros.readthedocs.io/en/latest/plugins/) and the
[full documentation](https://mavros.readthedocs.io/).


Features
--------

  - Communication with autopilot via serial port, UDP or TCP (e.g. [PX4 Pro][px4] or [ArduPilot][apm])
  - Internal proxy for Ground Control Station (serial, UDP, TCP)
  - Plugin system for ROS-MAVLink translation
  - Parameter manipulation tool
  - Waypoint manipulation tool
  - PX4Flow support (by [mavros\_extras][mrext])
  - OFFBOARD mode support
  - Geographic coordinates conversions.


Limitations
-----------

Only for Linux.

This package depends on the [ros-*-mavlink][ml] MAVLink library (built from [mavlink-gbp-release][mlgbp]).
It exists in ROS package index and usually updates each month.

Connection URL
--------------

Connection defined by URL, you can use any supported type for FCU and GCS.

Supported schemas:

  - Serial: `/path/to/serial/device[:baudrate]`
  - Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - Serial with hardware flow control: `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - UDP: `udp://[bind_host][:port]@[remote_host[:port]][/?ids=sysid,compid]`
  - UDP broadcast until GCS discovery: `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - UDP broadcast (permanent): `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
  - TCP server: `tcp-l://[bind_host][:port][/?ids=sysid,compid]`

Note:

  - Ids from URL overrides value given by system\_id & component\_id parameters.
  - bind\_host - default `0.0.0.0` - i.e. IP4 ANY
  - UDP default ports: 14555 @ 14550
  - UDP remote address updated every time with incoming packet on bind port.
  - TCP default port: 5760


Coordinate frames
-----------------

MAVROS does translate Aerospace NED frames, used in FCUs to ROS ENU frames and vice-versa.
For translate airframe related data we simply apply rotation 180° about ROLL (X) axis.
For local we apply 180° about ROLL (X) and 90° about YAW (Z) axes.
Please read documents from issue #473 for additional information.

All the conversions are handled in `src/lib/ftf_frame_conversions.cpp` and `src/lib/ftf_quaternion_utils.cpp` and tested in `test/test_frame_conversions.cpp` and `test/test_quaternion_utils.cpp` respectively.

Related issues: [#49 (outdated)][iss49], [#216 (outdated)][iss216], [#317 (outdated)][iss317], [#319 (outdated)][iss319], [#321 (outdated)][iss321], [#473][iss473].
Documents: [Frame Conversions][iss473rfc], [Mavlink coordinate frames][iss473table].

MAVROS also allows conversion of geodetic and geocentric coordinates through [GeographicLib][geolib]
given that:
  - `geographic_msgs` and `NatSatFix.msg` require the LLA fields to be filled in WGS-84 datum,
  meaning that the altitude should be the height above the WGS-84 ellipsoid. For that, a conversion
  from the height above the geoid (AMSL, considering the egm96 geoid model) to height above the
  WGS-84 ellipsoid, and vice-versa, is available and used in several plugins;
  - According to ROS REP 105, the `earth` frame should be propagated in ECEF (Earth-Centered,
  Earth-Fixed) local coordinates. For that, the functionalities of GeographicLib are used in
  order to allow conversion from geodetic coordinates to geocentric coordinates;
  - The translation from GPS coordinates to local geocentric coordinates require the definition
  of a local origin on the `map` frame, in ECEF, and calculate the offset to it in ENU. All
  the conversions are supported by GeographicLib classes and methods and implemented in the
  `global_position` plugin.


Composite nodes
---------------

See also: https://docs.ros.org/en/foxy/Tutorials/Composition.html

### mavros::router::Router

This is router node required to support connections to FCU(s), GCS(es) and UAS nodes.
The Router allows you to add/remove endpoints on the fly without node restart.

### mavros::uas::UAS

This node is a plugin container which manages all protocol plugins.
Each plugin is a subnode to this.


Programs
--------

### mavros\_node -- all-in-one container

That is a preconfigured composite node container, which provides similar parameters as ROS1 mavros\_node.
That container loads Router, UAS and configures them to work together (sets uas\_link, etc.).

Main node. Allow disable GCS proxy by setting empty URL.

    ros2 run mavros mavros_node --ros-args --params-file params.yaml


Executors
---------

MAVROS builds its executors via a factory that honors two environment variables:

  - `MAVROS_EXECUTOR_TYPE` - executor used by both `mavros_node` container and
    the UAS plugin executor:
      - `mt` (default) - `rclcpp::executors::MultiThreadedExecutor`
      - `events` (alias `cbg`) - the Callback Group Events executor
        (`rclcpp::executors::EventsCBGExecutor`), available only on Lyrical+
        (rclcpp >= 30.0.0). On older distros the request is ignored with a
        warning and `MultiThreadedExecutor` is used.
  - `MAVROS_UAS_EXECUTOR_THREADS` - number of threads for the UAS plugin
    executor (default: clamped hardware concurrency, min 2 max 4; must be
    >= 2 if set).

Example, running `mavros_node` with the events executor:

    MAVROS_EXECUTOR_TYPE=events ros2 run mavros mavros_node

When MAVROS is used as composable nodes inside a component container, the
container's executor is chosen with the container's own `--executor-type`
argument instead; see the composable launch below.


Launch Files
------------

**XXX TODO**! #1564

Launch files are provided for use with common FCUs, in particular [Pixhawk](https://pixhawk.org/):

  * [px4.launch](launch/px4.launch) -- for use with the PX4 Autopilot (for VTOL, multicopters and planes)
  * [apm.launch](launch/apm.launch) -- for use with APM flight stacks (e.g., all versions of ArduPlane, ArduCopter, etc)
  * [test_compose.launch.py](launch/test_compose.launch.py) -- loads `mavros::router::Router` and one or two
    `mavros::uas::UAS` nodes as composable nodes into a component container.

`test_compose.launch.py` accepts:

  - `fcu_url` (default `udp://0.0.0.0:14540@`) - FCU connection URL
  - `gcs_url` (default `udp://127.0.0.1:14555@`) - GCS connection URL
  - `executor` (default `mt`) - container executor: `mt`, `events`, or `auto`
    (`events` on Lyrical+, `mt` otherwise). The events (Callback Group
    Events) executor support inside a component container is still immature
    upstream (ros2/rclcpp#3186), so the reliable `component_container_mt` is
    the default.

Example:

    ros2 launch mavros test_compose.launch.py fcu_url:=udp://@192.168.60.192:15000 executor:=events

Components can also be loaded/unloaded at runtime against a running
container:

    ros2 component load /mavros_container mavros mavros::router::Router
    ros2 component load /mavros_container mavros mavros::uas::UAS
    ros2 component unload /mavros_container <component_uid>

Examples:

    ros2 launch mavros px4.launch
    ros2 launch mavros apm.launch fcu_url:=tcp://localhost gcs_url:=udp://@


Installation
------------

Installation instructions (binary, source and container) are maintained in the
[installation guide](https://mavros.readthedocs.io/en/latest/installation/) in
the documentation.

Troubleshooting
------------

### Error: serial0: receive: End of file
This issue should have been solve in mavros v0.23.2, it was found to be a Boost.ASIO error and should be fix in release > v1.12.0 ( >Boost 1.66).


Contributing
------------
See [CONTRIBUTING.md][contr].


Glossary
--------

  - *GCS* — Ground Control Station
  - *FCU* — Flight Control Unit (aka *FC*)
  - *OBC* — OnBoard Computer (your odroid or raspberry)


Links
-----

  - [MAVLink][ml] -- The communication protocol for Drones, used by flight controllers, ground control stations, and peripherals
  - [mavlink\_ros][mlros] -- original ROS node (few messages, no proxy)
  - [Pixhawk][pixhawk] -- Open Standards for drone hardware
  - [PX4 Autopilot][px4] -- Flight Controller with support for most vehicle types and hardened/tested MAVROS support
  - [ArduPilot][apm] -- tested autopilot APM:Plane (default command set)
  - [QGroundControl][qgc] -- Ground Control Station for MAVLink autopilots, with tested support for Android, iOS, Mac OS, Linux, and Windows
  - [mavros\_extras][mrext] -- extra plugins & node for mavros


[qgc]: https://qgroundcontrol.com/
[pixhawk]: https://pixhawk.org/
[px4]: https://px4.io/
[apm]: https://ardupilot.com/
[mlros]: https://github.com/mavlink/mavlink_ros
[ml]: https://mavlink.io/en/
[mlgbp]: https://github.com/mavlink/mavlink-gbp-release
[iss49]: https://github.com/mavlink/mavros/issues/49
[iss216]: https://github.com/mavlink/mavros/issues/216
[iss317]: https://github.com/mavlink/mavros/issues/317
[iss319]: https://github.com/mavlink/mavros/issues/319
[iss321]: https://github.com/mavlink/mavros/issues/321
[iss473]: https://github.com/mavlink/mavros/issues/473
[mrext]: https://github.com/mavlink/mavros/tree/master/mavros_extras
[iss473rfc]: https://docs.google.com/document/d/1bDhaozrUu9F915T58WGzZeOM-McyU20dwxX-NRum1KA/edit
[iss473table]: https://docs.google.com/spreadsheets/d/1LnsWTblU92J5_SMinTvBvHJWx6sqvzFa8SKbn8TXlnU/edit#gid=0
[geolib]: https://geographiclib.sourceforge.io/
[contr]: https://github.com/mavlink/mavros/blob/master/CONTRIBUTING.md
