MAVROS
======

MAVLink extendable communication node for ROS 2.

It provides communication with the autopilot via serial port, UDP or TCP (e.g.
[PX4][px4] or [ArduPilot][apm]), an internal proxy for a Ground Control
Station, a plugin system for ROS-MAVLink translation, OFFBOARD mode support,
and geographic coordinate conversions.

> Only for Linux.

The ROS API (topics, services, parameters) of every plugin is documented in the
[plugin reference](https://mavros.readthedocs.io/en/latest/plugins/) and the
[full documentation](https://mavros.readthedocs.io/).

The transport library is split into the standalone [libmavconn](../libmavconn/README.md)
package, which is usable outside ROS.

## Quick start

```shell
sudo apt install ros-${ROS_DISTRO}-mavros
sudo ros2 run mavros install_geographiclib_datasets.sh
ros2 launch mavros px4.launch
```

## Documentation

- [Installation](../docs/installation.md)
- [Connection URLs](../docs/connection_urls.md)
- [Coordinate frames](../docs/frames.md)
- [Nodes and launch](../docs/nodes.md)
- [Troubleshooting](../docs/troubleshooting.md)
- [Plugin reference](../docs/plugins/index.md)

## Links

- [MAVLink][ml] -- The communication protocol for drones, used by flight controllers, ground control stations, and peripherals
- [Pixhawk][pixhawk] -- Open standards for drone hardware
- [PX4 Autopilot][px4] -- Flight controller with support for most vehicle types and hardened/tested MAVROS support
- [ArduPilot][apm] -- tested autopilot stack (ArduPlane, ArduCopter, ArduRover, …)
- [QGroundControl][qgc] -- Ground Control Station for MAVLink autopilots
- [mavros_extras][mrext] -- extra plugins & nodes for mavros


[qgc]: https://qgroundcontrol.com/
[pixhawk]: https://pixhawk.org/
[px4]: https://px4.io/
[apm]: https://ardupilot.com/
[ml]: https://mavlink.io/en/
[mrext]: https://github.com/mavlink/mavros/tree/master/mavros_extras