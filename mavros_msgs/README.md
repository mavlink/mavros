mavros_msgs
===========

ROS 2 message, service and action interfaces used by [MAVROS][mr].

This package defines the MAVLink-facing data types exposed on the ROS 2 graph:

- `msg/` — telemetry and command messages (e.g. `State`, `Imu`, `Waypoint`,
  `GlobalPositionTarget`, `ParamEvent`, …).
- `srv/` — services (e.g. `CommandBool`, `WaypointPush`, `ParamPull`, …).
- `action/` — actions (if any).

Most messages wrap the corresponding MAVLink message fields; see the
[MAVROS plugin reference][plugins] for how they map to topics and services.

## Message / service reference

The generated C++/Python API reference is published on docs.ros.org:

- [mavros_msgs messages](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/)
- [mavros_msgs services](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/)

[plugins]: https://mavros.readthedocs.io/en/latest/plugins/
[mr]: https://github.com/mavlink/mavros