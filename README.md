# MAVROS

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/mavlink/mavros)](https://github.com/mavlink/mavros/releases) [![Documentation](https://readthedocs.org/projects/mavros/badge/?version=latest)](https://mavros.readthedocs.io/en/latest/) [![CI](https://github.com/mavlink/mavros/actions/workflows/main.yml/badge.svg)](https://github.com/mavlink/mavros/actions/workflows/main.yml) [![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros)

**A MAVLink-to-ROS 2 bridge for drones.**

MAVROS turns a MAVLink autopilot (PX4, ArduPilot, …) into a set of familiar
ROS 2 topics, services and parameters, so you can program a drone in ROS the
same way you would program any other robot — no need to speak MAVLink.

---

## What is it?

MAVLink is the wire protocol most flight controllers speak. ROS 2 is a
robotics framework for building software. MAVROS is the glue between them:

```mermaid
flowchart LR
    subgraph FCU["Flight controller"]
        AP[PX4 / ArduPilot]
    end
    subgraph MAV["MAVROS node"]
        BR[bridge / plugins]
    end
    subgraph APP["Your ROS 2 app"]
        N1[telemetry nodes]
        N2[command nodes]
        N3[mission / GCS nodes]
    end
    AP -- MAVLink (UDP/TCP/serial) --> BR
    BR -- ROS 2 topics / services / params --> N1
    BR --> N2
    BR --> N3
```

It does the boring, low-level work for you:

- **Telemetry out** — autopilot state, IMU, GPS, attitude, battery, and more
  arrive as standard ROS 2 `sensor_msgs`/`mavros_msgs` topics.
- **Commanding in** — arm/disarm, takeoff, land, set mode, upload missions and
  geofences through familiar ROS 2 services.
- **Routing** — MAVROS can proxy between a ground station, the FCU and multiple
  ROS processes over a single link.

## Why MAVROS?

- **Tested** against PX4 and ArduPilot, including SITL simulation.
- **Extensible** — a plugin architecture translates each MAVLink message; add
  behaviour without touching the core.
- **ROS 2 native** — supports Humble and newer ROS 2 releases.
- **Flexible transport** — USB/serial, UDP, TCP, or a shared UAS bus.

## Quick start

Install the package and its mandatory GeographicLib datasets:

```shell
sudo apt install ros-${ROS_DISTRO}-mavros
sudo ros2 run mavros install_geographiclib_datasets.sh
```

Launch the bridge against a flight controller (or a simulated one):

```shell
ros2 launch mavros node.launch \
  fcu_url:=udp://127.0.0.1:14550@127.0.0.1:14557
```

Then inspect the sensor stream:

```shell
ros2 topic echo /mavros/imu/data
```

See the [installation guide](docs/installation.md) for binary, source and
container installs, and the [examples](docs/examples/) for task control and
waypoint missions.

## Packages

| Package | Purpose |
|---------|---------|
| `mavros` | Core nodes (Router, UAS) and standard plugins |
| `mavros_extras` | Optional plugins (cameras, gimbals, terrain, …) |
| `mavros_msgs` | ROS 2 message and service interfaces |
| `libmavconn` | MAVLink transport library (usable outside ROS) |
| `test_mavros` | Hand-tests and SITL manual for PX4/ArduPilot |
| `mavros_examples` | Beginner-friendly ROS 2 example scripts (task control, waypoint missions, offboard) |

## Documentation

- [Full documentation](https://mavros.readthedocs.io/)
- [Plugin reference — every topic/service/parameter](docs/plugins/index.md)
- [Per-package C++/Python API](https://docs.ros.org/en/rolling/p/mavros/)
- [Changelog](https://mavros.readthedocs.io/en/latest/changelog/)

## Support

Ask questions (not bug reports) on:

- [MAVROS Gitter](https://gitter.im/mavlink/mavros)
- [PX4 Discuss](https://discuss.px4.io/)
- [PX4 Slack](https://slack.px4.io/)
- [ArduPilot Discuss](https://discuss.ardupilot.org/)

## CI status

- ROS2 Humble: [![Build Status](https://build.ros2.org/job/Hdev__mavros__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mavros__ubuntu_jammy_amd64/)
- ROS2 Jazzy: [![Build Status](https://build.ros2.org/job/Jdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mavros__ubuntu_noble_amd64/)
- ROS2 Kilted: [![Build Status](https://build.ros2.org/job/Kdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mavros__ubuntu_noble_amd64/)
- ROS2 Lyrical: [![Build Status](https://build.ros2.org/job/Ldev__mavros__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mavros__ubuntu_resolute_amd64/)
- ROS2 Rolling: [![Build Status](https://build.ros2.org/job/Rdev__mavros__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mavros__ubuntu_resolute_amd64/)
