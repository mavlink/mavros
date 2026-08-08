MAVROS
=====
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/mavlink/mavros)](https://github.com/mavlink/mavros/releases)  [![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)  [![CI](https://github.com/mavlink/mavros/actions/workflows/main.yml/badge.svg)](https://github.com/mavlink/mavros/actions/workflows/main.yml)  [![Documentation](https://readthedocs.org/projects/mavros/badge/?version=latest)](https://mavros.readthedocs.io/en/latest/)

MAVLink extendable communication node for ROS.

- Full documentation: [https://mavros.readthedocs.io/][rtd]
- Plugin reference (all ROS API): [Plugin index](docs/plugins/index.md)
- Per-package C++/Python API reference (auto-generated): [https://docs.ros.org/en/rolling/p/mavros/][api]
- Changelog: [https://mavros.readthedocs.io/en/latest/changelog/][changelog]


mavros package
--------------

It is the main package, please see its [README][mrrm].
Here you may read [installation instructions][inst].


mavros\_extras package
----------------------

This package contains some extra nodes and plugins for mavros, please see its [README][exrm].


libmavconn package
------------------

This package contain mavconn library, see its [README][libmc].
LibMAVConn may be used outside of ROS environment.


test\_mavros package
--------------------

This package contain hand-tests and [manual page][test] for APM and PX4 SITL.
Please see [README][test] first!


mavros\_msgs package
--------------------

This package contains messages and services used in MAVROS.


Support forums and chats
------------------------

Please ask your questions not related to bugs/feature or requests on:

- [MAVROS discussion in Gitter IM](https://gitter.im/mavlink/mavros)
- [PX4 Discuss Forum](https://discuss.px4.io/)
- [PX4 Slack](https://slack.px4.io/)
- [Ardupilot Discuss Forum](https://discuss.ardupilot.org/)
- [ArduPilot/VisionProjects in Gitter IM](https://gitter.im/ArduPilot/ardupilot/VisionProjects)

We'd like to keep the project bug tracker as free as possible, so please contact via the above methods. You can also PM us via Gitter and the PX4 Slack.


CI Statuses
-----------

  - ROS2 Humble: [![Build Status](https://build.ros2.org/job/Hdev__mavros__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mavros__ubuntu_jammy_amd64/)
  - ROS2 Jazzy: [![Build Status](https://build.ros2.org/job/Jdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mavros__ubuntu_noble_amd64/)
  - ROS2 Kilted: [![Build Status](https://build.ros2.org/job/Kdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mavros__ubuntu_noble_amd64/)
  - ROS2 Lyrical: [![Build Status](https://build.ros2.org/job/Ldev__mavros__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__mavros__ubuntu_resolute_amd64/)
  - ROS2 Rolling: [![Build Status](https://build.ros2.org/job/Rdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mavros__ubuntu_noble_amd64/)


[mrrm]: https://github.com/mavlink/mavros/blob/ros2/mavros/README.md
[rtd]: https://mavros.readthedocs.io/en/latest/
[api]: https://docs.ros.org/en/rolling/p/mavros/
[changelog]: https://mavros.readthedocs.io/en/latest/changelog/
[exrm]: https://github.com/mavlink/mavros/blob/ros2/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/ros2/libmavconn/README.md
[test]: https://github.com/mavlink/mavros/blob/ros2/test_mavros/README.md
[inst]: https://github.com/mavlink/mavros/blob/ros2/mavros/README.md#installation
