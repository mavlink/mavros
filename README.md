MAVROS
======
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/mavlink/mavros)](https://github.com/mavlink/mavros/releases)  [![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)  [![CI](https://github.com/mavlink/mavros/actions/workflows/main.yml/badge.svg)](https://github.com/mavlink/mavros/actions/workflows/main.yml)

MAVLink extendable communication node for ROS.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support separated from master to hydro-devel branch.
- Since 2015-03-04 all packages also dual licensed under terms of BSD license.
- Since 2015-08-10 all messages moved to mavros\_msgs package
- Since 2016-02-05 (v0.17) frame conversion changed again
- Since 2016-06-22 (pre v0.18) Indigo and Jade separated from master to indigo-devel branch.
- Since 2016-06-23 (0.18.0) support MAVLink 2.0 without signing.
- Since 2017-08-23 (0.20.0) [GeographicLib][geolib] and it's datasets are required. Used to convert AMSL (FCU) and WGS84 (ROS) altitudes.
- Since 2018-05-11 (0.25.0) support building master for Indigo and Jade stopped. Mainly because update of console-bridge package.
- Since 2018-05-14 (0.25.1) support for Indigo returned. We use compatibility layer for console-bridge.
- Since 2019-01-03 (0.28.0) support for Indigo by master not guaranteed. Consider update to more recent distro.
- 2020-01-01 version 1.0.0 released, please see [#1369][iss1369] for reasons and its purpose.
- 2021-05-28 version 2.0.0 released, it's the first alpha release for ROS2.
- 2023-09-09 version 2.6.0, dropped support for EOLed ROS2 releases. Now it require Humble+ (Humble, Iron, Rolling...).


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
  - ROS2 Iron: [![Build Status](https://build.ros2.org/job/Idev__mavros__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mavros__ubuntu_jammy_amd64/)
  - ROS2 Jazzy: [![Build Status](https://build.ros2.org/job/Jdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mavros__ubuntu_noble_amd64/)
  - ROS2 Rolling: [![Build Status](https://build.ros2.org/job/Rdev__mavros__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mavros__ubuntu_noble_amd64/)


[mrrm]: https://github.com/mavlink/mavros/blob/ros2/mavros/README.md
[exrm]: https://github.com/mavlink/mavros/blob/ros2/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/ros2/libmavconn/README.md
[test]: https://github.com/mavlink/mavros/blob/ros2/test_mavros/README.md
[inst]: https://github.com/mavlink/mavros/blob/ros2/mavros/README.md#installation
[geolib]: https://geographiclib.sourceforge.io/
[iss1369]: https://github.com/mavlink/mavros/issues/1369
