# Project history

Ancient roadmap / release notes for MAVROS. Please see the [Changelog](changelog.md)
for the detailed per-version change log.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support separated from master to hydro-devel branch.
- Since 2015-03-04 all packages also dual licensed under terms of BSD license.
- Since 2015-08-10 all messages moved to mavros_msgs package
- Since 2016-02-05 (v0.17) frame conversion changed again
- Since 2016-06-22 (pre v0.18) Indigo and Jade separated from master to indigo-devel branch.
- Since 2016-06-23 (0.18.0) support MAVLink 2.0 without signing.
- Since 2017-08-23 (0.20.0) [GeographicLib](https://geographiclib.sourceforge.io/) and it's datasets are required. Used to convert AMSL (FCU) and WGS84 (ROS) altitudes.
- Since 2018-05-11 (0.25.0) support building master for Indigo and Jade stopped. Mainly because update of console-bridge package.
- Since 2018-05-14 (0.25.1) support for Indigo returned. We use compatibility layer for console-bridge.
- Since 2019-01-03 (0.28.0) support for Indigo by master not guaranteed. Consider update to more recent distro.
- 2020-01-01 version 1.0.0 released, please see [issue #1369](https://github.com/mavlink/mavros/issues/1369) for reasons and its purpose.
- 2021-05-28 version 2.0.0 released, it's the first alpha release for ROS2.
- 2023-09-09 version 2.6.0, dropped support for EOLed ROS2 releases. Now it require Humble+ (Humble, Iron, Rolling...).