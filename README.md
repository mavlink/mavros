MAVROS
======

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

MAVLink extendable communication node for ROS.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support separated from master to hydro-devel branch.
- Since 2015-03-04 all packages also dual licensed under terms of BSD license.
- Since 2015-08-10 all messages moved to mavros\_msgs package
- Since 2016-02-05 (v0.17) frame conversion changed again
- Since 2016-06-22 (pre v0.18) Indigo and Jade separated from master to indigo-devel branch.
- Since 2016-06-23 (0.18.0) support MAVLink 2.0 without signing.
- Since 2017-08-23 (0.20.0) [GeographicLib][geolib] and it's datasets are required. Used to convert AMSL (FCU) and WGS84 (ROS) altitudes.


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

This package contains messages and services used in mavros.


Support forums and chats
------------------------

Please ask your questions not related to bugs/feauture or requests on:

- [px4users Google Group (Mailing List) ](https://groups.google.com/forum/#!forum/px4users)
- [Mavros on Gitter IM](https://gitter.im/mavlink/mavros)
- [PX4/Firmware on Gitter IM](https://gitter.im/PX4/Firmware)
- [ArduPilot/VisionProjects on Gitter IM](https://gitter.im/ArduPilot/ardupilot/VisionProjects)

We'd like to keep the project bugtracker as free as possible, so please contact via the above methods. You can also PM us via Gitter.


CI Statuses
-----------

  - ROS Kinetic: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__mavros__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__mavros__ubuntu_xenial_amd64/)
  - ROS Lunar: [![Build Status](http://build.ros.org/buildStatus/icon?job=Ldev__mavros__ubuntu_xenial_amd64)](http://build.ros.org/job/Ldev__mavros__ubuntu_xenial_amd64/)
  - Travis master on Indigo & Jade: [![travis status](https://travis-ci.org/mavlink/mavros.svg?branch=master)](https://travis-ci.org/mavlink/mavros)


[mrrm]: https://github.com/mavlink/mavros/blob/master/mavros/README.md
[exrm]: https://github.com/mavlink/mavros/blob/master/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/master/libmavconn/README.md
[test]: https://github.com/mavlink/mavros/blob/master/test_mavros/README.md
[inst]: https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation
[geolib]: https://geographiclib.sourceforge.io/
