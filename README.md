MAVROS
======

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

MAVLink extendable communication node for ROS.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support splited from master to hydro-devel branch.

mavros package
--------------

It is the main package, please see it's [README][mrrm].


mavros\_extras package
----------------------

This package contain some extra nodes and plugins for mavros, please see it's [README][exrm].


libmavconn package
------------------

This package contain mavconn library, see it's [README][libmc].
LibMAVConn may be used outside of ROS environment.


Support forums and chats
------------------------

Please ask your questions not related to bugs/feauture requests on:

- [px4users Google Group (Mailing List) ](https://groups.google.com/forum/#!forum/px4users)
- [Mavros on Gitter IM](https://gitter.im/mavlink/mavros)
- [PX4/Firmware on Gitter IM](https://gitter.im/PX4/Firmware)

We'd like to keep the project bugtracker as free as possible, so please contact via the above methods. You can also PM us via Gitter.

CI Statuses
-----------

  - ROS Hydro: [![Hydro build status](http://jenkins.ros.org/buildStatus/icon?job=devel-hydro-mavros)](http://jenkins.ros.org/job/devel-hydro-mavros/)
  - ROS Indigo: [![Indigo build status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-mavros)](http://jenkins.ros.org/job/devel-indigo-mavros/)
  - Travis Hydro (PX4): [![Hydro px4 status](https://travis-ci.org/mavlink/mavros.svg?branch=master)](https://travis-ci.org/mavlink/mavros)
  - Travis Hydro (Coverity Scan): [![Hydro scan status](https://travis-ci.org/mavlink/mavros.svg?branch=coverity_scan)](https://travis-ci.org/mavlink/mavros)
    : [![Coverity Scan](https://scan.coverity.com/projects/3183/badge.svg)](https://scan.coverity.com/projects/3183)


[mrrm]: https://github.com/mavlink/mavros/blob/master/mavros/README.md
[exrm]: https://github.com/mavlink/mavros/blob/master/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/master/libmavconn/README.md
