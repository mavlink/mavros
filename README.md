MAVROS
======

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

MAVLink extendable communication node for ROS.

Since 2014-08-11 this repository contains several packages.
Since 2014-11-02 hydro support splited from master to hydro-devel branch.

mavros package
--------------

It is the main package, please see it's [README][mrrm].


mavros\_extras package
----------------------

This package contain some extra nodes and plugins for mavros, please see it's [README][exrm].


libmavconn package
------------------

This package contain mavconn library, see it's [README][libmc].
MAVConn may be used outside of ROS environment.


CI Statuses
-----------

  - ROS Hydro: [![Hydro build status](http://jenkins.ros.org/buildStatus/icon?job=devel-hydro-mavros)](http://jenkins.ros.org/job/devel-hydro-mavros/)
  - ROS Indigo: [![Indigo build status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-mavros)](http://jenkins.ros.org/job/devel-indigo-mavros/)
  - Travis Hydro (PX4): [![Hydro px4 status](https://travis-ci.org/mavlink/mavros.svg?branch=master)](https://travis-ci.org/mavlink/mavros)
  - Travis Hydro (Coverity Scan): [![Hydro scan status](https://travis-ci.org/mavlink/mavros.svg?branch=coverity_scan)](https://travis-ci.org/mavlink/mavros)


[mrrm]: https://github.com/mavlink/mavros/blob/master/mavros/README.md
[exrm]: https://github.com/mavlink/mavros/blob/master/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/master/libmavconn/README.md
