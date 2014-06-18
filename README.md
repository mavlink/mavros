MAVROS
======

MAVLink extendable communication node for ROS
with UDP proxy for Ground Control Station (e.g. [QGroundControl][1]).

ROS API documentation moved to [wiki.ros.org][9].


Feutures
--------

  * Communication with autopilot via serial port (e.g. [ArduPilot][2])
  * UDP proxy for Ground Control Station
  * [mavlink\_ros][3] compatible ROS topics (Mavlink.msg)
  * Plugin system for ROS-MAVLink translation
  * Parameter manipulation tool
  * Waypoint manipulation tool


Limitations
-----------

Only for linux. Depends on [Boost library][4] >= 1.49.
Catkin build system required (tested with ROS Hydro Medusa and Indigo Igloo).

This package are dependent on ros-\*-mavlink build from [mavlink-gbp-release][7].
Since 2014-06-19 it exists in hydro and indigo package index (so you can install via rosdep).


Programs
--------

### mavros\_node -- main communication node

Main node.

Run example:

    rosrun mavros mavros_node _serial_port:=/dev/ttyACM0 _serial_baud:=115200 _gcs_host:=localhost


### ros\_udp -- additional UDP proxy

Allows you to add a UDP channel for GCS.
For example if you need to connect one GCS for HIL and the second on the tablet.

Example (HIL & DroidPlanner):

    rosrun mavros mavros_node _gcs_host:='hil-host' _gcs_port:=14556 _bind_port:=14551 &
    rosrun mavros ros_udp _gcs_host:='nexus7'


### mavparam -- parameter manipulation

Just see `--help`.

Examples:

    rosrun mavros mavparam dump /tmp/apm.param
    rosrun mavros mavparam load /tmp/apm2.param


### mavwp -- mission manipulation

See `--help`.

Examples:

    rosrun mavros mavwp show -p
    rosrun mavros dump /tmp/mission.txt


Installation
------------

Use `wstool` utility for installation. In your workspace do:

    wstool set -t src --git https://github.com/vooon/mavros.git
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro hydro -y

Then use regular `catkin_make` for biuld and install.
Notes: since v0.5 (and [#35][8]) mavlink submodule moved to special ROS 3rd party package [ros-\*-mavlink][7].

### Installing ros-\*-mavlink from source

If rosdep could'not install mavlink library, you could install it from source:

    mkdir -p ~/ros_deps/src
    cd ~/ros_deps
    rosinstall_generate mavlink | tee rosinstall.yaml
    wstool init src ./rosinstall.yaml
    catkin_make_isolated --install-space $ROSINSTALL --install -DCMAKE_BUILD_TYPE=Release

$ROSINSTALL must be writable for user.
Or you could build debian package by pulling right bloom branch from [mavlink-gbp-release][7].


Testing
-------

Simple communication library test (serial-udp bridge):

    catkin_make mavudpproxy && ./devel/lib/mavros/mavudpproxy /dev/ttyACM0 115200


Links
-----

  * [MAVLink][5] -- communication protocol
  * [mavlink\_ros][3] -- original ROS node (few messages, no proxy)
  * [ArduPilot][2] -- tested autopilot APM:Plane (default command set)
  * [QGroundControl][1] -- tested ground control station for linux
  * [DroidPlanner][6] -- tested GCS for Android


[1]: http://qgroundcontrol.org/
[2]: http://ardupilot.com/
[3]: https://github.com/mavlink/mavlink_ros
[4]: http://www.boost.org/
[5]: http://mavlink.org/mavlink/start
[6]: https://github.com/arthurbenemann/droidplanner/
[7]: https://github.com/vooon/mavlink-gbp-release
[8]: https://github.com/vooon/mavros/issues/35
[9]: http://wiki.ros.org/mavros
