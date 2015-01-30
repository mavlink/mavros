MAVROS
======

MAVLink extendable communication node for ROS
with proxy for Ground Control Station (e.g. [QGroundControl][qgc]).

ROS API documentation moved to [wiki.ros.org][wiki].


Features
--------

  - Communication with autopilot via serial port, UDP or TCP (e.g. [ArduPilot][apm])
  - Internal proxy for Ground Control Station (serial, UDP, TCP)
  - [mavlink\_ros][mlros] compatible ROS topics (Mavlink.msg)
  - Plugin system for ROS-MAVLink translation
  - Parameter manipulation tool
  - Waypoint manipulation tool
  - PX4Flow support (by [mavros\_extras][mrext])


Limitations
-----------

Only for linux. Depends on [Boost library][boost] >= 1.46 (hydro on 12.04).
Catkin build system required (tested with ROS Hydro Medusa and Indigo Igloo).

This package are dependent on [ros-\*-mavlink][mlwiki] build from [mavlink-gbp-release][mlgbp].
Since 2014-06-19 it exists in hydro and indigo package index (so you can install via rosdep).


Connection URL
--------------

*New in 0.7.0*. Connection now defined by URL,
you can use any supported type for FCU and GCS.

Supported schemas:

  - Serial: `/path/to/serial/device[:baudrate]`
  - Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - UDP: `udp://[bind_host[:port]]@[remote_host[:port]][/?ids=sysid,compid]`
  - TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
  - TCP server: `tcp-l://[bind_port][:port][/?ids=sysid,compid]`

Note: ids from URL overrides ids given by system\_id & component\_id parameters.


Coordinate frames
-----------------

MAVROS does translate Aerospace NED frames, used in FCU's to ROS ENU frames.
Rules descrided in [issue #49][iss49].


Programs
--------

### mavros\_node -- main communication node

Main node. Allow disable GCS proxy by setting empty URL.

Run example:

    rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:115200 _gcs_url:=tcp-l://


### gcs\_bridge -- additional UDP proxy

Allows you to add a channel for GCS.
For example if you need to connect one GCS for HIL and the second on the tablet.

Previous name: `ros_udp`.

Example (HIL & DroidPlanner):

    rosrun mavros mavros_node _gcs_url:='udp://:14556@hil-host:14551' &
    rosrun mavros gcs_bridge _gcs_url:='udp://@nexus7'


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


### mavsafety -- safety tool

See `--help`.

Examples:

    rosrun mavros mavsafety arm
    rosrun mavros mavsafety disarm


### mavcmd -- commander tool

See `--help`.

Examples:

    rosrun mavros mavcmd takeoff 20 15 0 0 50
    rosrun mavros mavcmd sethome --current-gps 0 0 0


Installation
------------

### Binary installation (debian)

Since v0.5 that programs available in precompiled debian packages for x86 and amd64 (x86\_64).
Just use `apt-get` for installation:

    sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras


### Source installation

Use `wstool` utility for installation. In your workspace do:

    wstool init src (if not already initialized)
    wstool set -t src mavros --git https://github.com/mavlink/mavros.git
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y

Then use regular `catkin_make` for build and install.
Notes:
  - since v0.5 (and [#35][iss35]) mavlink submodule moved to special ROS 3rd party package [ros-\*-mavlink][mlwiki].
  - since 2014-11-02 hydro support splitted to branch hydro-devel, add `--version hydro-devel` to wstool set.

*Important*. The current implementation of mavlink does not allow to select dialect in run-time,
so mavros package (and all plugin packages) have compile-time option `MAVLINK_DIALECT`, default is 'aurdupilotmega'.

If you want change dialect you can:

1. Add cmake definition to catkin: `catkin_make -DMAVLINK_DIALECT=pixhawk`
2. Edit configuration by `catkin_make edit_cache`
3. Use `cmake-gui build`, better: it creates drop-down list with all available dialects
   plus it will be used in next `catkin_make edit_cache`.
   Ubuntu: `sudo apt-get install cmake-qt-gui`


### Installing ros-\*-mavlink from source

If rosdep could not install mavlink library, you could install it from source:

    mkdir -p ~/ros_deps/src
    cd ~/ros_deps
    rosinstall_generator mavlink | tee rosinstall.yaml
    wstool init src ./rosinstall.yaml
    catkin_make_isolated --install-space $ROSINSTALL --install -DCMAKE_BUILD_TYPE=Release

$ROSINSTALL must be writable for user or you can add `sudo -s` to last command.
Or you could build debian package by pulling right bloom branch from [mavlink-gbp-release][mlgbp]
(common naming: `debian/<rosdistro>/<osdistro>/<package>`) using `dh binary`.


Links
-----

  - [MAVLink][ml] -- communication protocol
  - [mavlink\_ros][mlros] -- original ROS node (few messages, no proxy)
  - [ArduPilot][apm] -- tested autopilot APM:Plane (default command set)
  - [QGroundControl][qgc] -- tested ground control station for linux
  - [DroidPlanner][dp] -- tested GCS for Android
  - [mavros\_extras][mrext] -- extra plugins & node for mavros


[qgc]: http://qgroundcontrol.org/
[apm]: http://ardupilot.com/
[mlros]: https://github.com/mavlink/mavlink_ros
[boost]: http://www.boost.org/
[ml]: http://mavlink.org/mavlink/start
[dp]: https://github.com/arthurbenemann/droidplanner/
[mlgbp]: https://github.com/mavlink/mavlink-gbp-release
[iss35]: https://github.com/mavlink/mavros/issues/35
[iss49]: https://github.com/mavlink/mavros/issues/49
[wiki]: http://wiki.ros.org/mavros
[mrext]: https://github.com/mavlink/mavros/tree/master/mavros_extras
[mlwiki]: http://wiki.ros.org/mavlink
