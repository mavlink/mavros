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
  - OFFBOARD mode support.


Limitations
-----------

Only for linux. Depends on [Boost library][boost] >= 1.46 (hydro on 12.04).
Catkin build system required (tested with ROS Hydro Medusa, Indigo Igloo and Jade Turtle).

This package are dependent on [ros-\*-mavlink][mlwiki] build from [mavlink-gbp-release][mlgbp].
Since 2014-06-19 it exists in Hydro and Indigo package index (so you can install via rosdep).
Since 2015-02-25 exists for Jade too.


Connection URL
--------------

Connection defined by URL, you can use any supported type for FCU and GCS.

Supported schemas:

  - Serial: `/path/to/serial/device[:baudrate]`
  - Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - UDP: `udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]`
  - TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
  - TCP server: `tcp-l://[bind_host][:port][/?ids=sysid,compid]`

Note:

  - Ids from URL overrides value given by system\_id & component\_id parameters.
  - bind\_host - default `0.0.0.0` - i.e. IP4 ANY
  - UDP default ports: 14555 @ 14550
  - UDP remote address updated every time with incoming packet on bind port.
  - TCP default port: 5760


Coordinate frames
-----------------

MAVROS does translate Aerospace NED frames, used in FCUs to ROS ENU frames and vice-versa.
For translate we simply apply rotation 180° abount ROLL (X) axis.

All the conversions are handled in `src/lib/uas_frame_conversions.cpp` and `src/lib/uas_quaternion_utils.cpp` and tested in `test/test_frame_conversions.cpp` and `test/test_quaternion_utils.cpp` respectively.

Related issues: [#49 (outdated)][iss49], [#216 (outdated)][iss216], [#317][iss317], [#319][iss319], [#321][iss321].


Programs
--------

### mavros\_node -- main communication node

Main node. Allow disable GCS proxy by setting empty URL.

Run example:

    rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:115200 _gcs_url:=tcp-l://


### gcs\_bridge -- additional proxy

Allows you to add a channel for GCS.
For example if you need to connect one GCS for HIL and the second on the tablet.

Previous name: `ros_udp`.

Example (HIL & DroidPlanner):

    rosrun mavros mavros_node _gcs_url:='udp://:14556@hil-host:14551' &
    rosrun mavros gcs_bridge _gcs_url:='udp://@nexus7'

<!-- scripts moved to ROS wiki -->


Launch Files
------------

Launch files are provided for use with common FCUs:

  * [px4.launch](launch/px4.launch) -- for use with the PX4 native flight stack
  * [apm.launch](launch/apm.launch) -- for use with APM flight stacks (e.g., all versions of ArduPlane, ArduCopter, etc)

Examples:

    roslaunch mavros px4.launch
    roslaunch mavros apm.launch fcu_url:=tcp://localhost gcs_url:=udp://@


Installation
------------

### Binary installation (debian)

Since v0.5 that programs available in precompiled debian packages for x86 and amd64 (x86\_64).
Also v0.9+ exists in ARMv7 repo for Ubuntu armhf.
Just use `apt-get` for installation:

    sudo apt-get install ros-jade-mavros ros-jade-mavros-extras


### Source installation

Use `wstool` utility for retriving sources and [`catkin` tool][catkin] for build.

```sh
sudo apt-get install python-catkin-tools

# 1. unneded if you already has workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

# 2. get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
# alternative: latest source
rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall

# 3. latest released mavlink package
# you may run from this line to update ros-*-mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall

# 4. workspace & deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro jade -y

# 5. finally - build
catkin build
```

*Build error*. if you has error with missing `mavlink_*_t` or `MAVLINK_MSG_ID_*` then you need fresh mavlink package.
You may update from [ros-shadow-fixed][shadow] (binary installation) or redo script from step 3.

*Important*. The current implementation of mavlink does not allow to select dialect in run-time,
so mavros package (and all plugin packages) have compile-time option `MAVLINK_DIALECT`, default is 'aurdupilotmega'.

If you want change dialect change workspace config:

    catkin config --cmake-args -DMAVLINK_DIALECT=common


Contributing
------------

1. Fork the repo:
![fork](http://s24.postimg.org/pfvt9sdv9/Fork_mavros.png)
2. Clone the repo (`git clone https://github.com/mavlink/mavros.git`);
3. Create a remote connection to your repo (`git remote add <remote_repo> git@github.com:<YourGitUser>/mavros.git`);
4. Create a feature/dev branch (`git checkout -b <feature_branch>`);
5. Add the changes;
6. Apply the changes by commiting (`git commit -m "<message>"` or `git commit -a` and then write message; if adding new files: `git add <path/to/file.ext>`);
7. Check code style `uncrustify -c ${ROS_WORKSPACE}/mavros/mavros/tools/uncrustify-cpp.cfg --replace --no-backup <path/to/file.ext>`;
8. Fix small code style errors and typos;
9. Run tests:
 - with `catkin_make`, issue `catkin_make tests` and then `catkin_make run_tests`;
 - with `catkin tools`, issue `catkin run_tests`;
10. If everything goes as planned, push the changes (`git push -u <remote_repo> <feature_branch>`) and issue a pull request.


Glossary
--------

  - *GCS* — Ground Control Station
  - *FCU* — Flight Control Unit (aka *FC*)
  - *OBC* — OnBoard Computer (your odroid or raspberry)


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
[iss216]: https://github.com/mavlink/mavros/issues/216
[iss317]: https://github.com/mavlink/mavros/issues/317
[iss319]: https://github.com/mavlink/mavros/issues/319
[iss321]: https://github.com/mavlink/mavros/issues/321
[wiki]: http://wiki.ros.org/mavros
[mrext]: https://github.com/mavlink/mavros/tree/master/mavros_extras
[mlwiki]: http://wiki.ros.org/mavlink
[shadow]: http://packages.ros.org/ros-shadow-fixed/ubuntu/pool/main/r/ros-jade-mavlink/
[catkin]: https://catkin-tools.readthedocs.org/en/latest/
