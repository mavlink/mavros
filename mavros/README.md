MAVROS
======

MAVLink extendable communication node for ROS
with proxy for Ground Control Station (e.g. [QGroundControl][qgc]).

ROS API documentation moved to [wiki.ros.org][wiki].


Features
--------

  - Communication with autopilot via serial port, UDP or TCP (e.g. [PX4 Pro][px4] or [ArduPilot][apm])
  - Internal proxy for Ground Control Station (serial, UDP, TCP)
  - Plugin system for ROS-MAVLink translation
  - Parameter manipulation tool
  - Waypoint manipulation tool
  - PX4Flow support (by [mavros\_extras][mrext])
  - OFFBOARD mode support
  - Geographic coordinates conversions.


Limitations
-----------

Only for Linux. Depends on [Boost library][boost], GCC 4.8+ (C++11 support).
Catkin build system required.

This package are dependent on [ros-\*-mavlink][mlwiki] build from [mavlink-gbp-release][mlgbp].
It exists in ROS package index and usually updates each month.

Connection URL
--------------

Connection defined by URL, you can use any supported type for FCU and GCS.

Supported schemas:

  - Serial: `/path/to/serial/device[:baudrate]`
  - Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - Serial with hardware flow control: `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - UDP: `udp://[bind_host][:port]@[remote_host[:port]][/?ids=sysid,compid]`
  - UDP broadcast until GCS discovery: `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - UDP broadcast (permanent): `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]`
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
For translate airframe related data we simply apply rotation 180° about ROLL (X) axis.
For local we apply 180° about ROLL (X) and 90° about YAW (Z) axes.
Please read documents from issue #473 for additional information.

All the conversions are handled in `src/lib/ftf_frame_conversions.cpp` and `src/lib/ftf_quaternion_utils.cpp` and tested in `test/test_frame_conversions.cpp` and `test/test_quaternion_utils.cpp` respectively.

Related issues: [#49 (outdated)][iss49], [#216 (outdated)][iss216], [#317 (outdated)][iss317], [#319 (outdated)][iss319], [#321 (outdated)][iss321], [#473][iss473].
Documents: [Frame Conversions][iss473rfc], [Mavlink coordinate frames][iss473table].

MAVROS also allows conversion of geodetic and geocentric coordinates through [GeographicLib][geolib]
given that:
  - `geographic_msgs` and `NatSatFix.msg` require the LLA fields to be filled in WGS-84 datum,
  meaning that the altitude should be the height above the WGS-84 ellipsoid. For that, a conversion
  from the height above the geoid (AMSL, considering the egm96 geoid model) to height above the
  WGS-84 ellipsoid, and vice-versa, is available and used in several plugins;
  - According to ROS REP 105, the `earth` frame should be propagated in ECEF (Earth-Centered,
  Earth-Fixed) local coordinates. For that, the functionalities of GeographicLib are used in
  order to allow conversion from geodetic coordinates to geocentric coordinates;
  - The translation from GPS coordinates to local geocentric coordinates require the definition
  of a local origin on the `map` frame, in ECEF, and calculate the offset to it in ENU. All
  the conversions are supported by GeographicLib classes and methods and implemented in the
  `global_position` plugin.


Programs
--------

### mavros\_node -- main communication node

Main node. Allow disable GCS proxy by setting empty URL.

Run example (autopilot connected via USB at 921600 baud, GCS running on the host with IP 172.16.254.1):

    rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:921600 _gcs_url:=udp://@172.16.254.1

### gcs\_bridge -- additional proxy

Allows you to add a channel for GCS.
For example if you need to connect one GCS for HIL and the second on the tablet.

Example (SITL & QGroundControl):

    rosrun mavros mavros_node _gcs_url:='udp://:14556@172.16.254.129:14551' &
    rosrun mavros gcs_bridge _gcs_url:='udp://@172.16.254.129'




Launch Files
------------

Launch files are provided for use with common FCUs, in particular [Pixhawk](pixhawk):

  * [px4.launch](launch/px4.launch) -- for use with the PX4 Pro flight stack (for VTOL, multicopters and planes)
  * [apm.launch](launch/apm.launch) -- for use with APM flight stacks (e.g., all versions of ArduPlane, ArduCopter, etc)

Examples:

    roslaunch mavros px4.launch
    roslaunch mavros apm.launch fcu_url:=tcp://localhost gcs_url:=udp://@


Installation
------------

### Required dependencies

Most of the ROS dependencies are supported and installed by `rosdep`, including external
libraries as Eigen and Boost.

[GeographicLib][geolib] can be installed by `apt-get` and it is already included on the
rosdep of MAVROS package. It is also possible to compile it and install it from src but
be advised to have the proper install directories the same as the ones of the `apt-get`
install, in order to make sure that the `FindGeographicLib.cmake` finds the required
shared libraries (`libGeographic.so`).

Since **GeographicLib requires certain datasets** (mainly the geoid dataset) so to fulfill
certain calculations, these need to be installed manually by the user using `geographiclib-tools`,
which can be installed by `apt-get` in Debian systems. For a quicker procedure, just **run
the available script in the "mavros/scripts" folder, `install_geographiclib_datasets.sh`**.

Note that if you are using an older MAVROS release source install and want to update to a new one, remember to
run `rosdep update` before running `rosdep install --from-paths ${ROS_WORKSPACE} --ignore-src --rosdistro=${ROSDISTRO}`,
with `ROS_WORKSPACE` your src folder of catkin workspace. This will allow updating the `rosdep` list
and install the required dependencies when issuing `rosdep install`.

:bangbang: **The geoid dataset is mandatory to allow the conversion between heights in order to
respect ROS msg API. Not having the dataset available will shutdown the `mavros_node`** :bangbang:

:heavy_exclamation_mark:Run `install_geographiclib_datasets.sh` to install all datasets or
`geographiclib-datasets-download egm96_5` (*Debian 7*, *Ubuntu 14.04*, *14.10*), `geographiclib-get-geoids egm96-5`
(*Debian 8*, *Fedora 22*, *Ubuntu 15.04* or later) to install the geoid dataset only:heavy_exclamation_mark:


### Binary installation (deb)

ROS repository has binary packages for Ubuntu x86, amd64 (x86\_64) and armhf (ARMv7).
Kinetic also support Debian Jessie amd64 and arm64 (ARMv8).

Just use `apt-get` for installation:

    sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

Then install GeographicLib datasets by running the `install_geographiclib_datasets.sh` script:

    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh


### Source installation

Use `wstool` utility for retrieving sources and [`catkin` tool][catkin] for build.

NOTE: The source installation instructions are for the ROS Kinetic release.

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y

# 1. Create the workspace: unneeded if you already has workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

# 2. Install MAVLink
#    we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

# 3. Install MAVROS: get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
# alternative: latest source
# rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
# For fetching all the dependencies into your catkin_ws, just add '--deps' to the above scripts
# ex: rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall

# 4. Create workspace & deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 5. Install GeographicLib datasets:
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 6. Build source
catkin build

# 7. Make sure that you use setup.bash or setup.zsh from workspace.
#    Else rosrun can't find nodes from this workspace.
source devel/setup.bash
```

*Build error*. if you has error with missing `mavlink*` then you need fresh mavlink package.
You may update from [ros-shadow-fixed][shadow] (binary installation) or redo script steps 2 & 4.

*Note*. Since MAVLink 2.0 merged (0.18) all dialects supported by same binary.
Unfortunately overlap of v1.0 message ID's not fully handled, first loaded message forbid further changes.
Load order always:

1. common
2. ardupilotmega
3. alphabetical ordered list
4. ...

*Note*: `MAVLINK_DIALECT` not used anymore.


Troubleshooting
------------

### Error: serial0: receive: End of file
This issue should have been solve in mavros v0.23.2, it was found to be a Boost.ASIO error and should be fix in release > v1.12.0 ( >Boost 1.66).


Contributing
------------
See [CONTRIBUTING.md][contr].


Glossary
--------

  - *GCS* — Ground Control Station
  - *FCU* — Flight Control Unit (aka *FC*)
  - *OBC* — OnBoard Computer (your odroid or raspberry)


Links
-----

  - [MAVLink][ml] -- communication protocol
  - [mavlink\_ros][mlros] -- original ROS node (few messages, no proxy)
  - [Pixhawk][pixhawk] -- Reference hardware platform
  - [PX4][px4] -- Reference implementation in the academic community
  - [ArduPilot][apm] -- tested autopilot APM:Plane (default command set)
  - [QGroundControl][qgc] -- tested ground control station for Android, iOS, Mac OS, Linux and Windows
  - [mavros\_extras][mrext] -- extra plugins & node for mavros


[qgc]: http://qgroundcontrol.org/
[pixhawk]: http://pixhawk.org/
[px4]: http://px4.io/
[apm]: http://ardupilot.com/
[mlros]: https://github.com/mavlink/mavlink_ros
[boost]: http://www.boost.org/
[ml]: https://mavlink.io/en/
[mlgbp]: https://github.com/mavlink/mavlink-gbp-release
[iss35]: https://github.com/mavlink/mavros/issues/35
[iss49]: https://github.com/mavlink/mavros/issues/49
[iss216]: https://github.com/mavlink/mavros/issues/216
[iss317]: https://github.com/mavlink/mavros/issues/317
[iss319]: https://github.com/mavlink/mavros/issues/319
[iss321]: https://github.com/mavlink/mavros/issues/321
[iss473]: https://github.com/mavlink/mavros/issues/473
[iss856]: https://github.com/mavlink/mavros/issues/856
[wiki]: http://wiki.ros.org/mavros
[mrext]: https://github.com/mavlink/mavros/tree/master/mavros_extras
[mlwiki]: http://wiki.ros.org/mavlink
[shadow]: http://packages.ros.org/ros-shadow-fixed/ubuntu/pool/main/r/ros-jade-mavlink/
[catkin]: https://catkin-tools.readthedocs.org/en/latest/
[iss473rfc]: https://docs.google.com/document/d/1bDhaozrUu9F915T58WGzZeOM-McyU20dwxX-NRum1KA/edit
[iss473table]: https://docs.google.com/spreadsheets/d/1LnsWTblU92J5_SMinTvBvHJWx6sqvzFa8SKbn8TXlnU/edit#gid=0
[geolib]: https://geographiclib.sourceforge.io/
[contr]: https://github.com/mavlink/mavros/blob/master/CONTRIBUTING.md
