^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2021-05-05)
------------------
* lib: ftf: allow both Quaterniond and Quaternionf for quaternion_to_mavlink()
* extras: distance_sensor: rename param for custom orientation, apply uncrustify
* px4_config: Add distance_sensor parameters
* convert whole expression to mm
* Contributors: Alexey Rogachevskiy, Thomas, Vladimir Ermakov

1.7.1 (2021-04-05)
------------------
* re-generate all pymavlink enums
* Contributors: Vladimir Ermakov

1.7.0 (2021-04-05)
------------------
* lib: re-generate the code
* plugins: mission: re-generate the code
* MissionBase: correction to file information
* MissionBase: add copyright from origional waypoint.cpp
* uncrustify
* whitespace
* add rallypoint and geofence plugins to mavros plugins xml
* add rallypoint and geofence plugins to CMakeList
* Geofence: add geofence plugin
* Rallypoint: add rallypoint plugin
* Waypoint: inherit MissionBase class for mission protocol
* MissionBase: breakout mission protocol from waypoint.cpp
* README: Update PX4 Autopilot references
  Much needed fixes to clarify the project is named correctly throughout the README
  for the PX4 Autopilot, QGroundControl, and MAVLink
* Fix https://github.com/mavlink/mavros/issues/849
* Contributors: Charlie-Burge, Ramon Roche, Tobias Fischer, Vladimir Ermakov

1.6.0 (2021-02-15)
------------------
* fix inconsistency in direction of yaw when using set_position in BODY frames and fix problems with yaw in setponit_raw
* Contributors: zhouzhiwen2000

1.5.2 (2021-02-02)
------------------
* readme: add source install note for Noetic release
* Contributors: Vladimir Ermakov

1.5.1 (2021-01-04)
------------------
* Fix tests for renaming of ECEF cases
  Introduced in 6234af29
* Initialise message structures
  Uninitialised Mavlink 2 extension fields were sent if the fields were
  not later set. Initialising the fields to zero is the default value for
  extension fields and appears to the receiver as though sender is unaware
  of Mavlink 2.
  Instances were found with regex below, more may exist:
  mavlink::[^:]+::msg::[^:={]+ ?[^:={]*;
* Contributors: Rob Clarke

1.5.0 (2020-11-11)
------------------
* mavros/sys_status: Fill flight_custom_version field
* mavros: Add override specifiers
* mavros: Move ECEF tf enums to separate enum class
  This avoids a bunch of unhandled switch cases, and should
  improve type safety a bit.
* Contributors: Morten Fyhn Amundsen

1.4.0 (2020-09-11)
------------------
* mavros: use mavlink::minimal:: after incompatible changes in mavlink package
  Incompatible change: https://github.com/mavlink/mavlink/pull/1463
  Fix: `#1483 <https://github.com/mavlink/mavros/issues/1483>`_, https://github.com/mavlink/mavlink/issues/1474
* fixes based on vooon's review
* fix issue what we couldn't set real parameters to 0.0 in mavros
* Add error message
* Fixed compilation error: publish std_msgs::String, not std::string for gcs_ip
* Dispatch GCS IP address
* Contributors: Artem Batalov, Marcelino, Morten Fyhn Amundsen, Vladimir Ermakov, Øystein Skotheim

1.3.0 (2020-08-08)
------------------
* fake_gps.cpp: implement speed accuracy
* fake_gps.cpp: Add mocap_withcovariance configuration parameter
* fake_gps.cpp: add initial support for GPS_INPUT MAVLink message
* apm.launch: Avoid warning:
  Warning: You are using <arg> inside an <include> tag with the default=XY attribute - which is superfluous.
  Use value=XY instead for less confusion.
  Attribute name: respawn_mavros
* Added support for MavProxy parameter file format
* Ignore read-only parameters and statistics parameters in push operations
* fix indentation
* transform based on coordinate_frame
* wind plugin: fix ArduPilot wind transformation
* Contributors: Ben Wolsieffer, Dr.-Ing. Amilcar do Carmo Lucas, Yuan, Yuan Xu

1.2.0 (2020-05-22)
------------------
* has_capability only works for enums
* Uncrustify
* Reworked Waypoint plugin to use capabilities_cb
  Additionally added helper functions has_capability and has_capabilities
  so that people can use either ints or enums to check if the UAS has a
  capability. This might make accepting capabilities as a parameter moot
  though.
* Added alias for capabilities enum to UAS
* Added alias for capabilities enum to UAS
* Added a capabilities change cb queue
  Plugins can now write functions that they add to the
  capabilities_cb_vec. These functions will be called only when there is a
  change to the capabilities themselves not whenever the known status of
  the fcu_capabilities change.
  These functions should have a parameter of type
  mavlink::common::MAV_PROTOCOL_CAPABILITY which is essentially just a
  uint64_t however being more opinionated is helpful when looking for what
  the canonical enum names are in the mavlink project header files.
* Uncrustify
* Fixed Removed Uncrustify Option
  I'm not sure why this didn't break when I ran uncrustify previously but
  it seems that the align_number_left option was removed a while ago with
  this merge request but I may be mistaken
  https://github.com/uncrustify/uncrustify/pull/1393
  I replaced it which align_number_right=true since it seems to be the
  inverse of align_number_left=true.
* Removed deprecated MAV_FRAME values
* Removed use of variant in favor of templates
  Since ROS messages are now the storage type in the node, providing to
  and from conversion functions is sufficient and can be better expressed
  with function templates.
* Encode factor returns double
* Changed encoding factor cog code
* Uncrustify changes
* Added new parameter to config.yamls
* Updated waypoint plugin to support MISSION_ITEM_INT
  These changes add a new parameter use_mission_item_int, which allows
  users to prefer the old behavior. These changes also verify that the
  flight controller supports _INT messages since APM only sends
  REQUEST_ITEM messages even though it accepts _INT items back.
  This commit is functional and tested with the APM stack only.
  PX4 sitl jmavsim threw:
  WP: upload failed: Command is not supported.
  FCU: IGN MISSION_ITEM: Busy
* Removed x_lat, y_long, z_alt from WP
  These values seemed to be used due to the fact that double had
  a greater resolution than float and doubles are used in the
  ros msg. However they were only ever used for printing. Since
  the int version of these messages has a greater resolution I
  figure it is more useful to print the true value in the mavlink
  message rather than the ros message value
* Replaced MISSION_ITEM
* add yaw to CMD_DO_SET_HOME
* fix local angular velocity
* Contributors: Braedon, David Jablonski, Martina Rivizzigno

1.1.0 (2020-04-04)
------------------
* fixed styling
* fixed indent from using spaces
* updates apmrover2 modes and allows for arduboat mode changes
* mavsafety kill feature for emergency stop
* Include trajectory_msgs in CMakeLists.txt
  This allows build to complete successfully.
* Contributors: Anthony Goeckner, Matt Koos, aykutkabaoglu

1.0.0 (2020-01-01)
------------------

0.33.4 (2019-12-12)
-------------------
* Replaced estimator status hardcoded definition with cog.
* Refactor.
* Replaced bool with git add -u as already done.
* Added a publisher for estimator status message received from mavlink in sys_status.
* Contributors: saifullah3396

0.33.3 (2019-11-13)
-------------------
* package: fix 6fa58e59 - main package depends on trajectory_msgs, not extras
* Contributors: Vladimir Ermakov

0.33.2 (2019-11-13)
-------------------

0.33.1 (2019-11-11)
-------------------
* Add mutex
* Initialize type mask
* Handle frame with StaticTF
* Handle different frames
* Set yaw rate from message inputs
* Add setpoint trajectory reset interface
* Fix trajectory timestamp
* Address comments
* Pass reference with oneshot timers
* Set typemasks correctly
* Address more style comments
* Address style comments
* Visualize desired trajectory
* Handle end of trajectory correctly
* Remove message handlers
* Add setpoint_trajectory plugin template
* resolved merge conflict
* Contributors: David Jablonski, Jaeyoung-Lim

0.33.0 (2019-10-10)
-------------------
* Add vtol transition service
* CleanUp
* Update frame name in px4_config to match ROS standards
* Enable publishing multiple static tfs at once, publish standard static tfs
* moving ACK_TIMEOUT_DEFAULT out of class
* cog: Update all generated code
* mavros/src/plugins/command.cpp: one more style fix
* mavros/src/plugins/command.cpp: style fixes
* mavros/src/plugins/command.cpp: command_ack_timeout ms -> s
* mavros/src/plugins/command.cpp: command_ack_timeout_ms int -> double
* mavros/src/plugins/command.cpp: uncrustify
* mavros/src/plugins/command.cpp: parameter for command's ack timeout
  Sometimes commands take more time than default 5 seconds. Due to a low bandwidth
  of UART and a high rate of some mavlink streams. To eliminate this problem it's
  better to provide the parameter to configure the command's ack timeout.
* added manual flag to mavros/state
* Use GeoPoseStamped messages
* Fix build
* Add callback for SET_POSITION_TARGET_GLBOAL_INT
* Contributors: David Jablonski, Jaeyoung-Lim, Sergei Zobov, Vladimir Ermakov, kamilritz

0.32.2 (2019-09-09)
-------------------
* uncrustify
* Add boolean to check if IMU data has been received
  Follow sensor_msgs/Imu convention when data not present
* Uncrustify the GPS_GLOBAL_ORIGIN handler in global_position
* Fix global origin conversion to ecef (was using amsl where hae was required)
  Summary: Fix global origin conversion to ecef (was using amsl where hae was required)
* moved code to end of function
* added amount of satellites to global_position/raw/
* Contributors: David Jablonski, Nick Steele, Rob Clarke, Robert Clarke

0.32.1 (2019-08-08)
-------------------
* uncrustify
* Removed tf loop
* made small edit to handle augmented gps fix
* added a check for gps fix before setting origin for global_position/local odometry topic
* Contributors: Eric, Lucas Hill

0.32.0 (2019-07-06)
-------------------
* use direclty radians in yaml files
* A simple typo error has fixed. (`#1260 <https://github.com/mavlink/mavros/issues/1260>`_)
  * fix: a typing error "alredy" to "already"
  * Fix: typo error (helth -> health)
* Contributors: Martina Rivizzigno, 강정석

0.31.0 (2019-06-07)
-------------------
* readme: fix udp-pb formatting
* launch config: landing_target: fix and improve parameter list
* remove duplicated landing_target parameters
* enum_to_string: simplify landing_target_type_from_str
* enum_to_string: update enumerations and checksum
* extras: landing target: improve usability and flexibility
* remove landing_target from blacklist
* update to use pymavlink generator
* px4_config: landing_target: minor correction
* mav_frame: add frames of reference to wiki page; reference them on config
* landing_target: removed child_frame_id
* landing_target: minor code tweak/restructure
* landing_target: uncrustify code
* landing_target: updated to TF2 and Eigen math
* landing_target: adapted to latest master code
* landing_target: added timestamp and target size fields [!Won't compile unless a new mavlink release!]
* landing_target: first commit
* Switch to double-reflections instead of axes-reassignments
* specialize transform_frame_ned_enu and transform_frame_enu_ned for type
  Vector3d such that input vectors containing a NAN can be correctly transformed
* Update README.md
  update misspelling
* Contributors: Julian Kent, Martina Rivizzigno, Shingo Matsuura, TSC21, Vladimir Ermakov

0.30.0 (2019-05-20)
-------------------
* Filter heartbeats by component id as well
  This addresses `#1107 <https://github.com/mavlink/mavros/issues/1107>`_ and `#1227 <https://github.com/mavlink/mavros/issues/1227>`_, by filtering incoming heartbeats
  by component ids before publishing the state.
* mavros/src/plugins/command.cpp: log if command's wait ack timeout (`#1222 <https://github.com/mavlink/mavros/issues/1222>`_)
  * mavros/src/plugins/command.cpp: log if command's wait ack timeout
  * mavros/src/plugins/command.cpp: log timeout in wait_ack_for
* local_position fix `#1220 <https://github.com/mavlink/mavros/issues/1220>`_: initialize flags
* plugin waypoint: fix spelling
* Fix leading space before setpoint_raw
  This causes an error when running `roslaunch`:
  ```
  error loading <rosparam> tag:
  file /opt/ros/kinetic/share/mavros/launch/apm_config.yaml contains invalid YAML:
  while parsing a block mapping
  in "<string>", line 4, column 1:
  startup_px4_usb_quirk: false
  ^
  expected <block end>, but found '<block mapping start>'
  in "<string>", line 103, column 2:
  setpoint_raw:
  ^
  XML is <rosparam command="load" file="$(arg config_yaml)"/>
  The traceback for the exception was written to the log file
  ```
* global_position.cpp: spell in comment
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Josh Veitch-Michaelis, Nico van Duijn, Sergey Zobov, Vladimir Ermakov

0.29.2 (2019-03-06)
-------------------

0.29.1 (2019-03-03)
-------------------
* All: catkin lint files
* Update apm_config.yaml
  Setting thrust_scaling in the setpoint_raw message (in my case, to use /mavros/setpoint_raw/attitude)
  Without it, when using Gazebo, get the following problem
  "Recieved thrust, but ignore_thrust is true: the most likely cause of this is a failure to specify the thrust_scaling parameters on px4/apm_config.yaml. Actuation will be ignored." from the function void attitude_cb in setpoint_raw.cpp (http://docs.ros.org/kinetic/api/mavros/html/setpoint__raw_8cpp_source.html)
* cmake: fix `#1174 <https://github.com/mavlink/mavros/issues/1174>`_: add msg deps for package format 2
* Issue `#1174 <https://github.com/mavlink/mavros/issues/1174>`_ Added dependency for mavros_msgs and mavros
* Contributors: Adam Watkins, KiloNovemberDelta, Pierre Kancir, Vladimir Ermakov

0.29.0 (2019-02-02)
-------------------
* Fix broken documentation URLs
* px4_config: set the thrust_scaling to one by default
* local_position: add an aditional topic for velocity on the local frame
* Merge pull request `#1136 <https://github.com/mavlink/mavros/issues/1136>`_ from angri/param-timeout
  Request timed up parameters as soon as possible
* Merge branch 'master' into param-timeout
* plugin:param added logging regarding rerequests
* plugin:param fixed second and consequent timeouts in requesting list
* mavros_extras: Wheel odometry plugin updated according to the final mavlink WHEEL_DISTANCE message.
* mavros_extras: Wheel odometry plugin fixes after CR.
* mavros_extras: Wheel odometry plugin added.
* mavsys: add do_message_interval
* sys_status: add set_message_interval service
* lib: fix MAV_COMPONENT to_string
* lib: update sensor orientations
* plugin:param rerequest timed out parameters asap
  Avoid vaiting for the next timeout
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Pavlo Kolomiiets, Randy Mackay, TSC21, Vladimir Ermakov, angri

0.28.0 (2019-01-03)
-------------------
* plugin:param: publish new param value
* Merge pull request `#1148 <https://github.com/mavlink/mavros/issues/1148>`_ from Kiwa21/pr-param-value
  param plugin : add msg and publisher to catch latest param value
* sys_status: fix build
* sys_state: Small cleanup of `#1150 <https://github.com/mavlink/mavros/issues/1150>`_
* VehicleInfo : add srv into sys_status plugin to request basic info from vehicle
* sys_status: Fix `#1151 <https://github.com/mavlink/mavros/issues/1151>`_ bug - incorrect hex print
* plugins:sys_status: Update diag decoder
* frame_tf: mavlink_urt_to_covariance_matrix: make matrix symetrical
* uas_data: add comment on the reverse tf fcu_frd->fcu
* odom: add ODOMETRY handler and publisher
* Handle LOCAL_POSITION_NED_COV messages, add pose_cov, velocity_cov, accel topics
* sys_status : add MAV_TYPE as a parameter
* rc_io: extend handle_servo_output_raw to 16 channels
* param plugin : add msg and publisher to catch latest param value
* plugin:command: Update for C++11, style fix
  Signed-off-by: Vladimir Ermakov <vooon341@gmail.com>
* Fixed NavSatFix bug in mavcmd takeoffcur and landcur
* Fix mavros/param.py to work in python2 and python3, `#940 <https://github.com/mavlink/mavros/issues/940>`_
  Simplify python3 fixes, `#940 <https://github.com/mavlink/mavros/issues/940>`_
  Remove unnecessary functools
* Fix mavros/param.py to work in python2 and python3, `#940 <https://github.com/mavlink/mavros/issues/940>`_
  Simplify python3 fixes, `#940 <https://github.com/mavlink/mavros/issues/940>`_
* Fix mavros/param.py to work in python2 and python3, `#940 <https://github.com/mavlink/mavros/issues/940>`_
* correct the to_string function
* set value back to 30
* add autogenerated to_string function
* style clean up
* Use component_id to determine message sender
* change message name from COMPANION_STATUS to COMPANION_PROCESS_STATUS
* change message to include pid
* Change from specific avoidance status message to a more generic companion status message
* add plugin to receive avoidance status message
* Added RPYrT and uncrustified.
  Pushing version without spaces.
  Version with tabs?
  Fixed all?
  Finally fixed.
  Fixed requestes by @vooon
  Fixed a def.
  Fixed log format.
  Fixed time for log.
* apm_config: enable timesync and system for ardupilot
* Contributors: Dan Nix, Gregoire Linard, Oleg Kalachev, Randy Mackay, TSC21, Vladimir Ermakov, baumanta, fnoop, pedro-roque

0.27.0 (2018-11-12)
-------------------
* fix: a typing error "alredy" to "already"
* plugins `#1110 <https://github.com/mavlink/mavros/issues/1110>`_ `#1111 <https://github.com/mavlink/mavros/issues/1111>`_: add eigen aligment to plugins with eigen-typed members
* plugins: fix style
* with this fix ,it will avoid eigen error on 32 bits system
* Add service to send mavlink TRIGG_INTERVAL commands
  Adapt trigger_control service to current mavlink cmd spec. Add a new service to change trigger interval and integration time
* launch: fix `#1080 <https://github.com/mavlink/mavros/issues/1080>`_: APM now support mocap messages
* Contributors: Gaogeolone, Moritz Zimmermann, Vladimir Ermakov, rapsealk

0.26.3 (2018-08-21)
-------------------
* test: Fix sensor orientation. RPY 315 was removed in recent mavlink.
  https://github.com/mavlink/mavlink/commit/3d94bccfedc5fc7f2ffad247adecff0c2dc03501
* lib: update generated entries
* Contributors: Vladimir Ermakov

0.26.2 (2018-08-08)
-------------------
* Moving gps_rtk to mavros_extras
* Update copyright name
* Updating the gps_rtk plugin to fit mavros guidelines:
  - Updating max_frag_len to allow changes in size in MAVLink seamlessly
  - Using std::copy instead of memset
  - Zero fill with std::fill
  - Preapply the sequence flags
  - Use of std iterators
  - Add the maximal data size in the mavros_msgs
* uncrustify
* Update comments for the renaming
* Renaming the GPS RTK module, Adding fragmentation, Changing the RTCM message
* RTK Plugin; to forward RTCM messages
  Signed-off-by: Alexis Paques <alexis.paques@gmail.com>
* Contributors: Alexis Paques

0.26.1 (2018-07-19)
-------------------
* setpoint_velocity: fix yaw rate setpoint rotation
* lib fix `#1051 <https://github.com/mavlink/mavros/issues/1051>`_: Add APM BOAT modes support.
  Currently SURFACE_BOAT uses same code as Rover2,
  just different vehicle type.
* Contributors: TSC21, Vladimir Ermakov

0.26.0 (2018-06-06)
-------------------
* lib: add tunable timeout to gcs_quiet_mode
* udp bridge: pass only HEARTBEATs when GCS is offline
* sys_time : add advanced timesync algorithm
* libmavconn: add scheme for permanent UDP broadcasting
* GPS accuracy wo approximations (`#1034 <https://github.com/mavlink/mavros/issues/1034>`_)
  * GPS horizontal and vertical accuracy are based now on h_acc, v_acc of GPS_RAW_INT.
  * GPS horizontal and vertical accuracy are based now on h_acc, v_acc of GPS_RAW_INT if on mavlink v2.0,
  or on DOP values otherwise.
  * GPS accuracy update.
* Contributors: Mohammed Kabir, Oleg Kalachev, Pavlo Kolomiiets, Vladimir Ermakov

0.25.1 (2018-05-14)
-------------------

0.25.0 (2018-05-11)
-------------------
* wind plugin: uncrustify
* use eigen and tf conversions (fix conventions), sync timestamp, fix typos
* add wind estimation plugin
* launch: fix style and keep apm.launch consistent with px4.launch
* Updated apm.launch to forward new fcu_protocol parameter
* glob pos plugin: correct gps velocity convention (NEU->ENU)
* Split temperature publisher.
* setpoint_raw: correct yaw transform; remove yaw transform methods
* extras: odom: improve way frame naming is handled
* extras: update odom plugin to send ODOMETRY msgs
* lib: enum_to_string: update enums
* setpoint_attitude: rename topic from target_attitude to attitude
* imu plugin: fix pressure units
* imu plugin: publish differential pressure (`#1001 <https://github.com/mavlink/mavros/issues/1001>`_)
  * imu plugin: publish differential pressure
  * imu plugin: fix doxygen snippets
* lib: add PX4 mode AUTO.PRECLAND
* extras: add covariance parsing to vision_speed_estimate (`#996 <https://github.com/mavlink/mavros/issues/996>`_)
* Contributors: Anthony Lamping, Nuno Marques, Oleg Kalachev, Sondre Engebråten, TSC21, Thomas Stastny, Timo Hinzmann, Vladimir Ermakov

0.24.0 (2018-04-05)
-------------------
* frame_tf: add assertion over size of covariance matrix URT
* extras: update vision_pose_estimate plugin so it can send the covariance matrix also
* plugins fix `#990 <https://github.com/mavlink/mavros/issues/990>`_: Explicitly cast boolean values. Else someone can shoot in his foot.
* Update Readme for serial0: receive: End of file
* launch : remove vision_pose_estimate from blacklist on ardupilot
* plugin: ftp: fix typo
* Add ability to send STATUSTEXT messages
* Contributors: Anass Al, Andrei Korigodski, Pierre Kancir, TSC21, Vladimir Ermakov

0.23.3 (2018-03-09)
-------------------
* lib: simplify geolib cmake module, try to fix CI
* Contributors: Vladimir Ermakov

0.23.2 (2018-03-07)
-------------------
* launch: add optional respawn_mavros arg
* Contributors: Anthony Lamping

0.23.1 (2018-02-27)
-------------------
* lib: Update to_string
* plugin fix `#957 <https://github.com/mavlink/mavros/issues/957>`_: set MISSION_ITEM::mission_type
* Contributors: Vladimir Ermakov

0.23.0 (2018-02-03)
-------------------
* launch fix `#935 <https://github.com/mavlink/mavros/issues/935>`_: use orientation convention from message descr
  https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
* Blacklist HIL for APM since it is not relevent
* add MAV_DISTANCE_SENSOR enum to_string
* px4: add fcu_protocol argument to choose mavlink v1.0 or v2.0 to start
  mavros in node.launch
* node: add fcu_protocol parameter to be able to choose mavlink v1.0 or v2.0
  when starting mavros node
* mavros: default fcu_protocol parameter to mavlink v2.0
* manual_control: `send` topic for sending MANUAL_CONTROL message to FCU
* imu plugin: fix doxygen comments
* imu plugin: change sufixes to match the body coordinate frame
* Fix vision odom.
* IMU plugin: add raw IMU conversion for PX4
* mention rotation convention and fix NED to ENU description
* Contributors: ChristophTobler, James Goppert, James Mare, Martina, Oleg Kalachev, TSC21, Vladimir Ermakov

0.22.0 (2017-12-11)
-------------------
* scripts: Use non global mavros-ns allow to work __ns parameter
* update script to support cycle_time on cmd trigger_control
* plugin: Fix setpoint_position code style
* Global position setpoint plugin (`#764 <https://github.com/mavlink/mavros/issues/764>`_)
  * fix fake gps rate
  * fix
  * fix plugin_list
  * fix
  * add global position setpoint plugin
  * add plugin to CMakeList
  * fix bugs
  * add altitude
  * move GPS setpoints to setpoint_position plugin
  * fix gps setpoint subscriber name
  * move  GeographicLib::Geocentric earth inside callback
  * add warning msg if timestamp is not updates
  * Fix ROS_WARN
* doc: move contributing.md to root
* tools: add cogall.sh
* split contribuion guide to GH file
* Readme: add help for cog (`#876 <https://github.com/mavlink/mavros/issues/876>`_)
* Setpoints: remove mav_frame string for local variable
* Setpoints: add params for initial frame
* Setpoint_velocity: uncrustify
* Setpoint_position: uncrustify
* Setpoints: add service to specify frame
* Fix typo `#867 <https://github.com/mavlink/mavros/issues/867>`_
* Improve output of script, replace which with more reliable hash `#867 <https://github.com/mavlink/mavros/issues/867>`_
* Ensure dataset files exist, not just directories `#867 <https://github.com/mavlink/mavros/issues/867>`_
* Remove previous duplicated link
* Fixed issue link.
* Fixed section header. Ready for troubleshooting PR.
* Pushing troubleshooting section for Mavros.
* Contributors: Mohamed Abdelkader Zahana, Pierre Kancir, Vladimir Ermakov, andresR8, fnoop, khancyr, pedro-roque

0.21.5 (2017-11-16)
-------------------
* Yet another formatting.
* px4_config.yaml updated. Minor formatting update.
* global_position/raw/gps_vel should still be in earth fixed frame.
* GPS fix's frame_id changed to body-fixed.
* global_position/local angular twist changed from NANs to zeroes to be able to show in RViz.
* readme: source install: add note on fetching all the deps
* geolib_dataset: script: fix interpreter
* Contributors: Pavlo Kolomiiets, TSC21

0.21.4 (2017-11-01)
-------------------
* lib ftf: update dox, uncrustify
* ENU<->ECEF transforms fix. (`#847 <https://github.com/mavlink/mavros/issues/847>`_)
  * ENU<->ECEF transforms fix.
  * Changes after review. Unit tests added.
* test: fix copy-paste error in frame_tf
* Contributors: Vladimir Ermakov, pavloblindnology

0.21.3 (2017-10-28)
-------------------
* Update geographiclib script to work with zsh
* scripts: fix typos and improve help messages consistency
  commad -> command
  safty -> safety
  Start help messages with a capital letter.
* uncrustify
* plugin waypoints: Use stamped message
* plugin waypoint: Add MISSION_ITEM_REACHED publisher
  * Changes to be committed:
  modified:   mavros/src/plugins/waypoint.cpp
  modified:   mavros_msgs/CMakeLists.txt
  new file:   mavros_msgs/srv/WaypointReached.srv
  * change reached service name to classic topic
  * Changed reached service to topic
  * removed unused file
  * Removed WaypointReached service
  * Change reached message type to std_msgs::UInt16
  * Delete WaypointReached.srv
  * Restore WaypointPush.srv
  * Fix tipo
  * Update waypoint.cpp
* launch: sync APM and PX4 configs
* add debug plugin
* Contributors: Jonas Vautherin, Patrick Jose Pereira, TSC21, Vladimir Ermakov, gui2dev

0.21.2 (2017-09-25)
-------------------
* plugin: setpoint_attitude: Finish Andres fix
* fix: attitude callback trigger
* lib uas: remove inline on not inlined method
* odom: general fixes and code tighting
* Use tf2 for odom plugin and set reasoable defaults for local pos cov.
* Contributors: Andres Rengifo, James Goppert, TSC21, Vladimir Ermakov

0.21.1 (2017-09-22)
-------------------
* mavsys: mode: add solutions for setting AUTO.MISSION and AUTO.LOITER modes (`#814 <https://github.com/mavlink/mavros/issues/814>`_)
  * mavsys: add notes on how to change mode to AUTO.MISSION on PX4 Pro
  * enum_to_string: update enums
  * mavsys: mode: move AUTO submodes info to argparser
  * sys_status: leave note that MAV_TYPE_ONBOARD_CONTROLLER will be supported on PX4
  * mavsys: mode: add note on changing to AUTO.LOITER
* Solve the subscriber initialization
* lib frame_tf: Add to_eigen() helper
* Contributors: Alexis Paques, Nuno Marques, Vladimir Ermakov

0.21.0 (2017-09-14)
-------------------
* plugin waypoint: Uncrustify, update init list
* lib: Add to_sting for MAV_MISSION_RESULT
* plugin waypoint: Rename current seq in wp list message
* waypoint: Publish current waypoint seq
* waypoint partial: Check parameter first with hasParam
* waypoint partial: Documentation updates
* waypoint: Document mid level helpers and fix indenting on rx handlers
* waypoint: Document rx handlers
* waypoint partial: Move FCU detection to connection_cb
* waypoint partial: recommended changes to mavwp
* waypoint partial: code style cleanup
* waypoint partial: enable only on apm but allow override with parameter
* waypoint partial: Handle case when partial push is out of range with local list and uncrustify
* waypoint partial: enable only on apm through yaml
* waypoint partial: stopped partial push from clearing parts of local waypoint copy
* waypoint partial: uncrustify
* waypoint partial: extend mavwp cli tool to do partial updating in push
* waypoint partial: extended push in waypoint plugin to implement push partial
* waypoint: uncrustify
* waypoint: handle invalid_sequence mission_ack to prevent TXWP failure
* Partial waypoint: added wp_transfered to push partial service response
* Partial waypoint: renamed mavwp partial load arguments for consistency
* Partial waypoint: fixed end index and added partial tx state
* Partial Waypoint: handle service call in waypoint plugin
* Partial waypoint: added partial updating to mavwp
* imu_plugin: remove documentation of override func
* imu plugin: uncrustify
* imu plugin: don't be so explicit about in/out params
* imu plugin: fix indentation
* imu plugin: update setup_covariance method to use Eigen capabilities
* imu plugin: use simpler format for one line comments
* imu plugin: add code snippets to Doxygen documentation
* IMU and attitude: general clean-up
* CMake: explicitly link the atomic library (`#797 <https://github.com/mavlink/mavros/issues/797>`_)
  For arm & mips architecture, the linker must explicitly be asked to
  link the atomic library (with `-latomic`).
  Otherwise, the linking fails with:
  ```
  | devel/lib/libmavros.so: undefined reference to `__atomic_load_8'
  | devel/lib/libmavros.so: undefined reference to `__atomic_store_8'
  | collect2: error: ld returned 1 exit status
  ```
  Linking `atomic` unconditionally as library is strictly needed only
  for arm & mips, but it seems not to imply any further differences
  with other architectures. Hence, this commit simply adds `atomic`
  unconditionally for a uniform handling of all machine architectures.
  This is an alternative solution to the proposed solution in `#790 <https://github.com/mavlink/mavros/issues/790>`_.
  The issue was discovered cross-compiling mavros in meta-ros, the
  OpenEmbedded layer for ROS. Some further pointers are available at:
  https://github.com/bmwcarit/meta-ros/issues/525
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@gmail.com>
* setpoint_attitude: privatize message_filters subscribers
* Updating comments for PX4Flow
* Removing copter_visualization from the yaml files.
  Adding odometry to apm_config
  Changing frame_id to base_link for vibration
* Update the apm_config and px4flow_config files
* Update configuration from mavros_extras
* Updating default settings from px4.yaml
* * global_position/tf/send default to false
  * imu, checked
  * local_position/tf/send default to false
  * local_position/tf/send_fcu default to false
  * mission/pull_after_gcs default to true
* Update time reference to fcu
  Adding global_frame_id: 'earth' to apm_config
* fcu to base_link
* Changing fcu_utm to fcu
* Solving default frame consistency in config files
* Contributors: Alexis Paques, James Mare, James Stewart, Lukas Bulwahn, TSC21, Vladimir Ermakov

0.20.1 (2017-08-28)
-------------------

0.20.0 (2017-08-23)
-------------------
* update generated code in plugins
* update generated code
* geolib: datasets: warn when not installed; update install script; launch SIGINT when not installed (`#778 <https://github.com/mavlink/mavros/issues/778>`_)
  * geolib: make dataset install mandatory
  * travis_ci: install python3; use geographiclib-datasets-download
  * CMakeLists.txt: set datasets path
  * travis_ci: create a path for the geoid dataset
  * travis_ci: remove python3 install
  * CMakeLists.txt: remove restriction regarding the geoid model
  * CMakeLists.txt: only launch a warning if the geoid dataset is not installed
  * CMakeLists.txt: simplify dataset path search and presentation
  * scripts: install_geographiclib_datasets becomes version aware
  * uas_data: dataset init: shutdown node if exception caught
  * README: update GeographicLib info; geolib install script: check for more OS versions
  * uas_data: small typo fix
  * install_geolib_datasets: some fix
  * CMakeLists.txt: be more clear on geoid dataset fault
  * CMakeLists: push check geolib datasets to a cmake module
  * travis_ci: update ppa repository
  * uas_data: shutdown node and increase log level instead
  * install_geographiclib_datasets: simplify script to only check download script version available
  * uas_data: remove signal.h import
* HIL Plugin
  * add HilSensor.msg, HilStateQuaternion.msg, and add them in CMakeLists.txt
  * Add hil_sensor.cpp plugin to send HIL_SENSOR mavlink message to FCU.
  * fix HilSensor.msg. Make it more compact.
  * Fix HilStateQuaternion.msg. Make it more compact.
  * Add hil_state_quaternion plugin
  * fix files: some variable names were wrong+some syntax problems
  * fix syntax error in plugin .cpp files, make msg files match corresponding mavlink definitions
  * fix plugin source files
  * fix syntax
  * fix function name. It was wrong.
  * add HIL_GPS plugin
  * add HilGPS.msg to CMakeList
  * fix missing semicolon
  * fix call of class name
  * Add ACTUATOR_CONTROL_TARGET MAVLink message
  * fix code
  * increase number of fake satellites
  * control sensor and control rates
  * change control rate
  * change control rate
  * fix fake gps rate
  * fix
  * fix plugin_list
  * fix
  * remove unnecessary hil_sensor_mixin
  * update HilSensor.msg and usage
  * update HilStateQuaterion.msg and usage
  * redo some changes; update HilGPS.msg and usage
  * update hil_controls msg - use array of floats for aux channels
  * merge actuator_control with actuator_control_target
  * remove hil_sensor_mixin.h
  * update actuator_control logic
  * merge all plugins into a single one
  * delete the remaining plugin files
  * update description
  * redo some changes; reduce LOC
  * fix type cast on gps coord
  * add HIL_OPTICAL_FLOW send based on OpticalFlowRad sub
  * update authors list
  * update subscribers names
  * refactor gps coord convention
  * add HIL_RC_INPUTS_RAW sender; cog protec msg structure and content
  * apply correct rc_in translation; redo cog
  * apply proper rotations and frame transforms
  * remote throttle
  * fix typo and msg api
  * small changes
  * refactor rcin_raw_cb
  * new refactor to rcin_raw_cb arrays
  * update velocity to meters
  * readjust all the units so to match mavlink msg def
  * update cog
  * correct cog conversion
  * refefine msg definitions to remove overhead
  * hil: apply frame transform to body frame
* apm_config.yaml: change prevent collision in distance_sensor id
* Extras: add ardupilot rangefinder plugin
* msgs fix `#625 <https://github.com/mavlink/mavros/issues/625>`_: Rename SetMode.Response.success to mode_sent
* [WIP] Plugins: setpoint_attitude: add sync between thrust and attitude (`#700 <https://github.com/mavlink/mavros/issues/700>`_)
  * plugins: setpoint_attitude: add sync between throttle and attitude topics to be sent together
  * plugins: typo correction: replace throttle with thrust
  * plugins: msgs: setpoint_attitude: replaces Float32Stamped for Thrust msg
  * plugins: setpoint_attitude: add sync between twist and thrust (RPY+Thrust)
  * setpoint_attitude: update the logic of thrust normalization verification
  * setpoint_attitude: implement sync between tf listener and thrust subscriber
  * TF sync listener: generalize topic type that can be syncronized with TF2
  * TF2ListenerMixin: keep class template, use template for tf sync method only
  * TF2ListenerMixin: fix and improve sync tf2_start method
  * general update to yaml config files and parameters
  * setpoint_attitude: add note on Thrust sub name
  * setpoint_attitude: TF sync: pass subscriber pointer instead of binding it
* apm_config: add mavros_extras/fake_gps plugin param config
* px4_config: add gps_rate param
* frame tf: move ENU<->ECEF transforms to ftf_frame_conversions.cpp
* extras: mocap_fake_gps->fake_gps: generalize plugin and use GeographicLib possibilites
* UAS: Share egm96_5 geoid via UAS class
* Move FindGeographicLib.cmake to libmavconn, that simplify installation, simplify datasets instattator
* Use GeographicLib tools to guarantee ROS msg def and enhance features (`#693 <https://github.com/mavlink/mavros/issues/693>`_)
  * first commit
  * Check for GeographicLib first without having to install it from the beginning each compile time
  * add necessary cmake files
  * remove gps_conversions.h and use GeographicLib to obtain the UTM coordinates
  * move conversion functions to utils.h
  * geographic conversions: update CMakeLists and package.xml
  * geographic conversions: force download of the datasets
  * geographic conversions: remove unneeded cmake module
  * dependencies: use SHARED libs of geographiclib
  * dependencies: correct FindGeographicLib.cmake so it can work for common Debian platforms
  * CMakeList: do not be so restrict about GeographicLib dependency
  * global position: odometry-use ECEF instead of UTM; update other fields
  * global position: make travis happy
  * global position: fix ident
  * global_position: apply correct frames and frame transforms given each coordinate frame
  * global_position: convert rcvd global origin to ECEF
  * global_position: be more explicit about the ecef-enu transform
  * global position: use home position as origin of map frame
  * global position: minor refactoring
  * global position: shield code with exception catch
  * fix identation
  * move dataset install to script; update README with new functionalities
  * update README with warning
  * global_position: fix identation
  * update HomePosition to be consistent with the conversions in global_position to ensure the correct transformation of height
  * home|global_position: fix compile errors, logic and dependencies
  * home position: add height conversion
  * travis: update to get datasets
  * install geo dataset: update to verify alternative dataset folders
  * travis: remove dataset install to allow clean build
  * hp and gp: initialize geoid dataset once and make it thread safe
  * README: update description relative to GeographicLib; fix typos
  * global position: improve doxygen references
  * README: update with some tips on rosdep install
* [WIP] Set framework to define offset between global origin and current local position (`#691 <https://github.com/mavlink/mavros/issues/691>`_)
  * add handlers for GPS_GLOBAL_ORIGIN and SET_GPS_GLOBAL_ORIGIN
  * fix cast of encoding types
  * refactor gps coord conversions
  * uncrustify
  * global_position: add LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET handler
  * global_position: add trasform sender for offset
  * global_origin: refactor covariance matrix
  * global_position: update copyright
  * global_position: add initial support to REP 105
  * px4_config: global_position: update frame description
  * global_position: correct identation
  * global position: be consistent with frame and methods names (ecef!=wgs84, frame_id!=global_frame_id)
  * global_position: updates to code structure
  * global_position: fix identation
* lib: frame_tf: Style fix
* extras: odom: Minor fixes
* extras: Add odom plugin
* lib: frame_tf: Add support for 6d and 9d covariance matrices
* Contributors: James Goppert, Nuno Marques, TSC21, Vladimir Ermakov, khancyr

0.19.0 (2017-05-05)
-------------------
* launch: remove setpoint-attitude from apm blacklist
* lib: cleanup in enum_to_string
* extras: Add ADSB plugin
* plugin: home_position: Log poll
* plugin: home_position: Log report
* plugin `#695 <https://github.com/mavlink/mavros/issues/695>`_: Fix plugin
* plugin: Add home_position
* Added SAFETY_ALLOWED_AREA rx handler (`#689 <https://github.com/mavlink/mavros/issues/689>`_)
  * Added SAFETY_ALLOWED_AREA rx handler and publish PolygonStamped msg with the 2 points
  * add resize to array to avoid sigfault
* lib: Fix millis timesync passthrough
* Plugin: Add unstamped Twist subscriber for setpoint_velocity
* uas: Move timesync_mode enum to utils.h + fixes
  That enum are used for utils too, but forward declaration of class
  internal enum is impossible.
* sys_time: Add timesync mode selection parameter.
* sys_time : add multi-mode timesync
* uas : add multi-mode timesync
* uas : add multi-mode timesync
* launch fix `#670 <https://github.com/mavlink/mavros/issues/670>`_: Add configuration of distance_sensor plugin for APM
* Contributors: Kabir Mohammed, Nuno Marques, Pierre Kancir, Randy Mackay, Vladimir Ermakov

0.18.7 (2017-02-24)
-------------------
* readme: Add serial-hwfc:// proto
* trigger interface : rename to cycle_time to be consistent with PX4
* Contributors: Kabir Mohammed, Vladimir Ermakov

0.18.6 (2017-02-07)
-------------------
* lib `#626 <https://github.com/mavlink/mavros/issues/626>`_: Porting of PR `#650 <https://github.com/mavlink/mavros/issues/650>`_ - Fix OSX pthread set name.
* uas fix `#639 <https://github.com/mavlink/mavros/issues/639>`_: Remove Boost::signals2 from UAS
* Plugins: system_status change status field to system_status
  Add comment to State.msg for system_status enum
* Plugins: add system_status to state message
* Contributors: Fadri Furrer, Pierre Kancir, Vladimir Ermakov

0.18.5 (2016-12-12)
-------------------
* lib: update ArduPilot modes
* Contributors: Randy Mackay

0.18.4 (2016-11-11)
-------------------
* lib: Add ArduSub modes
* readme: Fix mavlink rosinstall_generator call
* mavros: README.md: its -> it's
  Here "it's" is a short form for "it is".
* add hil_actuator_controls mavlink message
* lib: Make cog.py scrips compatioble with Py3
* plugin:sys_status: Add logging health report
* Update README for all packages
* Update README.md
  Fix instructions: Only the Kinetic distro actually works for MAVLink 2.0
* Contributors: Beat Kung, Georgii Staroselskii, Lorenz Meier, Vladimir Ermakov

0.18.3 (2016-07-07)
-------------------
* plugin:param: Use mavlink::set_string() helper
* Update README.md
* Update README.md
  Fix very confusing instructions mixing steps.
* Update README.md
* Update README.md
* python `#569 <https://github.com/mavlink/mavros/issues/569>`_: convert_to_rosmsg() support for 2.0. NO SIGNING.
* python `#569 <https://github.com/mavlink/mavros/issues/569>`_: Update mavlink.convert_to_bytes()
* Contributors: Lorenz Meier, Vladimir Ermakov

0.18.2 (2016-06-30)
-------------------
* plugin:sys_status: Fix STATUSTEXT log prefix
* Contributors: Vladimir Ermakov

0.18.1 (2016-06-24)
-------------------
* lib: Fix base mode flag check
* plugins: Move pluginlib macros.h to tail
* plugin:param fix `#559 <https://github.com/mavlink/mavros/issues/559>`_: Ignore PX4 _HASH_CHECK param
* Contributors: Vladimir Ermakov

0.18.0 (2016-06-23)
-------------------
* lib `#439 <https://github.com/mavlink/mavros/issues/439>`_: MAV_CMD to_string is not required.
* plugin:sys_status `#458 <https://github.com/mavlink/mavros/issues/458>`_: Hanlde BATTERY_STATUS (PX4)
* plugin:sys_status fix `#458 <https://github.com/mavlink/mavros/issues/458>`_: Use sensor_msgs/BatteryState message.
  Minimal data, for all other need to handle BATTERY_STATUS.
* plugin:command fix `#561 <https://github.com/mavlink/mavros/issues/561>`_: PX4 now sends COMMAND_ACK.
  And like APM do not check confirmation field. :)
* readme `#544 <https://github.com/mavlink/mavros/issues/544>`_: add udp-b://@ URL
* plugin:hil_controls: Update plugin API
* Merge branch 'feature/hil_controls_plugin' of https://github.com/pvechersky/mavros into pvechersky-feature/hil_controls_plugin
  * 'feature/hil_controls_plugin' of https://github.com/pvechersky/mavros:
  Adding anchor to the HIL_CONTROLS message reference link
  Ran uncrustify on hil_controls plugin
  Utilizing synchronise_stamp and adding reference to MAVLINK msg documentation
  Added a plugin that publishes HIL_CONTROLS as ROS messages
* node: fix subscription message type checks
* plugin: use mavlink::to_string() for std::array<char, N>
* readme: update CI, no more MAVLINK_DIALECT
* plugin:waypoint: Fix target id's on MISSION_ITEM
* node: Add ~fcu_protocol parameter
* Ran uncrustify on hil_controls plugin
* Utilizing synchronise_stamp and adding reference to MAVLINK msg documentation
* node: set gcs_url on internal GCS bridge diag hardware Id
* plugins: Use UAS::msg_set_target()
* Added a plugin that publishes HIL_CONTROLS as ROS messages
* lib: PX4 add AUTO.FOLLOW_TARGET
* mavros: Update tests
* extras: Update UAS
* UAS: Update plugins for FTF module
* UAS: move enum stringify functions
* lib: Generate MAV_SENSOR_ORIENTATION
* UAS: move MAV_SENSOR_ORIENTATION out
* UAS: Move transformation utilities to ftf module
* plugin:rc_io: Fix log printf-format warning
* make GCC 4.8 happy. (travis)
* gcs_bridge: done
* param:ftp: Update API
* plugin:param: Works. Tested on APM
* plugin:param: Update, almost work
* plugin:waypoint: Fix Item - ROS binding
* Message type mismatch code do not work
* plugin:waypoint: Update API
* plugin:sys_time: Update API
* plugin:sys_status: Update API
* plugin:setpoint_raw: Update API
* plugin:setpoint_attitude: Update API
* plugin:setpoint_accel: Update API
* plugin:setpoint_velocity: Update API
* plugin:setpoint_position: Update API
* plugin:vfr_hud: Update API
* plugin:safety_area: Update API
* plugin:rc_io: Update API
* plugin:manual_control: Update API, fix uas init
* plugin:local_position: Update API
* plugin:imu_pub: Update API
* plugin:global_position: Update API
* mavros: make_handle() this shouldn't be const
* plugin:common: Update API
* plugin:altitude: uncrustify
* plugins: Rutine sed + fix misprint
* plugin:altitude: Update API
* plugins: Automatic replacement of routine API changes (sed)
* plugin:actuator_control: Update API
* plugin:3dr_radio: Update API
* node: Update plugin loading and message routing
* node: type_info -> SIGSEGV
* node: prepare new plugin loading
* node: Rename plugib base class - API incompatible to old class
* labmavconn: finding sigsegv
* Contributors: Pavel, Vladimir Ermakov

0.17.3 (2016-05-20)
-------------------
* libmavconn `#543 <https://github.com/mavlink/mavros/issues/543>`_: support build with mavlink 2.0 capable mavgen
* node: Remove warning about MAVLINK_VERSION redefine
* Fix bug with orientation in setpoint_raw plugin
  Fixes a bug where the ned_desired_orientation was not actually passed into set_attitude_target. Instead, the desired_orientation (wrong frame) was passed.
* Contributors: Justin Thomas, Vladimir Ermakov

0.17.2 (2016-04-29)
-------------------
* Update README.md
* Update README.md
  Updated / completed examples.
* Update README.md
* Fix for kinetic std::isnan.
* Contributors: James Goppert, Lorenz Meier

0.17.1 (2016-03-28)
-------------------
* lib: Add QLAND mode of APM:Plane
  https://github.com/mavlink/mavlink/commit/a0ed95c3a7d97a8f8d86ce3f95c4bf269f439c46
* Update contributing guide
  We forgot to mention uncrustify commit.
* Treat submarine vehicles like copter vehicles
* Contributors: Josh Villbrandt, Vladimir Ermakov

0.17.0 (2016-02-09)
-------------------
* update README
* rebased with master
* Fixed ROS_BREAK
* Updates for ROS_BREAK and code style
* Nitpicks and uncrustify
* Updated frame transformations and added odom publisher to local position plugin
* Contributors: Eddy, Vladimir Ermakov, francois

0.16.6 (2016-02-04)
-------------------
* node fix `#494 <https://github.com/mavlink/mavros/issues/494>`_: Report FCU firmware type in rosonsole log
* scripts fix `#478 <https://github.com/mavlink/mavros/issues/478>`_: Remove guided_enable garbage.
  I'm missed this when do `#407 <https://github.com/mavlink/mavros/issues/407>`_.
* Contributors: Vladimir Ermakov

0.16.5 (2016-01-11)
-------------------
* scripts: mavwp `#465 <https://github.com/mavlink/mavros/issues/465>`_: Remove WaypointGOTO from scrips and python library
* node: Report mavlink package version
* lib: Add APM:Plane QuadPlane modes.
  Sync with: https://github.com/mavlink/mavlink/commit/1fc4aef08a54130f297943c246f95b8c7e37b1bf
* readme: pixhawk dialect removed.
* Contributors: Vladimir Ermakov

0.16.4 (2015-12-14)
-------------------
* scripts: checkid: be always verbose, add --follow
* scripts: fix copyright indent
* scripts: mavcmd: Fix bug: param7 not passed to service call!
* scripts `#382 <https://github.com/mavlink/mavros/issues/382>`_: Add ID checker script.
  It is not complete, but i hope it helps in current state.
* scripts: mavcmd: Add support for broadcast requests
* event_launcher: fix bug: Trigger service server is not saved in Launcher
  Also fixes: environment variables may contain ~ (user dir) in expansion.
* using timestamp from mavlink message
* Update mavlink message documentation links
* lib: update MAV_TYPE stringify
* lib: Add RATTITUDE PX4 mode
* remove "altitude\_" prefix from members
* updated copyright
* implemented altitude plugin
* Contributors: Andreas Antener, Vladimir Ermakov

0.16.3 (2015-11-19)
-------------------
* use safe methods to get imu data in local_position plugin
* Contributors: Andreas Antener

0.16.2 (2015-11-17)
-------------------
* transform yaw and yaw rate from enu to ned
* Contributors: Andreas Antener

0.16.1 (2015-11-13)
-------------------
* python: fix import error of goto service
* don't warn anymore about px4 not supporting rc_io
* Contributors: Andreas Antener, Vladimir Ermakov

0.16.0 (2015-11-09)
-------------------
* lib: Update ArduCopter mode list
* plugin: sys_status `#423 <https://github.com/mavlink/mavros/issues/423>`_: set_mode set arming and HIL flags based on previous state
* lib `#423 <https://github.com/mavlink/mavros/issues/423>`_: Save base_mode in UAS.
* Finalized local position topic names
* readme: add link to catkin-tools docs
* readme `#409 <https://github.com/mavlink/mavros/issues/409>`_: merge mavlink and mavros installation instruction
* Fixed redundant rotation of IMU data and redundant orientation data
* plugin: setpoint_raw fix `#418 <https://github.com/mavlink/mavros/issues/418>`_: add attitude raw setpoint
  Related `#402 <https://github.com/mavlink/mavros/issues/402>`_.
* Added velocity output of FCU's local position estimate to ROS node
* plugin: sys_status fix `#417 <https://github.com/mavlink/mavros/issues/417>`_: remove APM statustext quirk
* plugin: waypoint fix `#414 <https://github.com/mavlink/mavros/issues/414>`_: remove GOTO service.
  It is replaced with more standard global setpoint messages.
* plugin: setpoint_raw fix `#415 <https://github.com/mavlink/mavros/issues/415>`_: add global position target support
  Related to `#402 <https://github.com/mavlink/mavros/issues/402>`_.
* plugin: command fix `#407 <https://github.com/mavlink/mavros/issues/407>`_: remove guided_enable sevice
* plugin: setpoint_raw `#402 <https://github.com/mavlink/mavros/issues/402>`_: implement loopback.
* plugin: setpoint_raw `#402 <https://github.com/mavlink/mavros/issues/402>`_: Initial import.
* readme fix `#410 <https://github.com/mavlink/mavros/issues/410>`_: use only catkin tool
* readme: add defaults for URL
* pass new extended state to ros
* python: add util to convert pymavlink message to Mavlink.msg
* python: convert input to bytearray
* python: add payload convertion util
* gcs_bridge `#394 <https://github.com/mavlink/mavros/issues/394>`_: enable both UDPROS and TCPROS transports
* EL: add try-except on handlers
* event_launcher: show logfile path
* event_launcher `#386 <https://github.com/mavlink/mavros/issues/386>`_: expand shell vars for logfile
* Mavros library depends on mavros_msgs headers
  Adding this dependency makes sure that mavros_msgs message headers are
  generated before the mavros library is built, since it needs those
  headers.
* Contributors: Andreas Antener, Eddy, Jon Binney, Vladimir Ermakov

0.15.0 (2015-09-17)
-------------------
* lib: fix timesync uninit bug.
  Uninitialized variable caused wrong timestamps with APM.
* python `#286 <https://github.com/mavlink/mavros/issues/286>`_: use checksum - save ticks
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: output to log-file
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: remove RosrunHandler and RoslaunchHandler
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: attempt to implement rosrun fails.
  ROSLaunch class wants all node operations from main thread.
  That is not possible.
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: fix shell-killer, but logging are broken and removed
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: shell-launcher now works!
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: add example configuration
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: shell handler done. next - rosparam handling
* script `#385 <https://github.com/mavlink/mavros/issues/385>`_: starting work on simple shell launcher
* scripts: starting event_launcher
* python: Remove unneded slice operation. Fix copyright year.
  `list[:len(list)]` is equal to `list`, but creates new list with data
  from that slice.
* updated mavlink byte buffer conversion
* plugin: manual_control: Use shared pointer message
  Fix alphabetic order of msgs.
* python: add helper for converting mavros_msgs/Mavlink to pymavlink
* Add MANUAL_CONTROL handling with new plugin
* Contributors: Andreas Antener, Vladimir Ermakov, v01d

0.14.2 (2015-08-20)
-------------------

0.14.1 (2015-08-19)
-------------------
* package: Fix depend on rosconsole-bridge
* Removed <remap\>
* Contributors: Vladimir Ermakov, devbharat

0.14.0 (2015-08-17)
-------------------
* python: call of mavros.set_namespace() is required.
* scripts: mavftp fix `#357 <https://github.com/mavlink/mavros/issues/357>`_: add verify command
* scripts: mavftp `#357 <https://github.com/mavlink/mavros/issues/357>`_: progressbar on download operation
* scripts: mavftp `#357 <https://github.com/mavlink/mavros/issues/357>`_: progress bar for upload operation.
* scripts: mavftp: New command `cd`.
  All path arguments now may handle relative paths.
* readme: fix frame tansform section
* mavros: readme: update info on frame conversions
* mavros: readme: update contribution steps
* node: Replace deprecated copy functions.
  Also allow mavlink to & from topics to be namespaced.
* extras: scripts: use API from mavros module
* scripts: fix for new message location
* python: update mavros lib to new message location
* package: remove not exist dependency
* plugin: waypoint: Fix message include
* plugin: vfr_hud: Fix message include
* plugin: rc_io: Fix message include
* plugin: param: Fix message include
* plugin: ftp: Fix message include
* plugin: sys_status: Fix message include
* plugin: command: Fix message include
* plugin: 3dr_radio: Fix message include
* plugin: actuator_control: Fix message include.
* msgs: update copyright year
* msgs: deprecate mavros::Mavlink and copy utils.
* msgs: change description, make catkin lint happy
* msgs `#354 <https://github.com/mavlink/mavros/issues/354>`_: move all messages to mavros_msgs package.
* Minor typo fix.
* node: increase diag timer to 2 Hz
* node: move diagnostic to AsyncSpinner threads.
* Contributors: TSC21, Tony Baltovski, Vladimir Ermakov

0.13.1 (2015-08-05)
-------------------
* lib `#358 <https://github.com/mavlink/mavros/issues/358>`_: cleanup.
  Replace UAS::getYaw() with UAS::quaternion_get_yaw().
* lib `#358 <https://github.com/mavlink/mavros/issues/358>`_: found correct getYaw(). Test for each degrees in -180..180.
* test `#358 <https://github.com/mavlink/mavros/issues/358>`_: test more different angles. Compare rotation result.
* lib `#358 <https://github.com/mavlink/mavros/issues/358>`_: try to implement algo from wikipedia.
* lib `#358 <https://github.com/mavlink/mavros/issues/358>`_: still failing. add recursive test for range -Pi..+Pi
* lib `#358 <https://github.com/mavlink/mavros/issues/358>`_: try solve issue using older eulerAngles()
* lib `#358 <https://github.com/mavlink/mavros/issues/358>`_: remove to_rpy test
* Merge branch 'master' of github.com:mavlink/mavros
  * 'master' of github.com:mavlink/mavros:
  global_position: move relative_alt and compass_heading init back
  add nav_msgs to dependencies so to make Travis happy
  global_position: update pose and twist to odom msg
* test fix `#359 <https://github.com/mavlink/mavros/issues/359>`_: split out quaternion tests.
* lib `#359 <https://github.com/mavlink/mavros/issues/359>`_: move quaternion utils.
* global_position: move relative_alt and compass_heading init back
* add nav_msgs to dependencies so to make Travis happy
* global_position: update pose and twist to odom msg
* test `#358 <https://github.com/mavlink/mavros/issues/358>`_: add tests for negative values and quaternion_to_rpy tf2 compatibility check
  Tests now fails!
* sctipts: fix gps topic path
* lib: fix input validation in UAS::orientation_from_str()
* test: add case for num str->sensor orientation
* package: fix CHANGELOG.rst
* Contributors: TSC21, Vladimir Ermakov

0.13.0 (2015-08-01)
-------------------
* plugin: setpoint_attitude `#352 <https://github.com/mavlink/mavros/issues/352>`_: use new helper.
* plugin: sys: Fix cppcheck and YouCompleteMe warnings
* plugin: ftp: Fix cppcheck errors.
* lib `#352 <https://github.com/mavlink/mavros/issues/352>`_: Add helper function UAS::quaternion_to_mavlink()
* Fixed bug in send_attitude_target()
  The transformed quaternion wasn't being passed to set_attitude_target(), resulting in an incorrect attitude setpoint. I've now fixed this issue.
* scripts: fix mavwp
* test: add test cases for new sensor orientation functions
* remove tf1 dep
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_: Remove TF types from UAS
* plugin: param: new message type: ParamValue
* msgs: Move MAV_CMD values to separate msg
* plugin: command: fix build
* fix whitespaces in python scripts
* Merge pull request `#312 <https://github.com/mavlink/mavros/issues/312>`_ from mhkabir/cam_imu_sync
  Camera IMU synchronisation support added
* Added launch file for PX4 posix sitl to launch gcs_bridge node for bridging posix and gazebo
* scripts: mavftp: little speed up by aligning access to payload length
* launch: Add optional log_output arg
* Merge branch 'orientation_enum_name'
  * orientation_enum_name:
  distance_sensor `#342 <https://github.com/mavlink/mavros/issues/342>`_: correct orientation parameter handling.
  lib `#342 <https://github.com/mavlink/mavros/issues/342>`_: try to convert numeric value too
  px4_config: adapt to distance_sensor params to new features
  distance_sensor: restructure orientation matching and verification
  lib `#342 <https://github.com/mavlink/mavros/issues/342>`_: Added sensor orientation string repr.
* lib `#342 <https://github.com/mavlink/mavros/issues/342>`_: try to convert numeric value too
* px4_config: adapt to distance_sensor params to new features
* lib `#342 <https://github.com/mavlink/mavros/issues/342>`_: Added sensor orientation string repr.
* launch: update local_position conf
* test: Add test for UAS::sensor_orientation_matching()
* Update cmake Eigen3 finding rules.
  Migration described at:
  http://wiki.ros.org/jade/Migration#Eigen_CMake_Module_in_cmake_modules
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_, `#341 <https://github.com/mavlink/mavros/issues/341>`_: preparation for str->MAV_SENSOR_ORIENTATION func
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_: Return quaternion from UAS::sensor_matching()
* lib: Remove unneded NodeHandle
* launch fix `#340 <https://github.com/mavlink/mavros/issues/340>`_: update default component id of PX4.
* plugin: sys_status: Add fallback to adressed version request.
* Can not remove tf package before `#319 <https://github.com/mavlink/mavros/issues/319>`_ is done.
  tf::Vector3 and other tf1-bullet still in use.
* plugin: sys_status: Use broadcast for version request.
* fix `#71 <https://github.com/mavlink/mavros/issues/71>`_: replace depend tf to tf2_ros.
* plugin: Use UAS::syncronized_header() for reduce LOC.
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_: use similar names for covariances as eigen vector
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_: transform_frame() for Covariance3x3
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_: remove unused bullet based transform_frame()
* extras: vision_pose `#71 <https://github.com/mavlink/mavros/issues/71>`_: Use TF2 listener.
  Also `#319 <https://github.com/mavlink/mavros/issues/319>`_.
* plugin `#71 <https://github.com/mavlink/mavros/issues/71>`_: Implement TF2 listener. Change param names.
  Breaks extras.
* uas `#71 <https://github.com/mavlink/mavros/issues/71>`_: Use single TF2 objects for broadcasting and subscription.
* launch: Update configs.
* lib: Add UAS::quaternion_to_rpy()
* plugin: safety_area `#319 <https://github.com/mavlink/mavros/issues/319>`_: Change transform_frame()
* plugin: local_position `#71 <https://github.com/mavlink/mavros/issues/71>`_ `#319 <https://github.com/mavlink/mavros/issues/319>`_: port to TF2 and Eigen
* lib: Add UAS::synchonized_header()
* plugin: command: Add command broadcasting support.
* Perform the autopilot version request as broadcast
* lib: Update PX4 mode list
* plugin: global_position `#325 <https://github.com/mavlink/mavros/issues/325>`_: port tf broadcaster to tf2
  Also `#71 <https://github.com/mavlink/mavros/issues/71>`_.
* plugin: global_position `#325 <https://github.com/mavlink/mavros/issues/325>`_: reenable UTM calc
* plugin: gps `#325 <https://github.com/mavlink/mavros/issues/325>`_: remove gps plugin.
* plugin: global_position `#325 <https://github.com/mavlink/mavros/issues/325>`_: merge gps_raw_int handler
* plugin: setpoint_accel `#319 <https://github.com/mavlink/mavros/issues/319>`_: use eigen frame transform.
  I don't think that PX4 support any other frame than LOCAL_NED.
  So i removed comment.
  Also style fix in setpoint_velocity.
* plugin: setpoint_velocity `#319 <https://github.com/mavlink/mavros/issues/319>`_: use eigen based frame transform.
* plugin: setpoint_position `#273 <https://github.com/mavlink/mavros/issues/273>`_: remove PX4 quirk, it is fixed.
* plugin: ftp: Update command enum.
* plugin: imu_pub fix `#320 <https://github.com/mavlink/mavros/issues/320>`_: move constants outside class, else runtime linkage error.
* plugin: imu_pub `#320 <https://github.com/mavlink/mavros/issues/320>`_: first attempt
* eigen `#319 <https://github.com/mavlink/mavros/issues/319>`_: handy wrappers.
* eigen `#319 <https://github.com/mavlink/mavros/issues/319>`_: add euler-quat function.
  Also `#321 <https://github.com/mavlink/mavros/issues/321>`_.
* test `#321 <https://github.com/mavlink/mavros/issues/321>`_: remove duplicated test cases, separate by library.
  Add test for checking compatibility of tf::quaternionFromRPY() and Eigen
  based math.
* test `#321 <https://github.com/mavlink/mavros/issues/321>`_: testing eigen-based transforms.
  We should check what convention used by tf::Matrix to be sure that
  our method is compatible.
* mavros `#319 <https://github.com/mavlink/mavros/issues/319>`_: Add Eigen dependency and cmake rule.
* test: test for UAS::transform_frame_attitude_rpy() (ERRORs!)
* test: test for UAS::transform_frame_xyz()
* test: Initial import test_frame_conv
* cam_imu_sync : fix running
* imu_cam_sync : fix formatting
* command handling in mavcmd for camera trigger
* Camera IMU synchronisation support added
* Contributors: Anurag Makineni, Lorenz Meier, Mohammed Kabir, TSC21, Vladimir Ermakov, devbharat

0.12.0 (2015-07-01)
-------------------
* plugin: sys_time, sys_status `#266 <https://github.com/vooon/mavros/issues/266>`_: check that rate is zero
* test `#321 <https://github.com/vooon/mavros/issues/321>`__: disable tests for broken transforms.
* lib `#321 <https://github.com/vooon/mavros/issues/321>`__: frame transform are broken. again! revert old math.
  RULE for me: do not accept patch without wide testing from author.
  That PR changes all plugins code, instead of do API, test and only after
  that touching working code. My bad.
* unittest: added 6x6 Covariance conversion test
* frame_conversions: update comments; filter covariance by value of element 0
* unittests: corrected outputs from conversion tests
* test: other quaternion transform tests
* test: UAS::transform_frame_attitude_q()
* test: test for UAS::transform_frame_attitude_rpy() (ERRORs!)
* test: test for UAS::transform_frame_xyz()
* test: Initial import test_frame_conv
* coverity: make them happy
* uncrustify: fix style on frame conversions
* uncrustify: includes
* plugin: sys_status `#266 <https://github.com/vooon/mavros/issues/266>`_: replace period with rate parameter
* plugin: sys_time `#266 <https://github.com/vooon/mavros/issues/266>`_: Replace period with rate parameters
* frame_conversion: last fix patch
* frame_conversions: use inline functions to identify direction of conversion
* changed frame conversion func name; add 3x3 cov matrix frame conversion; general doxygen comment cleanup
* frame_conversions: added covariance frame conversion for full pose 6x6 matrix
* frame_conversions: added frame_conversion specific lib file; applied correct frame conversion between ENU<->NED
* sys_status `#300 <https://github.com/vooon/mavros/issues/300>`_: PX4 place in [0] lest significant byte of git hash.
* sys_status fix `#300 <https://github.com/vooon/mavros/issues/300>`_: fix u8->hex func.
* plugin: waypoint: cosmetics.
* vibration_plugin: first commit
* Changes some frames from world to body conversion for NED to ENU.
* mavsys `#293 <https://github.com/vooon/mavros/issues/293>`_: add --wait option
* mavsys: Fix arguments help
* mavcmd `#293 <https://github.com/vooon/mavros/issues/293>`_: Add --wait option.
  New function: util.wait_fcu_connection(timeout=None) implement wait
  option.
* sys_status `#300 <https://github.com/vooon/mavros/issues/300>`_: AUTOPILOT_VERSION APM quirk
* mavros `#302 <https://github.com/vooon/mavros/issues/302>`_: fix style
* mavros `#302 <https://github.com/vooon/mavros/issues/302>`_: split UAS impl by function blocks
* mavros fix `#301 <https://github.com/vooon/mavros/issues/301>`_: move sensor orientation util to UAS
* distance_sensor: typo; style fixe
* sensor_orientation: list values correction
* launch: APM:Plane 3.3.0 now support local_position.
  Blacklist distance_sensor.
* sensor_orientation: use MAX as last index macro
* distance_sensor: changed to usable config
* launch: APM:Plane 3.3.0 now support local_position.
  Blacklist distance_sensor.
* sensor_orientation: updated orientation enum; updated data type
* sensor_orientation: included array type on utils.h
* sensor_orientation: added sensor orientation matching helper func
* distance_sensor: updated config file
* distance_sensor: define sensor position through param config
* distance_sensor: array limiting; cast correction; other minor correc
* distance_sensor: small enhancements
* sys_status `#293 <https://github.com/vooon/mavros/issues/293>`_: initialize state topic
* sys_status `#293 <https://github.com/vooon/mavros/issues/293>`_: expose connection flag in mavros/State.
* Contributors: TSC21, Tony Baltovski, Vladimir Ermakov

0.11.2 (2015-04-26)
-------------------
* plugin: param fix `#276 <https://github.com/vooon/mavros/issues/276>`_: add check before reset request downcounter.
  If on MR request FCU responses param with different `param_index`
  do not reset repeat counter to prevent endless loop.
* gcs bridge fix `#277 <https://github.com/vooon/mavros/issues/277>`_: add link diagnostics
* plugin: setpoint_position `#273 <https://github.com/vooon/mavros/issues/273>`__: add quirk for PX4.
* readme: fir glossary misprint
* readme: add notes about catkin tool
* Contributors: Vladimir Ermakov

0.11.1 (2015-04-06)
-------------------
* scripts `#262 <https://github.com/vooon/mavros/issues/262>`_: update mavwp
* scripts `#262 <https://github.com/vooon/mavros/issues/262>`_: mavsetp, new module mavros.setpoint
* mavftpfuse `#129 <https://github.com/vooon/mavros/issues/129>`_: cache file attrs
* mavparam `#262 <https://github.com/vooon/mavros/issues/262>`_: use get_topic()
* mavsys `#262 <https://github.com/vooon/mavros/issues/262>`_: use get_topic()
* mavcmd `#262 <https://github.com/vooon/mavros/issues/262>`_: use get_topic()
* mavftp `#263 <https://github.com/vooon/mavros/issues/263>`_, `#262 <https://github.com/vooon/mavros/issues/262>`_: use crc32 checksums
* python `#262 <https://github.com/vooon/mavros/issues/262>`_: add get_topic()
* Update local_position.cpp
  removed irritating comment
* readme: add short glossary
* plugin: setpoint_attitude: remove unneded ns
* Contributors: Marcel Stuettgen, Vladimir Ermakov

0.11.0 (2015-03-24)
-------------------
* plugin: setpoint_position `#247 <https://github.com/vooon/mavros/issues/247>`_: rename topic
* launch `#257 <https://github.com/vooon/mavros/issues/257>`_: rename blacklist.yaml to pluginlists.yaml
* node `#257 <https://github.com/vooon/mavros/issues/257>`_: implement while list.
* plugin: actuator_control `#247 <https://github.com/vooon/mavros/issues/247>`_: update topic name.
* mavros: Initialize UAS before connecting plugin routing.
  Inspired by `#256 <https://github.com/vooon/mavros/issues/256>`_.
* plugin: sys_status: Check sender id.
  Inspired by `#256 <https://github.com/vooon/mavros/issues/256>`_.
* plugin: sys_status: Use WARN severity for unknown levels
* uas: Add `UAS::is_my_target()`
  Inspired by `#256 <https://github.com/vooon/mavros/issues/256>`_.
* plugin: global_position: Fill status and covariance if no raw_fix.
  Additional fix for `#252 <https://github.com/vooon/mavros/issues/252>`_.
* launch: change apm target component id
  APM uses 1/1 (sys/comp) by default.
* plugin: sys_status: publish state msg after updating uas
  Before this commit, the custom mode string published in the
  state message was computed using the autopilot type from the
  previous heartbeat message--*not* the autopilot type from the
  current hearbeat message.
  Normally that isn't a problem, but when running a GCS and mavros
  concurrently, both connected to an FCU that routes mavlink packets
  (such as APM), then this causes the custom mode to be computed
  incorrectly, because the mode string for the GCS's hearbeat packet
  will be computed using the FCU's autopilot type, and the mode string
  for the FCU's heartbeat packet will be computed using the GCS's
  autopilot type.
* plugin: global_position: fix nullptr crash
  This fixes a crash in cases where a GLOBAL_POSITION_INT message
  is received before a GPS_RAW_INT message, causing the `gps_fix`
  pointer member to be dereferenced before it has been set.
* msgs: fix spelling, add version rq.
* coverity: init ctor in 3dr_radio
* launch fix `#249 <https://github.com/vooon/mavros/issues/249>`_: update apm blacklist
* launch: rename APM2 to APM.
* launch `#211 <https://github.com/vooon/mavros/issues/211>`_: update configs
* plugin: gps: remove unused param
* plugin: sys_time: remove unused param
* launch fix `#248 <https://github.com/vooon/mavros/issues/248>`_: remove radio launch
* plugin: 3dr_radio `#248 <https://github.com/vooon/mavros/issues/248>`_: add/remove diag conditionally
* plugin: sys_status: move connection params to ns
* plugin: sys_time: fix `#206 <https://github.com/vooon/mavros/issues/206>`_ (param ns)
* node: Inform what dialect built-in node
* plugin: sys_status: Conditionaly add APM diag
* plugin: sys_status: fix `#244 <https://github.com/vooon/mavros/issues/244>`_
* uas `#244 <https://github.com/vooon/mavros/issues/244>`_: add enum lookups
* package: update lic
* license `#242 <https://github.com/vooon/mavros/issues/242>`_: update mavros headers
* plugin: local_positon: use auto
* plugin: imu_pub: Update UAS store.
* plugin: gps: remove diag class, change UAS storage API.
* plugin api `#241 <https://github.com/vooon/mavros/issues/241>`_: move diag updater to UAS.
* plugin api `#241 <https://github.com/vooon/mavros/issues/241>`_: remove global private node handle.
  Now all plugins should define their local node handle (see dummy.cpp).
  Also partially does `#233 <https://github.com/vooon/mavros/issues/233>`_ (unmerge setpoint topic namespace).
* plugin api `#241 <https://github.com/vooon/mavros/issues/241>`_: remove `get_name()`
* package: mavros now has any-link proxy, not only UDP
* Update years. I left gpl header, but it is BSD too.
* Add BSD license option `#220 <https://github.com/vooon/mavros/issues/220>`_
* plugin: sys_status: AUTOPILOT_VERSION support.
  Fix `#96 <https://github.com/vooon/mavros/issues/96>`_.
* mavros fix `#235 <https://github.com/vooon/mavros/issues/235>`_: Use AsyncSpinner to allow plugins chat.
  Old single-threaded spinner have a dead-lock if you tried to call
  a service from for example timer callback.
  For now i hardcoded thread count (4).
* uncrustify: actuator_control
* Merge branch 'master' of github.com:mstuettgen/Mavros
* fixed missing ;
* code cosmetics
* further removed unneeded white spaces and minor code cosmetics
* fixed timestamp and commented in the not-working function call
* code cosmetics, removed whitespaces and re-ordered function signatures
* more code comment cosmetic
* code comment cosmetic
* uncrustify: fix style
* readme: add contributing notes
* uncrustify: mavros base plugins
* uncrustify: mavros lib
* uncrustify: mavros headers
* tools: add uncrustify cfg for fixing codestyle
  Actually it different from my codestyle,
  but much closer than others.
* added more const to function calls to ensure data consistency
* modified code to fit new message
* added group_mix to ActuatorControl.msg and a link to mixing-wiki
* plugin: rc_io: Add override support warning
* REALLY added ActuatorControl.msg
* added ActuatorControl.msg
* fixed latest compiler error
* renamed cpp file to actuator_control.cpp and added the new plugin to mavros_plugins.xml
* removed unneeded Mixinx and reverse_throttle, and unneeded variables in function signatures
* inital draft for set_actuator_control plugin
* launch: enable setpoint plugins for APM
  As of ArduCopter 3.2, APM supports position and velocity setpoints via SET_POSITION_TARGET_LOCAL_NED.
* plugin: setpoint_velocity: Fix vx setpoint
  vz should have been vx.
* Contributors: Clay McClure, Marcel Stuettgen, Vladimir Ermakov

0.10.2 (2015-02-25)
-------------------
* Document launch files
* launch: Fix vim modelines `#213 <https://github.com/vooon/mavros/issues/213>`_
* launch `#210 <https://github.com/vooon/mavros/issues/210>`_: blacklist image_pub by px4 default.
  Fix `#210 <https://github.com/vooon/mavros/issues/210>`_.
* Contributors: Clay McClure, Vladimir Ermakov

0.10.1 (2015-02-02)
-------------------
* Fix @mhkabir name in contributors.
* uas `#200 <https://github.com/vooon/mavros/issues/200>`_: Add APM:Rover custom mode decoding.
  Fix `#200 <https://github.com/vooon/mavros/issues/200>`_.
* uas `#200 <https://github.com/vooon/mavros/issues/200>`_: Update APM:Plane and APM:Copter modes.
* Contributors: Vladimir Ermakov

0.10.0 (2015-01-24)
-------------------
* mavros `#154 <https://github.com/vooon/mavros/issues/154>`_: Add IO stats to diagnostics.
  Fix `#154 <https://github.com/vooon/mavros/issues/154>`_.
* Add rosindex metadata
* plugin: ftp: init ctor.
* plugin: sts_time: Code cleanup and codestyle fix.
* plugin: command: Quirk for older FCU's (component_id)
  Older FCU's expect that commands addtessed to MAV_COMP_ID_SYSTEM_CONTROL.
  Now there parameter: `~cmd/use_comp_id_system_control`
* plugin: rc_io: `#185 <https://github.com/vooon/mavros/issues/185>`_ Use synchronized timestamp.
* plugin: gps: `#185 <https://github.com/vooon/mavros/issues/185>`_ use synchronized timestamp
  common.xml tells that GPS_RAW_INT have time_usec stamps.
* uas: Fix ros timestamp calculation.
  Issues: `#186 <https://github.com/vooon/mavros/issues/186>`_, `#185 <https://github.com/vooon/mavros/issues/185>`_.
* plugin: add synchronisation to most plugins (fixed)
  Closes `#186 <https://github.com/vooon/mavros/issues/186>`_.
* readme: Add notes about coordinate frame conversions `#49 <https://github.com/vooon/mavros/issues/49>`_
* Contributors: Mohammed Kabir, Vladimir Ermakov

0.9.4 (2015-01-06)
------------------
* plugin: sys_time: enable EMA
* Contributors: Mohammed Kabir

0.9.3 (2014-12-30)
------------------
* plugin: visualization finshed
* Restore EMA. Works better for low rates.
* Update sys_time.cpp
* plugin : add time offset field to dt_diag
* Final fixes
* minor
* plugin : fixes timesync. FCU support checked.
* Visualisation system import
* param: Fix float copying too
* param: Fix missing
* param: Trynig to fix 'crosses initialization of XXX' error.
* param: Try to fix `#170 <https://github.com/vooon/mavros/issues/170>`_.
* Update units
* New message, moving average compensation
* Initial import new sync interface
* plugin: sys_status: Enable TERRAIN health decoding.
* Contributors: Mohammed Kabir, Vladimir Ermakov

0.9.2 (2014-11-04)
------------------

0.9.1 (2014-11-03)
------------------
* Update installation notes for `#162 <https://github.com/vooon/mavros/issues/162>`_
* Contributors: Vladimir Ermakov

0.9.0 (2014-11-03)
------------------

0.8.2 (2014-11-03)
------------------
* REP140: update package.xml format.
  Hydro don't accept this format correctly,
  but after split i can update.
* Contributors: Vladimir Ermakov

0.8.1 (2014-11-02)
------------------
* fix build deps for gcs_bridge
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Enable rosconsole bridge.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Move mavconn tests.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Fix headers used in mavros. Add readme.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Fix mavros build.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Move library to its own package
  Also rosconsole replaced by console_bridge, so now library can be used
  without ros infrastructure.
* plugin: sys_time: Set right suffixes to uint64_t constants.
  Issue `#156 <https://github.com/vooon/mavros/issues/156>`_.
* plugin: sys_time: Add time syncronization diag.
  Issue `#156 <https://github.com/vooon/mavros/issues/156>`_.
* plugin: sys_time: Debug result.
  Issue `#156 <https://github.com/vooon/mavros/issues/156>`_.
* plugin: Store time offset in UAS.
  TODO: implement fcu_time().
  Issue `#156 <https://github.com/vooon/mavros/issues/156>`_.
* plugin: sys_time: Fix code style.
  Also reduce class variables count (most not used outside the method).
  Issue `#156 <https://github.com/vooon/mavros/issues/156>`_.
* Update repo links.
  Package moved to mavlink organization.
* Nanosecond fix
* Fix
* Fixes
* Update sys_time.cpp
* Update sys_time.cpp
* Update sys_time.cpp
* Update sys_time.cpp
* Update CMakeLists.txt
* Update mavros_plugins.xml
* Update sys_time.cpp
* Fix build
* sys_time import. Removed all time related stuff from gps and sys_status
* Initial sys_time plugin import
* plugin: ftp: Bytes written now transfered in payload.
* Contributors: Mohammed Kabir, Vladimir Ermakov

0.8.0 (2014-09-22)
------------------
* plugin: ftp: Disable debugging and change level for some log messages.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Translate protocol errors to errno.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* scripts: mavftp: Add upload subcommand.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* python: Add more ftp utils.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Fix write offset calculation.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add FTP:Checksum.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add support for FTP:Rename.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* python: Add FTP:Truncate
* plugin: ftp: Add FTP:Truncate call.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* python: Move common mission classes to mavros.mission module.
  Issue `#157 <https://github.com/vooon/mavros/issues/157>`_.
* python: Move useful utils to mavros.param module.
  Issue `#157 <https://github.com/vooon/mavros/issues/157>`_.
* python: Move common utils to mavros.utils module.
  Issue `#157 <https://github.com/vooon/mavros/issues/157>`_.
* python: Create python module for ftp utils.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_, `#157 <https://github.com/vooon/mavros/issues/157>`_.
* scripts: ftp: Implement file-like object for IO.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Implement write file.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* scripts: mavftp: Add remove subcommand.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add FTP:Remove call.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add response errno from server.
* plugin: ftp: Add support for 'Skip' list entries.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* scripts: mavftp: Add mkdir/rmdir support.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add mkdir/rmdir support.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugins: ftp: Update protocol headers.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* Revert "Update package.xml format to REP140 (2)."
  This reverts commit 81286eb84090a95759591cfab89dd9718ff35b7e.
  ROS Hydro don't fully support REP140: rospack can't find plugin
  descriptions.
  Fix `#151 <https://github.com/vooon/mavros/issues/151>`_.
* scripts: mavwp: Fix --follow mode
* plugin: imu_pub: Fix RAW_IMU/SCALED_IMU angular scale constant.
  Fix `#152 <https://github.com/vooon/mavros/issues/152>`_.
* launch: remove px4_local_gcs.launch again.
  It removed in 826be386938c2735c9dab72283ba4ac1c68dc860,
  but accidentally returned.
* extras: launch: Use includes.
  Fix `#144 <https://github.com/vooon/mavros/issues/144>`_.
* launch: PX4: use node.launch in PX4 scripts.
  Also remove px4_local_gcs.launch: please use
  `roslaunch mavros px4.launch gcs_url:=udp://@localhost` instead.
  Issue `#144 <https://github.com/vooon/mavros/issues/144>`_.
* launch: APM2: Add node.launch and update apm scripts to use it.
  Issue `#144 <https://github.com/vooon/mavros/issues/144>`_.
* plugin: command: Fix CommandInt x,y types.
* Update package.xml format to REP140 (2).
  Fix `#104 <https://github.com/vooon/mavros/issues/104>`_.
* launch: Blacklist FTP for APM.
* scripts: mavwp: Add decoding for some DO-* mission items.
* scripts: mavwp: Add preserve home location option at load operation.
  Useful if FCU stores home location in WP0 (APM).
* Added src location.
* Updated README wstool instructions.
* plugin: ftp: Init ctor
* service: mavftp: Initial import.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Implemnet reset call.
  Sometimes kCmdReset can restore normal operation,
  but it might be dangerous.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Implement FTP:Read call.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Fix open error.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Implement FTP:Open (read) and FTP:Close.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Implement FTP:List method.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Implement list parsing
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Fix CRC32 calculation.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add plugin skeleton.
  Based on QGroundContol QGCUASFileManager.h/cc.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: ftp: Add size info
* plugin: ftp: Add plugin service API.
  Issue `#128 <https://github.com/vooon/mavros/issues/128>`_.
* plugin: vfr_hud: Initial import.
  Also this plugin publish APM specific WIND estimation message.
  Fix `#86 <https://github.com/vooon/mavros/issues/86>`_.
* node: coverity fails at UAS initilizer list
* plugin: setpoint_attitude: Init ctor, remove code dup.
* cmake: Add check MAVLINK_DIALECT value
  Fix `#139 <https://github.com/vooon/mavros/issues/139>`_.
* Move common cmake rules to modules.
  Same mech as in `cmake_modules` package.
  Issue `#139 <https://github.com/vooon/mavros/issues/139>`_.
* launch: corrected launch for gcs bridge
* scripts: mavsetp: Fix misprint.
* launch files: added px4 launch files for connection with radio and gcs
* scripts: mavsetp: Fix twist.angular vector construction.
  Small style fix.
* Update doxygen documentation.
  Add split lines in UAS, and make UAS.connection atomic.
  Add rosdoc configuration for mavros_extras.
* scripts: mavsetp: corrected API; added possibility of parse angles in dg or rad
* scripts: mavsetp: corrected msg API; mavteleop: added prefix to rc override
* scripts: mavsetp: added local accel; corrected how the OFFBOARD mode is swtch.
* scripts: mavsetp: changed the way offboard mode is switched
* node: init ctor (coverity)
* nodelib: add std::array header
* return msg generator deps for mavconn
* scripts: mavsys: Implement set rate command.
* scripts: Add mavsys tool.
  Implented only `mode` operation.
  Issue `#134 <https://github.com/vooon/mavros/issues/134>`_.
* plugin: sys_status: Implement set_mode service.
  Previous command shortcut removed.
  Issue `#136 <https://github.com/vooon/mavros/issues/136>`_, `#134 <https://github.com/vooon/mavros/issues/134>`_.
* node: Implement reverse mode lookup.
  Issue `#136 <https://github.com/vooon/mavros/issues/136>`_.
* plugin: sys_status: Move custom mode decoder to UAS.
  Issue `#136 <https://github.com/vooon/mavros/issues/136>`_.
* node: Catch URL open exception.
  Also update connection pointer type.
* nodelib: move sources to subdir
* node: Move UAS to mavros namespace
* node: Move node code to library.
* node: Catch DeviceError; use C++11 foreach shugar.
* plugin: command: Add COMMAND_INT suport.
  Fix `#98 <https://github.com/vooon/mavros/issues/98>`_.
* Contributors: Nuno Marques, Tony Baltovski, Vladimir Ermakov

0.7.1 (2014-08-25)
------------------
* plugins: setpoint: Update SET_POSITION_TARGET_LOCAL_NED message.
  Fix `#131 <https://github.com/vooon/mavros/issues/131>`_.
* scripts: mavsetp: Enable OFFBOARD mode.
  Issue `#126 <https://github.com/vooon/mavros/issues/126>`_.
* plugin: command: Add guided_enable shortcut
  It enable PX4 OFFBOARD mode.
  Issue `#126 <https://github.com/vooon/mavros/issues/126>`_.
* scripts: Add mavsetp script.
  Only local setpoint for now.
  Issue `#126 <https://github.com/vooon/mavros/issues/126>`_.
* plugins: Change UAS FCU link name.
  Reduce smart pointer count, that hold fcu link object.
* scripts: mavcmd: Add takeoffcur and landcur commands
  Fix `#91 <https://github.com/vooon/mavros/issues/91>`_, `#92 <https://github.com/vooon/mavros/issues/92>`_. Inspired by `#125 <https://github.com/vooon/mavros/issues/125>`_.
* Closes `#122 <https://github.com/vooon/mavros/issues/122>`_, closes `#123 <https://github.com/vooon/mavros/issues/123>`_; plugins: move mocap & vision plugins to extras, change vision plugins name
* plugins: UAS remove std::atomic<double>
  It don't work at some compilers.
  Issue `#89 <https://github.com/vooon/mavros/issues/89>`_.
* plugin: global_position: Fill NavSatFix status filed.
  Issue `#87 <https://github.com/vooon/mavros/issues/87>`_, `#118 <https://github.com/vooon/mavros/issues/118>`_.
* plugins: Add GPS data to UAS
* plugins: Move setpoint_mixin.h
  Fix `#120 <https://github.com/vooon/mavros/issues/120>`_.
* plugin: mocap: Fix load.
  Issue `#121 <https://github.com/vooon/mavros/issues/121>`_.
* plugins: global_position: get pose orientation from the one stored in uas
* plugins: global_position: use relative_alt on position.z;
  mavros_plugins.xml - corrected declaration of mocap_pose_estimate
* plugin - global_position - changed parameter path / orientation source
* launch: APM2 blacklist global_position plugin
* plugin: global_position: Unit unification.
* plugin: global_position: Move heaedr; Style fix.
* added rel_pos and compass_hdg pub; minor corrections
* Merge branch 'master' of https://github.com/vooon/mavros into global_position
* global_position plugin - initial commit
* launch: APM2 blacklist mocap plugin.
* Updated mavros_plugins.xml
* Fixed dual sources error warning.
* Fixed styles.
* Minor changes.
* added time stamp to received msgs
* Removed un-needed times.
* Added mocap_pose_estimate plugin.
* Code style update
* setpoint attitude change - warning message
* Update on setpoint_attitude plugin
  * changed Twist to TwistStamped
  * added reverse_throttle option for throttle control
  * use cmd_vel as the same topic to control linear a angular velocities (it's commonly used by controllers)
  * added normalization filter to thrust
* node: Remove deprecated conn parameters.
  Fix `#108 <https://github.com/vooon/mavros/issues/108>`_
* plugin: vision_speed: Update plugin API.
* plugin: setpoint_attitude: Update plugin API.
* plugin: setpoint_accel: Update plugin API.
* plugin: setpoint_velocity: Update plugin API.
* plugin: 3dr_radio: Update plugin API.
* plugin: safety_area: Update plugin API.
* plugin: setpoint_position: Update plugin API.
* plugin: vision_position: Update plugin API.
* plugin: local_position: Update plugin API.
* plugin: command: Update plugin API.
* plugin: rc_io: Update plugin API.
* plugin: waypoint: Update plugin API.
* plugin: param: Update plugin API.
* plugin: gps: Update plugin API.
* plugin: imu_pub: Update plugin API.
* plugin: sys_status: Update plugin API.
* plugin: Update plugin API.
* plugins: disable most of plugins
* plugin: setpoint_attitude: Add thrust topic.
  Fix `#106 <https://github.com/vooon/mavros/issues/106>`_.
* Fix URLs in readme
* mavros -> ros-message parameter fix
  only parameter1 was forwarded into the ros message
* Switch travis to pixhawk dialect.
  Default dialect build by ros buildfarm.
  Also remove duplicate ci statuses from mavros readme.
* Contributors: Nuno Marques, Tony Baltovski, Vladimir Ermakov, mthz

0.7.0 (2014-08-11)
------------------
* Add package index readme, Fix `#101 <https://github.com/vooon/mavros/issues/101>`_
* move mavros to subdirectory, `#101 <https://github.com/vooon/mavros/issues/101>`_
* Merge branch 'master' of github.com:vooon/mavros
  * 'master' of github.com:vooon/mavros:
  Add link to ros-\*-mavlink package wiki page.
* plugins: setpoint: Update setpoint message name.
  Issue `#94 <https://github.com/vooon/mavros/issues/94>`_, Fix `#97 <https://github.com/vooon/mavros/issues/97>`_.
* plugin: setpoint_attitude: Update message name.
  Issues `#94 <https://github.com/vooon/mavros/issues/94>`_, `#97 <https://github.com/vooon/mavros/issues/97>`_.
* Add link to ros-\*-mavlink package wiki page.
* plugin: gps: Fix gcc 4.6 build (atomic).
  Not recommended to use std::atomic with gcc 4.6.
  So i limited to prederined atomics for simple types like int, float etc.
* plugin: sys_status: Implement PX4 mode decoding.
  Fix `#84 <https://github.com/vooon/mavros/issues/84>`_.
* plugin: gps: Add EPH & EPV to diagnostic.
  Issue `#95 <https://github.com/vooon/mavros/issues/95>`_
* plugin: gps: Move message processing to individual handlers.
  Issue `#95 <https://github.com/vooon/mavros/issues/95>`_.
* plugin: rc_io: Replace override service with topic. (ROS API change).
  Fix `#93 <https://github.com/vooon/mavros/issues/93>`_.
* Add dialect selection notes
* plugins: Change severity for param & wp done messages.
* plugins: Store raw autopilot & mav type values.
  This may fix or not issue `#89 <https://github.com/vooon/mavros/issues/89>`_.
* plugins: init ctor (coverity)
* plugin: imu_pub: Add ATTITUDE_QUATERNION support.
  Also reduce copy-paste and use mode readable bitmask check.
  Fix `#85 <https://github.com/vooon/mavros/issues/85>`_.
* scriptis: mavcmd: Spelling
* scripits: Add mavcmd tool
* Add links to mavros_extras
* param: sys_status: Option to disable diagnostics (except heartbeat)
* plugin: command: Add takeoff and land aliases.
  Issue `#68 <https://github.com/vooon/mavros/issues/68>`_.
* plugin: command: Add quirk for PX4.
  Fix `#82 <https://github.com/vooon/mavros/issues/82>`_.
* plugin: Add UAS.is_px4() helper. Replace some locks with atomic.
  Issue `#82 <https://github.com/vooon/mavros/issues/82>`_.
* launch: Clear PX4 blacklist.
  Issue `#68 <https://github.com/vooon/mavros/issues/68>`_.
* launch: Add target ids.
  Also fix PX4 wrong ?ids usage (it set mavros ids, not target).
  Issue `#68 <https://github.com/vooon/mavros/issues/68>`_.
* plugin: imu_pub: Fix HRIMU pressure calc. 1 mBar is 100 Pa.
  Fix `#79 <https://github.com/vooon/mavros/issues/79>`_.
* plugins: C++11 chrono want time by ref, return \*_DT
  Fix `#80 <https://github.com/vooon/mavros/issues/80>`_.
* plugins: Replace boost threads with C++11.
  And remove boost thread library from build rules.
  Issue `#80 <https://github.com/vooon/mavros/issues/80>`_.
* plugins: Replace Boost condition variables with C++11
  Issue `#80 <https://github.com/vooon/mavros/issues/80>`_.
* plugins: Replace boost mutexes with C++11.
  Issue `#80 <https://github.com/vooon/mavros/issues/80>`_.
* travis clang to old, fails on boost signals2 library. disable.
* travis: enable clang build.
* node: Make project buildable by clang.
  Clang produce more readable errors and provide
  some static code analysis, so i want ability to build mavros
  with that compilator.
* plugins: replace initial memset with c++ initializer list
* launch: PX4 default ids=1,50.
  Also waypoint plugin works (with first_pos_control_flight-5273-gd3d5aa9).
  Issue `#68 <https://github.com/vooon/mavros/issues/68>`_.
* launch: Use connection URL
* plugin: vision_speed: Initial import.
  Fix `#67 <https://github.com/vooon/mavros/issues/67>`_.
* plugin: sys_status: Add SYSTEM_TIME sync send.
  Fix `#78 <https://github.com/vooon/mavros/issues/78>`_.
* plugin: sys_status: Decode sensor health field.
  Fix `#75 <https://github.com/vooon/mavros/issues/75>`_.
* Add ci badges to readme
* plugin: param: erase invalidates iterator.
  Real error found by coverity :)
* plugins: Init ctor
* plugins: Add ctor initialization.
  Coverity recommends init all data members.
* test: trying travis-ci && coverity integration.
  Real ci doing by ros buildfarm.
* plugins: Fix clang-check errors.
* test: Add tcp client reconnect test.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* test: Split open_url test to individual tests.
  Also removed tcp client deletion on close, heisenbug here.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Emit port_closed after thread stop.
  Also use tx state flag, improve error messages and move io post out of
  critical section.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Fix TCP server client deletion.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* test: Remove not needed sleep.
* mavconn: Remove new MsgBuffer dup. Message drop if closed.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Fix TCP server.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* launch: APM2: Blacklist extras.
* mavconn: Add mutex to channel allocation.
* mavconn: Fix TCP server for gcc 4.6
  Fix `#74 <https://github.com/vooon/mavros/issues/74>`_.
* Remove libev from package.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: GCC 4.6 does not support typedef like using.
  Issue `#74 <https://github.com/vooon/mavros/issues/74>`_.
* Merge pull request `#73 <https://github.com/vooon/mavros/issues/73>`_ from vooon/mavconn-revert-asio
  mavconn: Revert to Boost.ASIO
* mavconn: Cleanup boost threads.
  I will use C++11 standard libs.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Remove libev default loop thread.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Port MAVConnTCPServer to Boost.ASIO.
  TCP send test fails.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Port MAVConnTCPClient to Boost.ASIO.
  Also it disables MAVConnTCPServer before i rewrite it.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Revert MAConnSerial back to Boost.ASIO.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* test: Fix send_message tests. Use C++11.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* mavconn: Revert MAVConnUDP back to Boost.ASIO.
  Also starting to change boost threads and mutexes to C++11.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* test: Enable send tests.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* test: And hand test for mavconn hangs.
  Issue `#72 <https://github.com/vooon/mavros/issues/72>`_.
* node: Remove anonimous flag from gcs_bridge.
  Rename node if you want start several copies.
* install: Remove duplicate
* node: Fix mavros_node termination message.
  Issue `#58 <https://github.com/vooon/mavros/issues/58>`_.
* node: Use URL in mavros_node.
  Fix `#58 <https://github.com/vooon/mavros/issues/58>`_.
* node: Use URL in gcs_bridge.
  Issue `#58 <https://github.com/vooon/mavros/issues/58>`_.
* node: Rename ros_udp to gcs_bridge.
  Because now it's not UDP only.
  Issue `#58 <https://github.com/vooon/mavros/issues/58>`_.
* Cleanup boost components
* mavconn: Implement URL parsing.
  Supported shemas:
  * Serial: `/path/to/serial/device[:baudrate]`
  * Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  * UDP: `udp://[bind_host[:port]]@[remote_host[:port]][/?ids=sysid,compid]`
  * TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
  * TCP server: `tcp-l://[bind_port][:port][/?ids=sysid,compid]`
  Note: ids from URL overrides ids given to open_url().
  Issue `#58 <https://github.com/vooon/mavros/issues/58>`_.
* test: Add tests for UDP, TCP, SERIAL.
  Send message testa are broken, need to find workaround.
  Fix `#70 <https://github.com/vooon/mavros/issues/70>`_.
* plugin: vision_position: Add transform timestamp check.
  Issue `#60 <https://github.com/vooon/mavros/issues/60>`_.
* mavconn: Implement TCP server mode.
  Fix `#57 <https://github.com/vooon/mavros/issues/57>`_.
* mavconn: Initial support for TCP client mode.
  Issue `#57 <https://github.com/vooon/mavros/issues/57>`_.
* mavconn: Boost::asio cleanup.
* plugin: Remove TimerService from UAS.
  Fix `#59 <https://github.com/vooon/mavros/issues/59>`_.
* plugin: param: Add state check to sheduled pull.
* mavparam: Add force pull.
* plugin: param: Use ros::Timer for timeouts
  Also new option for force pull parameters from FCU instead of cache.
  Fix `#59 <https://github.com/vooon/mavros/issues/59>`_.
* Add mavsafety info to README.
* launch: Add apm2_radio.launch (for use with 3DR Radio)
* plugin: 3dr_radio: Fix build error.
  Issue `#62 <https://github.com/vooon/mavros/issues/62>`_.
* plugin: 3dr_radio: Publish status data for rqt_plot
  Also tested with SiK 1.7.
  Fix `#62 <https://github.com/vooon/mavros/issues/62>`_.
* plugin: setpoint_attitude: Fix ENU->NED conversion.
  Fix `#64 <https://github.com/vooon/mavros/issues/64>`_.
  Related `#33 <https://github.com/vooon/mavros/issues/33>`_, `#49 <https://github.com/vooon/mavros/issues/49>`_.
* launch: Add setpoint plugins to APM2 blacklist
* plugin: setpoint_attitude: Initial import.
  XXX: need frame conversion `#49 <https://github.com/vooon/mavros/issues/49>`_.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_, `#64 <https://github.com/vooon/mavros/issues/64>`_.
* plugin: Move common tf code to mixin.
  Remove copy-paste tf_listener.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugin: setpoint_position: Generalize topic NS with other `setpoint_*`
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_, `#61 <https://github.com/vooon/mavros/issues/61>`_.
* plugin: setpoint_accel: Initial import.
  Issues: `#33 <https://github.com/vooon/mavros/issues/33>`_, `#61 <https://github.com/vooon/mavros/issues/61>`_.
* plugin: position_velocity: Initial import.
  Also it fix ignore mask in setpoint_position.
  Issues `#33 <https://github.com/vooon/mavros/issues/33>`_, `#61 <https://github.com/vooon/mavros/issues/61>`_.
* plugins: 3rd_radio: Initial import.
  Untested.
  Issue `#61 <https://github.com/vooon/mavros/issues/61>`_.
* scripts: Add mavsafety tool.
  Also add safety_area to APM2 blacklist.
  Fix `#51 <https://github.com/vooon/mavros/issues/51>`_.
* plugins: safty_area: Initial import.
  This plugin listen `~/safety_area/set` and send it's data to FCU.
  Issue `#51 <https://github.com/vooon/mavros/issues/51>`_.
* plugins: position: Add TF rate limit.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugin: waypoint: Use ros::Timer for timeouts.
  Also add some debug messages for next debugging PX4.
  Issue `#59 <https://github.com/vooon/mavros/issues/59>`_.
* plugin: sys_status: Use ros::Timer for timeouts
  Also move message rx to it's own handlers.
  Issue `#59 <https://github.com/vooon/mavros/issues/59>`_.
* Remove rosdep.yaml and update readme
* Add deb build notes to readme.
  Issue `#55 <https://github.com/vooon/mavros/issues/55>`_.
* Add sudo notes to readme.
* Merge pull request `#56 <https://github.com/vooon/mavros/issues/56>`_ from vooon/54_try_libev
  Switch to libev
* Add libev to README
* package: Add temporary rosdep for libev-dev.
  Issue `#54 <https://github.com/vooon/mavros/issues/54>`_.
* mavconn: Move MAVConnUDP to libev.
  And fix docs in serial.
  Issue `#54 <https://github.com/vooon/mavros/issues/54>`_.
* mavconn: Move MAVConnSerial to libev.
  Adds stub for open URL function.
  Issure `#54 <https://github.com/vooon/mavros/issues/54>`_.
* Contributors: Vladimir Ermakov, Mohammed Kabir, Nuno Marques, Glenn Gregory

0.6.0 (2014-07-17)
------------------
* plugin: local_position: Use same timestamp in topic and TF.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugins: TF thread required, remove notes.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* launch: Add example launch for PX4
  Issue `#45 <https://github.com/vooon/mavros/issues/45>`_.
* plugin: imu_pub: Fix attitude store in UAS
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
  Fix `#53 <https://github.com/vooon/mavros/issues/53>`_.
* plugins: Disable position topics if tf_listen enabled
  Also change default frame names: `vision` and `setpoint`.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugins: Fix typo in frame_id params.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugins: Add vision and setpoint TF listeners
  Also change parameter names to same style.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugin: vision_position: Add PositionWithCovarianceStamped option
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* Add boost filesystem lib to link
  On some platforms its absence breaks build by:
  undefined reference to `boost::filesystem::path::codecvt()`
* launch: Add example for APM2
  Fix `#45 <https://github.com/vooon/mavros/issues/45>`_.
* plugin: setpoint_position: Initial import
  And some small doc changes in other position plugins.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* node: Add connection change message
  Fix `#52 <https://github.com/vooon/mavros/issues/52>`_.
* plugins: vision_position: Initial import
  TODO: check ENU->NED maths.
  Issue `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugins: Remove unneded 'FCU' from diag
* plugin: local_position: Change plane conversion
  Bug: `#49 <https://github.com/vooon/mavros/issues/49>`_.
* plugin: imu_pub: Fix magnetic vector convertions
  Bug: `#49 <https://github.com/vooon/mavros/issues/49>`_.
* Use dialects list from package
* plugin: local_position: Fix orientation source
  Part of `#33 <https://github.com/vooon/mavros/issues/33>`_.
* node: Show target system on startup
  Fix `#47 <https://github.com/vooon/mavros/issues/47>`_.
* plugin: local_position: Initial add
  Receive LOCAL_POSITION_NED message and publish it via TF and PoseStamped
  topic in ENU frame.
  Part of `#33 <https://github.com/vooon/mavros/issues/33>`_.
* node: Use boost::make_shared for message allocation
  Fix `#46 <https://github.com/vooon/mavros/issues/46>`_.
* plugins: Use boost::make_shared for message allocation
  Part of `#46 <https://github.com/vooon/mavros/issues/46>`_.
* plugin: imu_pub: Fix misprint in fill function
  Fix magnetometer vector convertion (HR IMU).
  Related `#33 <https://github.com/vooon/mavros/issues/33>`_.
* plugin: imu_pub: setup cleanup.
* Update readme
* plugin: gps: Fix gps_vel calculation
  Fix `#42 <https://github.com/vooon/mavros/issues/42>`_.
* plugins: Make name and messages methods const. (breaking).
  WARNING: this change broke external plugins.
  Please add const to get_name() and get_supported_messages().
  Part of `#38 <https://github.com/vooon/mavros/issues/38>`_.
* plugins: Use mavlink_msg_*_pack_chan() functions
  Fix `#43 <https://github.com/vooon/mavros/issues/43>`_.
* mavconn: Reuse tx buffer (resize by extents)
  Part of `#38 <https://github.com/vooon/mavros/issues/38>`_.
* mavconn: Do not finalize messages if id pair match
  mavlink_*_pack also do finalize, so explicit finalization just
  recalculate crc and seq number (doubles work).
  Test later if we need check seq too.
* mavconn: Documentation and cleanup
  Make MAVConn classes noncopyable.
  Remove copy-paste copy and following async_write calls.
  Reserve some space in tx queues.
  Replace auto_ptr with unique_ptr.
* test: Fix header include
* mavconn: Fix possible array overrun in channel alocation.
  Problem found by clang.
* fix some roslint errors
* mavconn: move headers to include
* node: Implement plugin blacklist.
  New parameter: `~/plugin_blacklist` lists plugin aliases
  with glob syntax.
  Fix `#36 <https://github.com/vooon/mavros/issues/36>`_.
* plugins: Change constants to constexpr (for gcc 4.6)
* mavconn: Add gencpp dependency (utils.h requiers generated header)
* Move duplicate Mavlink.msg copy to utils.h
* Remove tests that requires connection to FCU
* plugins: imu_pub: Fix PX4 imu/data linear_accelerarion field
  Should fix: `#39 <https://github.com/vooon/mavros/issues/39>`_.
* plugins: imu_pub: Add magnitic covariance
  Trying to move constants with constexpr.
  Related: `#13 <https://github.com/vooon/mavros/issues/13>`_.
* Remove testing info
  Need to remove tests that could not run on build farm.
* Contributors: Vladimir Ermakov

0.5.0 (2014-06-19)
------------------
* Remove mavlink submodule and move it to package dependency
  Bloom release tool don't support git submodules,
  so i've ceate a package as described in http://wiki.ros.org/bloom/Tutorials/ReleaseThirdParty .
  Fix `#35 <https://github.com/vooon/mavros/issues/35>`_.
* plugins: param: add missing gcc 4.6 fix.
* plugins: fix const initializers for gcc 4.6
* plugins: imu_pub: fix const initializers for gcc 4.6
  Fix for build failure devel-hydro-mavros `#4 <https://github.com/vooon/mavros/issues/4>`_.
* Add support for GCC 4.6 (C++0x, ubuntu 12.04)
  I don't use complete c++11, so we could switch to c++0x if it supported.
* plugins: rc_io: Add override rcin service
  Fix: `#22 <https://github.com/vooon/mavros/issues/22>`_.
* plugins: sys_status: fix timeouts
  Fix `#26 <https://github.com/vooon/mavros/issues/26>`_.
* plugins: sys_status: add set stream rate service
  Some additional testing required.
  Fix `#23 <https://github.com/vooon/mavros/issues/23>`_.
* Remove unused boost libarary: timer
  Build on jenkins for hydro failed on find boost_timer.
* 0.4.1
* Add changelog for releasing via bloom

0.4.1 (2014-06-11)
------------------
* node: Show serial link status in diag
  Now 'FCU connection' shows actual status of connection (HEARTBEATS).
* Fix `#29 <https://github.com/vooon/mavros/issues/29>`_. Autostart mavlink via USB on PX4
  Changes mavconn interface, adds new parameter.
* Fix installation rules.
  Fix `#31 <https://github.com/vooon/mavros/issues/31>`_.
* Setup UDP transport for /mavlink messages
* Fix mavlink dialect selection
  Fix `#28 <https://github.com/vooon/mavros/issues/28>`_.
* Add link to wiki.ros.org
  Part of `#27 <https://github.com/vooon/mavros/issues/27>`_.

0.4.0 (2014-06-07)
------------------
* Release 0.4.0
  And some docs for CommandPlugin.
* plugins: command: Command shortcuts
  Fix `#12 <https://github.com/vooon/mavros/issues/12>`_.
* plugins: command: Add ACK waiting list
  Part of `#12 <https://github.com/vooon/mavros/issues/12>`_.
* plugins: command: Initial naive realization.
  Partial `#12 <https://github.com/vooon/mavros/issues/12>`_, `#25 <https://github.com/vooon/mavros/issues/25>`_.
* mavconn: Fix build on Odroid with Ubuntu 13.10
  Fix `#24 <https://github.com/vooon/mavros/issues/24>`_.
* plugins: rc_io: initial add RC_IO plugin
  Topics:
  * ~/rc/in -- FCU RC inputs in raw microseconds
  * ~/rc/out -- FCU Servo outputs
  Fix `#17 <https://github.com/vooon/mavros/issues/17>`_.
  Partiall `#22 <https://github.com/vooon/mavros/issues/22>`_.
* Fix installation wstool command.
  `wstool set`, not `wstool add`.
* Add installation notes to README
  Installing pymavlink is not required, but try if errors.
* Fix headers in README.md
* ros_udp: New node for UDP proxing
  Add some examples to README.md.
  Fix `#21 <https://github.com/vooon/mavros/issues/21>`_.
* sys_status: Add state publication
  Fix `#16 <https://github.com/vooon/mavros/issues/16>`_.
* sys_status: Sent HEARTBEAT if conn_heartbeat > 0
  Fix `#20 <https://github.com/vooon/mavros/issues/20>`_.
* sys_status: add sensor diagnostic
  See `#16 <https://github.com/vooon/mavros/issues/16>`_.
* sys_status: Add battery status monitoring
  Fix `#19 <https://github.com/vooon/mavros/issues/19>`_, partial `#16 <https://github.com/vooon/mavros/issues/16>`_.
* sys_status: HWSTATUS support
  Fix `#18 <https://github.com/vooon/mavros/issues/18>`_, partial `#20 <https://github.com/vooon/mavros/issues/20>`_.
* plugins: imu_pub: Add RAW_IMU, SCALED_IMU and SCALED_PRESSURE handlers
  Fix `#13 <https://github.com/vooon/mavros/issues/13>`_. Refactor message processing.
  Combination of used messages:
  On APM: ATTITUDE + RAW_IMU + SCALED_PRESSURE
  On PX4: ATTITUDE + HIGHRES_IMU
  On other: ATTITUDE + (RAW_IMU|SCALED_IMU + SCALED_PRESSURE)|HIGHRES_IMU
  Published topics:
  * ~imu/data         - ATTITUDE + accel data from \*_IMU
  * ~imu/data_raw     - HIGHRES_IMU or SCALED_IMU or RAW_IMU in that order
  * ~imu/mag          - magnetometer (same source as data_raw)
  * ~imu/temperature  - HIGHRES_IMU or SCALED_PRESSURE
  * ~imu/atm_pressure - same as temperature
* Update readme
* mavwp: Add --pull option for 'show' operation.
  Reread waypoints before show.
* MissionPlanner use format QGC WPL, Fix `#15 <https://github.com/vooon/mavros/issues/15>`_
  Code cleanup;
* Update mavlink version.
* Update mavlink version
* mavparam: fix `#14 <https://github.com/vooon/mavros/issues/14>`_ support for QGC param files
* mavwp: Add mavwp to install

0.3.0 (2014-03-23)
------------------
* Release 0.3.0
* mavwp: Add MAV mission manipulation tool
  Uses WaypointPlugin ROS API for manipulations with FCU mission.
  - show -- show current mission table
  - pull -- update waypoint table
  - dump -- update and save to file
  - load -- loads mission from file
  - clear -- delete all waypoints
  - setcur -- change current waypoint
  - goto -- execute guided goto command (only APM)
  Currently supports QGroundControl format only.
* plugins: wp: Add GOTO, update documentation
* plugins: wp: Auto pull
* plugins: wp: SetCurrent & Clear now works
* plugins: wp: Push service works
* plugins: wp: push almost done
* plugins: wp: Pull done
* plugins: param: remove unused ptr
* plugins: wp: mission pull almost done
* plugins: wp: Add convertors & handlers
* plugins: Waypoint plugin initial
* Use C++11 feuture - auto type
* plugins: refactor context & link to single UAS class
  UAS same functions as in QGC.
* plugins: Add msgs and srvs for Waypoint plugin
* Update mavlink library
* Update mavlink version
* mavparam: Fix for DroidPlanner param files & cleanup
  DroidPlanner adds some spaces, don't forget to strip it out.
  Cleanup unused code from Parameter class.

0.2.0 (2014-01-29)
------------------
* mavparam: Add MAV parameter manipulation tool
  Uses ParamPlugin ROS API for manipulating with fcu params.
  - load -- load parameter from file
  - dump -- dump parameter to file
  - get -- get parameter
  - set -- set parameter
  Currently supports MissionPlanner format only.
  But DroidPlanner uses same format.
* Update README and documentation
* plugins: param: implement ~param/push service
  Also implement sync for rosparam:
  - ~param/pull service pulls data to rosparam
  - ~param/push service send data from rosparam
  - ~param/set service update rosparam if success
* plugins: param: implement ~param/set service
* plugins: param: implement ~param/get service
* plugins: param: Implement automatic param list requesting
* plugins: use recursive_mutex everywhere
* plugins: param now automaticly requests data after connect
* plugins: Add common io_service for plugins, implement connection timeout
  Some plugin require some delayed processes. Now we can use
  boost::asio::\*timer.
  New parameter:
  - ~/conn_timeout connection timeout in seconds
* plugins: add param services
* mavconn: set thread names
  WARNING: pthread systems only (BSD/Linux)
* plugins: implement parameters fetch service
* plugins: fix string copying from mavlink msg
* plugins: Setup target in mav_context
  New params:
  - ~target_system_id - FCU System ID
  - ~target_component_id - FCU Component ID
* plugins: IMU Pub: add stdev parameters, change topic names.
  Add parameters:
  - ~imu/linear_acceleration_stdev - for linear acceleration covariance
  - ~imu/angular_velocity_stdev - for angular covariance
  - ~imu/orientation_stdev - for orientation covariance
  Change topic names (as in other IMU drivers):
  - ~imu -> ~/imu/data
  - ~raw/imu -> ~/imu/data_raw
* plugins: Params initial dirty plugin
* Fix mavlink dialect choice.
* plugins: Add context storage for automatic quirk handling
  ArduPlilot requires at least 2 quirks:
  - STATUSTEXT severity levels
  - parameter values is float
* Implement MAVLink dialect selection
  ArduPilotMega is default choice.
* doc: add configuration for rosdoc_lite

0.1.0 (2014-01-05)
------------------
* Version 0.1.0
  Milestone 1: all features from mavlink_ros
  package.xml was updated.
* Fix typo and add copyright string
  NOTE: Please check typos before coping and pasting :)
* plugins: gps: Add GPS_RAW_INT handler
  GPS_STATUS not supported by APM:Plane.
  ROS dosen't have standard message for satellites information.
* mavconn: small debug changes
  Limit no GCS message to 10 sec.
* node: Terminate node on serial port errors
* plugins: Add GPS plugin
  SYSTEM_TIME to TimeReference support.
  TODO GPS fix.
* Fix build and update MAVLink library
* plugins: sys_status: Add SYSTEMTEXT handler
  Two modes:
  - standard MAV_SEVERITY values
  - APM:Plane (default)
  TODO: add mavlink dialect selection option
* plugins: add some header doxygen tags
  Add license to Dummy.cpp (plugin template).
* plugins: sys_status: Add MEMINFO handler
  MEMINFO from ardupilotmega.xml message definition.
  Optional.
* update README
* update TODO
* plugins: Add imu_pub plugin.
  Publish ATTITUDE and HIGHRES_IMU data.
  HIGHRES__IMU not tested: Ardupilot sends ATTITUDE only :(
* node: publish Mavlink.msg only if listners > 0
* plugins: Add sys_status plugin.
  Initial.
* plugins: implement loading & rx routing
* plugins: initial
* node: Add diagnostics for mavlink interfaces
* mavconn: add information log wich serial device we use.
* mavconn: fix overloaded MAVConn*::send_message(msg)
* mavros: Add mavros_node (currently serial-ros-udp bridge)
  Message paths:
  Serial -+-> ROS /mavlink/from
  +-> UDP gcs_host:port
  ROS /mavlink/to    -+-> Serial
  UDP bind_host:port -+
* Add README and TODO files.
* mavconn: fix MAVConnUDP, add mavudpproxy test
  mavudpproxy -- connection proxy for QGroundControl, also used as test
  for MAVConnUDP and MAVConnSerial.
* mavconn: add UDP support class
* mavconn: fix: should use virtual destructor in interface class
* mavconn: add getters/setters for sys_id, comp_id; send_message return.
* mavconn: simple test.
  tested with APM:Plane: works.
* mavconn: fix linking
* mavconn: serial interface
* Add mavconn library prototype
  mavconn - handles MAVLink connections via Serial, UDP and TCP.
* Add MAVLink library + build script
* Initial
  Import Mavlink.msg from mavlink_ros package
  ( https://github.com/mavlink/mavlink_ros ).
* Contributors: Vladimir Ermakov

