^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2023-09-09)
------------------
* msgs: move generator code
* cog: regenerate all
* Merge branch 'master' into ros2
  * master:
  1.17.0
  update changelog
  cog: regenerate all
  Bugfix/update map origin with home position (`#1892 <https://github.com/mavlink/mavros/issues/1892>`_)
  mavros: Remove extra ';'
  mavros_extras: Fix some init order warnings
  Suppress warnings from included headers
  1.16.0
  update changelog
  made it such that the gp_origin topic published latched.
  use hpp instead of deprecated .h pluginlib headers
* 1.17.0
* update changelog
* cog: regenerate all
* local takeoff and land topics (`#1890 <https://github.com/mavlink/mavros/issues/1890>`_)
  * local takeoff and land topics
  * vector3 position type, rename to TOLLocal
  * remove auto include line
* Merge pull request `#1871 <https://github.com/mavlink/mavros/issues/1871>`_ from Vladislavert/feature/optical_flow_msg
  Addition of New OpticalFlow.msg
* Added geometry_msgs/Vector3 to OpticalFlow.msg
* Added vectors to the message OpticalFlow.msg
* Added message optical flow
* 1.16.0
* update changelog
* Contributors: Ido Guzi, Vladimir Ermakov, Vladislavert

2.5.0 (2023-05-05)
------------------

2.4.0 (2022-12-30)
------------------
* msgs: re-generate
* Merge branch 'master' into ros2
  * master:
  1.15.0
  update changelog
  ci: update actions
  Implement debug float array handler
  mavros_extras: Fix a sequence point warning
  mavros_extras: Fix a comparison that shouldn't be bitwise
  mavros: Fix some warnings
  mavros_extras: Fix buggy check for lat/lon ignored
  libmavconn: fix MAVLink v1.0 output selection
* 1.15.0
* update changelog
* Merge pull request `#1811 <https://github.com/mavlink/mavros/issues/1811>`_ from scoutdi/debug-float-array
  Implement debug float array handler
* Implement debug float array handler
  Co-authored-by: Morten Fyhn Amundsen <morten.f.amundsen@scoutdi.com>
* Contributors: Sverre Velten Rothmund, Vladimir Ermakov

2.3.0 (2022-09-24)
------------------
* Merge branch 'master' into ros2
  * master:
  1.14.0
  update changelog
  scripts: waypoint and param files are text, not binary
  libmavconn: fix MAVLink v1.0 output selection
  plugins: add guided_target to accept offboard position targets
  add cmake module path for geographiclib on debian based systems
  use already installed FindGeographicLib.cmake
* 1.14.0
* update changelog
* Contributors: Vladimir Ermakov

2.2.0 (2022-06-27)
------------------
* Merge branch 'master' into ros2
  * master:
  mount_control.cpp: detect MOUNT_ORIENTATION stale messages
  ESCTelemetryItem.msg: correct RPM units
  apm_config.yaml: add mount configuration
  sys_status.cpp fix free memory for values > 64KiB
  uncrustify cellular_status.cpp
  Add CellularStatus plugin and message
  *_config.yaml: document usage of multiple batteries diagnostics
  sys_status.cpp: fix compilation
  sys_status.cpp: support diagnostics on up-to 10 batteries
  sys_status.cpp: do not use harcoded constants
  sys_status.cpp: Timeout on MEMINFO and HWSTATUS mavlink messages and publish on the diagnostics
  sys_status.cpp: fix enabling of mem_diag and hwst_diag
  sys_status.cpp: Do not use battery1 voltage as voltage for all other batteries (bugfix).
  sys_status.cpp: ignore sys_status mavlink messages from gimbals
  mount_control.cpp: use mount_nh for params to keep similarities with other plugins set diag settings before add()
  sys_status.cpp: remove deprecated BATTERY2 mavlink message support
  Mount control plugin: add configurable diagnostics
  Bugfix: increment_f had no value asigned when input LaserScan was bigger than obstacle.distances.size()
  Bugfix: wrong interpolation when the reduction ratio (scale_factor) is not integer.
  Disable startup_px4_usb_quirk in px4_config.yaml
* msgs: support humble

2.1.1 (2022-03-02)
------------------

2.1.0 (2022-02-02)
------------------
* Merge branch 'master' into ros2
  * master:
  1.13.0
  update changelog
  py-lib: fix compatibility with py3 for Noetic
  re-generate all coglets
  test: add checks for ROTATION_CUSTOM
  lib: Fix rotation search for CUSTOM
  Removed CamelCase for class members.  Publish to "report"
  More explicitly state "TerrainReport" to allow for future extension of the plugin to support other terrain messages
  Fixed callback name to match `handle\_{MESSAGE_NAME.lower()}` convention
  Add extra MAV_FRAMES to waypoint message as defined in https://mavlink.io/en/messages/common.html
  Fixed topic names to match more closely what other plugins use.  Fixed a typo.
  Add plugin for reporting terrain height estimate from FCU
  1.12.2
  update changelog
  Set time/publish_sim_time to false by default
  plugin: setpoint_raw: move getParam to initializer
  extras: trajectory: backport `#1667 <https://github.com/mavlink/mavros/issues/1667>`_
* 1.13.0
* update changelog
* Merge pull request `#1690 <https://github.com/mavlink/mavros/issues/1690>`_ from mavlink/fix-enum_sensor_orientation
  Fix enum sensor_orientation
* re-generate all coglets
* Merge pull request `#1680 <https://github.com/mavlink/mavros/issues/1680>`_ from AndersonRayner/new_mav_frames
  Add extra MAV_FRAMES to waypoint message
* Merge pull request `#1677 <https://github.com/mavlink/mavros/issues/1677>`_ from AndersonRayner/add_terrain
  Add plugin for reporting terrain height estimate from the FCU
* More explicitly state "TerrainReport" to allow for future extension of the plugin to support other terrain messages
* Add extra MAV_FRAMES to waypoint message as defined in https://mavlink.io/en/messages/common.html
* Add plugin for reporting terrain height estimate from FCU
* 1.12.2
* update changelog
* Merge branch 'master' into ros2
  * master:
  1.12.1
  update changelog
  mavconn: fix connection issue introduced by `#1658 <https://github.com/mavlink/mavros/issues/1658>`_
  mavros_extras: Fix some warnings
  mavros: Fix some warnings
* 1.12.1
* update changelog
* Contributors: Vladimir Ermakov, matt

2.0.5 (2021-11-28)
------------------
* Merge branch 'master' into ros2
  * master:
  1.12.0
  update changelog
  Fix multiple bugs
  lib: fix mission frame debug print
  extras: distance_sensor: revert back to zero quaternion
* 1.12.0
* update changelog
* extras: fix some more lint warns
* msgs: update conversion header
* Merge branch 'master' into ros2
  * master:
  1.11.1
  update changelog
  lib: fix build
* 1.11.1
* update changelog
* Merge branch 'master' into ros2
  * master:
  1.11.0
  update changelog
  lib: fix ftf warnings
  msgs: use pragmas to ignore unaligned pointer warnings
  extras: landing_target: fix misprint
  msgs: fix convert const
  plugin: setpoint_raw: fix misprint
  msgs: try to hide 'unaligned pointer' warning
  plugin: sys: fix compillation error
  plugin: initialize quaternions with identity
  plugin: sys: Use wall timers for connection management
  Use meters for relative altitude
  distance_sensor: Initialize sensor orientation quaternion to zero
  Address review comments
  Add camera plugin for interfacing with mavlink camera protocol
* 1.11.0
* update changelog
* msgs: use pragmas to ignore unaligned pointer warnings
* msgs: fix convert const
* msgs: try to hide 'unaligned pointer' warning
* Merge pull request `#1651 <https://github.com/mavlink/mavros/issues/1651>`_ from Jaeyoung-Lim/pr-image-capture-plugin
  Add camera plugin for interfacing with mavlink camera protocol
* Address review comments
* Add camera plugin for interfacing with mavlink camera protocol
  Add camera image captured message for handling camera trigger information
* Merge branch 'master' into ros2
  * master:
  msgs: add yaw field to GPS_INPUT
* msgs: add yaw field to GPS_INPUT
* Contributors: Jaeyoung-Lim, Vladimir Ermakov

2.0.4 (2021-11-04)
------------------
* Merge branch 'master' into ros2
  * master:
  1.10.0
  prepare release
* 1.10.0
* prepare release
* Merge branch 'master' into ros2
  * master:
  msgs: update gpsraw to have yaw field
* msgs: update gpsraw to have yaw field
* Merge branch 'master' into ros2
  * master: (25 commits)
  Remove reference
  Catch std::length_error in send_message
  Show ENOTCONN error instead of crash
  Tunnel: Check for invalid payload length
  Tunnel.msg: Generate enum with cog
  mavros_extras: Create tunnel plugin
  mavros_msgs: Add Tunnel message
  MountControl.msg: fix copy-paste
  sys_time.cpp: typo
  sys_time: publish /clock for simulation times
  1.9.0
  update changelog
  Spelling corrections
  Changed OverrideRCIn to 18 channels
  This adds functionality to erase all logs on the SD card via mavlink
  publish BATTERY2 message as /mavros/battery2 topic
  Mavlink v2.0 specs for RC_CHANNELS_OVERRIDE accepts upto 18 channels. The plugin publishes channels 9 to 18 if the FCU protocol version is 2.0
  Added NAV_CONTROLLER_OUTPUT Plugin
  Added GPS_INPUT plugin
  Update esc_status plugin with datatype change on MAVLink.
  ...
* Merge pull request `#1625 <https://github.com/mavlink/mavros/issues/1625>`_ from scoutdi/tunnel-plugin
  Plugin for TUNNEL messages
* Tunnel.msg: Generate enum with cog
* mavros_msgs: Add Tunnel message
* Merge pull request `#1623 <https://github.com/mavlink/mavros/issues/1623>`_ from amilcarlucas/pr/more-typo-fixes
  More typo fixes
* MountControl.msg: fix copy-paste
* 1.9.0
* update changelog
* Merge pull request `#1616 <https://github.com/mavlink/mavros/issues/1616>`_ from amilcarlucas/pr/RC_CHANNELS-mavlink2-extensions
  Mavlink v2.0 specs for RC_CHANNELS_OVERRIDE accepts upto 18 channels.…
* Changed OverrideRCIn to 18 channels
* Merge pull request `#1617 <https://github.com/mavlink/mavros/issues/1617>`_ from amilcarlucas/pr/NAV_CONTROLLER_OUTPUT-plugin
  Added NAV_CONTROLLER_OUTPUT Plugin
* Merge pull request `#1618 <https://github.com/mavlink/mavros/issues/1618>`_ from amilcarlucas/pr/GPS_INPUT-plugin
  Added GPS_INPUT plugin
* Mavlink v2.0 specs for RC_CHANNELS_OVERRIDE accepts upto 18 channels. The plugin publishes channels 9 to 18 if the FCU protocol version is 2.0
* Added NAV_CONTROLLER_OUTPUT Plugin
* Added GPS_INPUT plugin
* Merge branch 'master' into master
* Update esc_status plugin with datatype change on MAVLink.
  ESC_INFO MAVLink message was updated to have negative temperates and also at a different resolution. This commit updates those changes on this side.
* Remove Mount_Status plugin. Add Status data to Mount_Control plugin. Remove Mount_Status message.
* msgs: re-generate file lists
* Merge branch 'master' into ros2
  * master:
  extras: esc_telemetry: fix build
  extras: fix esc_telemetry centi-volt/amp conversion
  extras: uncrustify all plugins
  plugins: reformat xml
  extras: reformat plugins xml
  extras: fix apm esc_telemetry
  msgs: fix types for apm's esc telemetry
  actually allocate memory for the telemetry information
  fixed some compile errors
  added esc_telemetry plugin
  Reset calibration flag when re-calibrating. Prevent wrong data output.
  Exclude changes to launch files.
  Delete debug files.
  Apply uncrustify changes.
  Set progress array to global to prevent erasing data.
  Move Compass calibration report to extras. Rewrite code based on instructions.
  Remove extra message from CMakeLists.
  Add message and service definition.
  Add compass calibration feedback status. Add service to call the 'Next' button in calibrations.
* msgs: fix types for apm's esc telemetry
* actually allocate memory for the telemetry information
* added esc_telemetry plugin
* Add Mount angles message for communications with ardupilotmega.
* Remove extra message from CMakeLists.
* Add message and service definition.
* Contributors: Abhijith Thottumadayil Jagadeesh, André Filipe, Dr.-Ing. Amilcar do Carmo Lucas, Karthik Desai, Morten Fyhn Amundsen, Ricardo Marques, Russell, Vladimir Ermakov

2.0.3 (2021-06-20)
------------------

2.0.2 (2021-06-20)
------------------

2.0.1 (2021-06-06)
------------------
* Add rcl_interfaces dependency
* Merge branch 'master' into ros2
  * master:
  readme: update
  1.8.0
  update changelog
  Create semgrep-analysis.yml
  Create codeql-analysis.yml
* 1.8.0
* update changelog
* Contributors: Rob Clarke, Vladimir Ermakov

2.0.0 (2021-05-28)
------------------
* msgs: update command codes
* msgs: update param services
* plugins: setpoint_velocity: port to ros2
* Merge branch 'master' into ros2
  * master:
  1.7.1
  update changelog
  re-generate all pymavlink enums
  1.7.0
  update changelog
* mavros: generate plugin list
* Merge branch 'master' into ros2
  * master:
  msgs: re-generate the code
  lib: re-generate the code
  plugins: mission: re-generate the code
  MissionBase: correction to file information
  MissionBase: add copyright from origional waypoint.cpp
  uncrustify
  whitespace
  add rallypoint and geofence plugins to mavros plugins xml
  add rallypoint and geofence plugins to CMakeList
  Geofence: add geofence plugin
  Rallypoint: add rallypoint plugin
  Waypoint: inherit MissionBase class for mission protocol
  MissionBase: breakout mission protocol from waypoint.cpp
  README: Update PX4 Autopilot references
  Fix https://github.com/mavlink/mavros/issues/849
* router: catch DeviceError
* router: weak_ptr segfaults, replace with shared_ptr
* router: implement params handler
* mavros: router decl done
* lib: port enum_to_string
* lib: update sensor_orientation
* msgs: add linter
* libmavconn: start porintg, will use plain asio, without boost
* msgs: remove redundant dependency which result in colcon warning
* msgs: cogify file lists
* Merge pull request `#1186 <https://github.com/mavlink/mavros/issues/1186>`_ from PickNikRobotics/ros2
  mavros_msgs Ros2
* Merge branch 'ros2' into ros2
* msgs: start porting to ROS2
* fixing cmakelists
* updating msg and srv list
* reenable VfrHud once renamed to match ROS2 conventions
  add ros1_bridge mapping rule for renamed VfrHud message
* make mavro_msgs compile in ROS 2
* Contributors: Mikael Arguedas, Mike Lautman, Vladimir Ermakov

1.17.0 (2023-09-09)
-------------------
* cog: regenerate all
* Contributors: Vladimir Ermakov

1.16.0 (2023-05-05)
-------------------

1.15.0 (2022-12-30)
-------------------
* Merge pull request `#1811 <https://github.com/mavlink/mavros/issues/1811>`_ from scoutdi/debug-float-array
  Implement debug float array handler
* Implement debug float array handler
  Co-authored-by: Morten Fyhn Amundsen <morten.f.amundsen@scoutdi.com>
* Contributors: Sverre Velten Rothmund, Vladimir Ermakov

1.14.0 (2022-09-24)
-------------------
* Merge pull request `#1742 <https://github.com/mavlink/mavros/issues/1742>`_ from amilcarlucas/correct_rpm_units
  ESCTelemetryItem.msg: correct RPM units
* ESCTelemetryItem.msg: correct RPM units
* Merge pull request `#1727 <https://github.com/mavlink/mavros/issues/1727>`_ from BV-OpenSource/pr-cellular-status
  Pr cellular status
* Add CellularStatus plugin and message
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Rui Mendes, Vladimir Ermakov

1.13.0 (2022-01-13)
-------------------
* Merge pull request `#1690 <https://github.com/mavlink/mavros/issues/1690>`_ from mavlink/fix-enum_sensor_orientation
  Fix enum sensor_orientation
* re-generate all coglets
* Merge pull request `#1680 <https://github.com/mavlink/mavros/issues/1680>`_ from AndersonRayner/new_mav_frames
  Add extra MAV_FRAMES to waypoint message
* Merge pull request `#1677 <https://github.com/mavlink/mavros/issues/1677>`_ from AndersonRayner/add_terrain
  Add plugin for reporting terrain height estimate from the FCU
* More explicitly state "TerrainReport" to allow for future extension of the plugin to support other terrain messages
* Add extra MAV_FRAMES to waypoint message as defined in https://mavlink.io/en/messages/common.html
* Add plugin for reporting terrain height estimate from FCU
* Contributors: Vladimir Ermakov, matt

1.12.2 (2021-12-12)
-------------------

1.12.1 (2021-11-29)
-------------------

1.12.0 (2021-11-27)
-------------------

1.11.1 (2021-11-24)
-------------------

1.11.0 (2021-11-24)
-------------------
* msgs: use pragmas to ignore unaligned pointer warnings
* msgs: fix convert const
* msgs: try to hide 'unaligned pointer' warning
* Merge pull request `#1651 <https://github.com/mavlink/mavros/issues/1651>`_ from Jaeyoung-Lim/pr-image-capture-plugin
  Add camera plugin for interfacing with mavlink camera protocol
* Address review comments
* Add camera plugin for interfacing with mavlink camera protocol
  Add camera image captured message for handling camera trigger information
* msgs: add yaw field to GPS_INPUT
* Contributors: Jaeyoung-Lim, Vladimir Ermakov

1.10.0 (2021-11-04)
-------------------
* msgs: update gpsraw to have yaw field
* Merge pull request `#1625 <https://github.com/mavlink/mavros/issues/1625>`_ from scoutdi/tunnel-plugin
  Plugin for TUNNEL messages
* Tunnel.msg: Generate enum with cog
* mavros_msgs: Add Tunnel message
* Merge pull request `#1623 <https://github.com/mavlink/mavros/issues/1623>`_ from amilcarlucas/pr/more-typo-fixes
  More typo fixes
* MountControl.msg: fix copy-paste
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Morten Fyhn Amundsen, Vladimir Ermakov

1.9.0 (2021-09-09)
------------------
* Merge pull request `#1616 <https://github.com/mavlink/mavros/issues/1616>`_ from amilcarlucas/pr/RC_CHANNELS-mavlink2-extensions
  Mavlink v2.0 specs for RC_CHANNELS_OVERRIDE accepts upto 18 channels.…
* Changed OverrideRCIn to 18 channels
* Merge pull request `#1617 <https://github.com/mavlink/mavros/issues/1617>`_ from amilcarlucas/pr/NAV_CONTROLLER_OUTPUT-plugin
  Added NAV_CONTROLLER_OUTPUT Plugin
* Merge pull request `#1618 <https://github.com/mavlink/mavros/issues/1618>`_ from amilcarlucas/pr/GPS_INPUT-plugin
  Added GPS_INPUT plugin
* Mavlink v2.0 specs for RC_CHANNELS_OVERRIDE accepts upto 18 channels. The plugin publishes channels 9 to 18 if the FCU protocol version is 2.0
* Added NAV_CONTROLLER_OUTPUT Plugin
* Added GPS_INPUT plugin
* Merge branch 'master' into master
* Update esc_status plugin with datatype change on MAVLink.
  ESC_INFO MAVLink message was updated to have negative temperates and also at a different resolution. This commit updates those changes on this side.
* Remove Mount_Status plugin. Add Status data to Mount_Control plugin. Remove Mount_Status message.
* msgs: fix types for apm's esc telemetry
* actually allocate memory for the telemetry information
* added esc_telemetry plugin
* Add Mount angles message for communications with ardupilotmega.
* Remove extra message from CMakeLists.
* Add message and service definition.
* Contributors: Abhijith Thottumadayil Jagadeesh, André Filipe, Dr.-Ing. Amilcar do Carmo Lucas, Karthik Desai, Ricardo Marques, Russell, Vladimir Ermakov

1.8.0 (2021-05-05)
------------------

1.7.1 (2021-04-05)
------------------
* re-generate all pymavlink enums
* Contributors: Vladimir Ermakov

1.7.0 (2021-04-05)
------------------
* msgs: re-generate the code
* Contributors: Vladimir Ermakov

1.6.0 (2021-02-15)
------------------

1.5.2 (2021-02-02)
------------------

1.5.1 (2021-01-04)
------------------

1.5.0 (2020-11-11)
------------------
* mavros_msgs/VehicleInfo: Add flight_custom_version field
  Mirroring the field in the corresponding MAVLink message.
* mavros_msgs/State: Fix PX4 flight mode constants
  Turns out ROS message string literals don't need quotes,
  so adding quotes creates strings including the quotes.
* mavros_msgs/State: Add flight mode constants
* mavros_msgs: Don't move temporary objects
* Contributors: Morten Fyhn Amundsen

1.4.0 (2020-09-11)
------------------
* play_tune: Assign tune format directly
* play_tune: Write new plugin
* Contributors: Morten Fyhn Amundsen

1.3.0 (2020-08-08)
------------------
* Add esc_status plugin.
* Add gps_status plugin to publish GPS_RAW and GPS_RTK messages from FCU.
  The timestamps for the gps_status topics take into account the mavlink time and uses the convienence function
* adding support for publishing rtkbaseline msgs over ROS
* Contributors: CSCE439, Dr.-Ing. Amilcar do Carmo Lucas, Ricardo Marques

1.2.0 (2020-05-22)
------------------
* add yaw to CMD_DO_SET_HOME
* Contributors: David Jablonski

1.1.0 (2020-04-04)
------------------

1.0.0 (2020-01-01)
------------------

0.33.4 (2019-12-12)
-------------------
* Splitted the message fields.
* Updated esimator status msg according to the new cog based definition of estimator status.
* Added comments to msg.
* Added new line char at end of message.
* Added a publisher for estimator status message received from mavlink in sys_status.
* Contributors: saifullah3396

0.33.3 (2019-11-13)
-------------------

0.33.2 (2019-11-13)
-------------------

0.33.1 (2019-11-11)
-------------------
* resolved merge conflict
* Contributors: David Jablonski

0.33.0 (2019-10-10)
-------------------
* Add vtol transition service
* Apply comments
* Add mount configure service message
* cog: Update all generated code
* added manual flag to mavros/state
* use header.stamp to fill mavlink msg field time_usec
* use cog for copy
* adapt message and plugin after mavlink message merge
* rename message and adjust fields
* add component id to mavros message to distinguish ROS msgs from different systems
* component_status message and plugin draft
* Contributors: David Jablonski, Jaeyoung-Lim, Vladimir Ermakov, baumanta

0.32.2 (2019-09-09)
-------------------

0.32.1 (2019-08-08)
-------------------

0.32.0 (2019-07-06)
-------------------
* add mav_cmd associated with each point in trajectory plugin
* Use MountControl Msg
* Define new MountControl.msg
* Contributors: Jaeyoung-Lim, Martina Rivizzigno

0.31.0 (2019-06-07)
-------------------
* mavros_msgs: LandingTarget: update msg description link
* extras: landing target: improve usability and flexibility
* Contributors: TSC21

0.30.0 (2019-05-20)
-------------------

0.29.2 (2019-03-06)
-------------------

0.29.1 (2019-03-03)
-------------------
* All: catkin lint files
* mavros_msgs: Fix line endings for OpticalFlowRad message
* Contributors: Pierre Kancir, sfalexrog

0.29.0 (2019-02-02)
-------------------
* Fix broken documentation URLs
* Merge branch 'master' into param-timeout
* mavros_extras: Wheel odometry plugin updated according to the final mavlink WHEEL_DISTANCE message.
* mavros_msgs: Float32ArrayStamped replaced by WheelOdomStamped.
* mavros_msgs: Float32ArrayStamped message added.
  For streaming timestamped data from FCU sensors (RPM, WHEEL_DISTANCE, etc.)
* msgs: Fix message id type, mavlink v2 uses 24 bit msg ids
* mavros_msgs: add MessageInterval.srv to CMakeLists
* sys_status: add set_message_interval service
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Pavlo Kolomiiets, Randy Mackay, Vladimir Ermakov

0.28.0 (2019-01-03)
-------------------
* plugin:param: publish new param value
* Merge pull request `#1148 <https://github.com/mavlink/mavros/issues/1148>`_ from Kiwa21/pr-param-value
  param plugin : add msg and publisher to catch latest param value
* msgs: update Header
* sys_state: Small cleanup of `#1150 <https://github.com/mavlink/mavros/issues/1150>`_
* VehicleInfo : add srv into sys_status plugin to request basic info from vehicle
* mavros_msgs/msg/LogData.msg: Define "offset" field to be of type uint32
* param plugin : add msg and publisher to catch latest param value
* style clean up
* Use component_id to determine message sender
* change message name from COMPANION_STATUS to COMPANION_PROCESS_STATUS
* change message to include pid
* Change from specific avoidance status message to a more generic companion status message
* Add message for avoidance status
* Contributors: Gregoire Linard, Vladimir Ermakov, baumanta, mlvov

0.27.0 (2018-11-12)
-------------------
* Add service to send mavlink TRIGG_INTERVAL commands
  Adapt trigger_control service to current mavlink cmd spec. Add a new service to change trigger interval and integration time
* Contributors: Moritz Zimmermann

0.26.3 (2018-08-21)
-------------------
* fixup! 5a4344a2dcedc157f93b620cebd2e0b273ec24be
* mavros_msgs: Add msg and srv files related to log transfer
* Contributors: mlvov

0.26.2 (2018-08-08)
-------------------
* Updating the gps_rtk plugin to fit mavros guidelines:
  - Updating max_frag_len to allow changes in size in MAVLink seamlessly
  - Using std::copy instead of memset
  - Zero fill with std::fill
  - Preapply the sequence flags
  - Use of std iterators
  - Add the maximal data size in the mavros_msgs
* Renaming the GPS RTK module, Adding fragmentation, Changing the RTCM message
* RTK Plugin; to forward RTCM messages
  Signed-off-by: Alexis Paques <alexis.paques@gmail.com>
* Contributors: Alexis Paques

0.26.1 (2018-07-19)
-------------------

0.26.0 (2018-06-06)
-------------------
* mavros_msgs : add timesync status message
* Contributors: Mohammed Kabir

0.25.1 (2018-05-14)
-------------------

0.25.0 (2018-05-11)
-------------------
* trajectory: add time_horizon field
* change message name from ObstacleAvoidance to Trajectory since it is
  general enough to support any type of trajectory
* CMakeLists: add ObstacleAvoidance message
* add ObstacleAvoidance message
* msgs: Update message doc link
* CommandCode: update list of available commands on MAV_CMD enum (`#995 <https://github.com/mavlink/mavros/issues/995>`_)
* Contributors: Martina, Nuno Marques, Vladimir Ermakov

0.24.0 (2018-04-05)
-------------------
* Add ability to send STATUSTEXT messages
* Contributors: Anass Al

0.23.3 (2018-03-09)
-------------------

0.23.2 (2018-03-07)
-------------------

0.23.1 (2018-02-27)
-------------------

0.23.0 (2018-02-03)
-------------------

0.22.0 (2017-12-11)
-------------------
* SetMavFrame.srv: add FRAME\_ prefix
* Add cog for SetMavFrame.srv
* Setpoints: add service to specify frame
* Contributors: Pierre Kancir, khancyr

0.21.5 (2017-11-16)
-------------------

0.21.4 (2017-11-01)
-------------------

0.21.3 (2017-10-28)
-------------------
* plugin waypoints: Use stamped message
* add debug plugin
* Contributors: TSC21, Vladimir Ermakov

0.21.2 (2017-09-25)
-------------------

0.21.1 (2017-09-22)
-------------------

0.21.0 (2017-09-14)
-------------------
* plugin waypoint: Rename current seq in wp list message
* waypoint: Publish current waypoint seq
* waypoint partial: code style cleanup
* waypoint partial: extend existing service
* Partial waypoint: added wp_transfered to push partial service response
* Partial waypoint: added partial updating to mavwp
* Contributors: James Mare, James Stewart, Vladimir Ermakov

0.20.1 (2017-08-28)
-------------------

0.20.0 (2017-08-23)
-------------------
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
* update ExtendedState with new MAV_LANDED_STATE enum
* Contributors: Nicklas Stockton, Nuno Marques, Vladimir Ermakov

0.19.0 (2017-05-05)
-------------------
* msgs: Add cog script to finish ADSBVehicle.msg
* extras: Add ADSB plugin
* plugin `#695 <https://github.com/mavlink/mavros/issues/695>`_: Fix plugin
* plugin: Add home_position
* Contributors: Nuno Marques, Vladimir Ermakov

0.18.7 (2017-02-24)
-------------------
* trigger interface : rename to cycle_time to be consistent with PX4
* Contributors: Kabir Mohammed

0.18.6 (2017-02-07)
-------------------
* Plugins: system_status change status field to system_status
  Add comment to State.msg for system_status enum
* Plugins: add system_status to state message
* Contributors: Pierre Kancir

0.18.5 (2016-12-12)
-------------------

0.18.4 (2016-11-11)
-------------------
* msgs: Fix `#609 <https://github.com/mavlink/mavros/issues/609>`_
* add hil_actuator_controls mavlink message
* Contributors: Beat Kung, Vladimir Ermakov

0.18.3 (2016-07-07)
-------------------

0.18.2 (2016-06-30)
-------------------

0.18.1 (2016-06-24)
-------------------

0.18.0 (2016-06-23)
-------------------
* Adding anchor to the HIL_CONTROLS message reference link
* Utilizing synchronise_stamp and adding reference to MAVLINK msg documentation
* Added a plugin that publishes HIL_CONTROLS as ROS messages
* node: Rename plugib base class - API incompatible to old class
* msgs `#543 <https://github.com/mavlink/mavros/issues/543>`_: Update for MAVLink 2.0
* Contributors: Pavel, Vladimir Ermakov

0.17.3 (2016-05-20)
-------------------

0.17.2 (2016-04-29)
-------------------

0.17.1 (2016-03-28)
-------------------

0.17.0 (2016-02-09)
-------------------
* rebased with master
* Contributors: francois

0.16.6 (2016-02-04)
-------------------

0.16.5 (2016-01-11)
-------------------

0.16.4 (2015-12-14)
-------------------
* Update mavlink message documentation links
* remove "altitude\_" prefix from members
* implemented altitude plugin
* Contributors: Andreas Antener, Vladimir Ermakov

0.16.3 (2015-11-19)
-------------------

0.16.2 (2015-11-17)
-------------------

0.16.1 (2015-11-13)
-------------------

0.16.0 (2015-11-09)
-------------------
* msgs `#418 <https://github.com/mavlink/mavros/issues/418>`_: add message for attitude setpoints
* plugin: waypoint fix `#414 <https://github.com/mavlink/mavros/issues/414>`_: remove GOTO service.
  It is replaced with more standard global setpoint messages.
* msgs `#415 <https://github.com/mavlink/mavros/issues/415>`_: Add message for raw global setpoint
* msgs `#402 <https://github.com/mavlink/mavros/issues/402>`_: PositionTarget message type
* setting constant values and reference docs
* pass new extended state to ros
* msgs `#371 <https://github.com/mavlink/mavros/issues/371>`_: add missing message
* msgs `#371 <https://github.com/mavlink/mavros/issues/371>`_: add HomePosition message
* Contributors: Andreas Antener, Vladimir Ermakov

0.15.0 (2015-09-17)
-------------------
* msgs `#286 <https://github.com/mavlink/mavros/issues/286>`_: fix bug with packet header.
* msgs `#286 <https://github.com/mavlink/mavros/issues/286>`_: Add valid flag and checksum to Mavlink.msg
* plugin: manual_control: Use shared pointer message
  Fix alphabetic order of msgs.
* removed old commend in .msg file
* Add MANUAL_CONTROL handling with new plugin
* Contributors: Vladimir Ermakov, v01d

0.14.2 (2015-08-20)
-------------------

0.14.1 (2015-08-19)
-------------------

0.14.0 (2015-08-17)
-------------------
* msgs: Add mixer group constants ActuatorControl
* msgs: Add notes to message headers.
* msgs: sort msgs in alphabetical order
* msgs: use std::move for mavlink->ros convert
* msgs: add note about convert function
* msgs: change description, make catkin lint happy
* msgs: move convert functions to msgs package.
* msgs: fix message generator and runtime depent tags
* msgs: remove never used Mavlink.fromlcm field.
* msgs: add package name for all non basic types
* msgs: fix msgs build
* msgs `#354 <https://github.com/mavlink/mavros/issues/354>`_: move all messages to mavros_msgs package.
* Contributors: Vladimir Ermakov
