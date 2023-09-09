^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_extras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2023-09-09)
------------------
* switch to use tf2_eigen.hpp, but that drops support for EOL distros
* ament uncrustify
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
* Merge pull request `#1889 <https://github.com/mavlink/mavros/issues/1889>`_ from MKargus0/feature/fix_landing_target_time_conversion
  Fixed header.stamp conversion in landing target
* fixed style
* fixed header.stamp conversion in landing target
* Merge pull request `#1871 <https://github.com/mavlink/mavros/issues/1871>`_ from Vladislavert/feature/optical_flow_msg
  Addition of New OpticalFlow.msg
* Added geometry_msgs/Vector3 to OpticalFlow.msg
* Added vectors to the message OpticalFlow.msg
* Added message optical flow
* Merge pull request `#1865 <https://github.com/mavlink/mavros/issues/1865>`_ from scoutdi/warnings
  Fix / suppress some build warnings
* mavros_extras: Fix some init order warnings
* Suppress warnings from included headers
* 1.16.0
* update changelog
* Merge pull request `#1817 <https://github.com/mavlink/mavros/issues/1817>`_ from lucasw/pluginlib_hpp
  use hpp instead of deprecated .h pluginlib headers
* use hpp instead of deprecated .h pluginlib headers
* Contributors: Lucas Walter, Mikhail Kolodochka, Morten Fyhn Amundsen, Vladimir Ermakov, Vladislavert

2.5.0 (2023-05-05)
------------------

2.4.0 (2022-12-30)
------------------
* extras: uncrustify
* extras: fix build, 2
* extras: fix build
* extras: fix cog
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
* Merge pull request `#1807 <https://github.com/mavlink/mavros/issues/1807>`_ from scoutdi/fix-bitwise-comparison
  mavros_extras: Fix a comparison that shouldn't be bitwise
* Merge pull request `#1808 <https://github.com/mavlink/mavros/issues/1808>`_ from scoutdi/fix-sequence-point-warning
  mavros_extras: Fix a sequence point warning
* mavros_extras: Fix a sequence point warning
* mavros_extras: Fix a comparison that shouldn't be bitwise
* Merge pull request `#1805 <https://github.com/mavlink/mavros/issues/1805>`_ from scoutdi/fix-latlon-check
  mavros_extras: Fix buggy check for lat/lon ignored
* mavros_extras: Fix buggy check for lat/lon ignored
* Contributors: Morten Fyhn Amundsen, Sverre Velten Rothmund, Vladimir Ermakov

2.3.0 (2022-09-24)
------------------
* extras: fix linter errors
* extras: fix toMsg
* extras: fix build
* extras: port guided_target
* mavros: remove custom find script, re-generate
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
* Merge pull request `#1780 <https://github.com/mavlink/mavros/issues/1780>`_ from snktshrma/master
  guided_target: accept position-target-global-int messages
* plugins: add guided_target to accept offboard position targets
  Update guided_target.cpp
  Update guided_target.cpp
  Update mavros_plugins.xml
  Update CMakeLists.txt
  Added offboard_position.cpp
  Update apm_config.yaml
  Update offboard_position.cpp
  Update offboard_position.cpp
  Rename offboard_position.cpp to guided_target.cpp
  Update CMakeLists.txt
  Update mavros_plugins.xml
  Update apm_config.yaml
  Update guided_target.cpp
* Contributors: Sanket Sharma, Vladimir Ermakov

2.2.0 (2022-06-27)
------------------
* extras: fix build
* extras: fix build
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
* cmake: style fix
* cmake: downgrade to C++17 as 20 breaks something in rclcpp
* cmake: hide -std=c++2a
* Merge pull request `#1744 <https://github.com/mavlink/mavros/issues/1744>`_ from amilcarlucas/pr_gimbal_diagnostics_fixes
  mount_control.cpp: detect MOUNT_ORIENTATION stale messages
* extras: fix cog re to extract plugin name

1.17.0 (2023-09-09)
-------------------
* Merge pull request `#1865 <https://github.com/mavlink/mavros/issues/1865>`_ from scoutdi/warnings
  Fix / suppress some build warnings
* mavros_extras: Fix some init order warnings
* Suppress warnings from included headers
* Contributors: Morten Fyhn Amundsen, Vladimir Ermakov

1.16.0 (2023-05-05)
-------------------
* Merge pull request `#1817 <https://github.com/mavlink/mavros/issues/1817>`_ from lucasw/pluginlib_hpp
  use hpp instead of deprecated .h pluginlib headers
* use hpp instead of deprecated .h pluginlib headers
* Contributors: Lucas Walter, Vladimir Ermakov

1.15.0 (2022-12-30)
-------------------
* Merge pull request `#1811 <https://github.com/mavlink/mavros/issues/1811>`_ from scoutdi/debug-float-array
  Implement debug float array handler
* Implement debug float array handler
  Co-authored-by: Morten Fyhn Amundsen <morten.f.amundsen@scoutdi.com>
* Merge pull request `#1807 <https://github.com/mavlink/mavros/issues/1807>`_ from scoutdi/fix-bitwise-comparison
  mavros_extras: Fix a comparison that shouldn't be bitwise
* Merge pull request `#1808 <https://github.com/mavlink/mavros/issues/1808>`_ from scoutdi/fix-sequence-point-warning
  mavros_extras: Fix a sequence point warning
* mavros_extras: Fix a sequence point warning
* mavros_extras: Fix a comparison that shouldn't be bitwise
* Merge pull request `#1805 <https://github.com/mavlink/mavros/issues/1805>`_ from scoutdi/fix-latlon-check
  mavros_extras: Fix buggy check for lat/lon ignored
* mavros_extras: Fix buggy check for lat/lon ignored
* Contributors: Morten Fyhn Amundsen, Sverre Velten Rothmund, Vladimir Ermakov

1.14.0 (2022-09-24)
-------------------
* Merge pull request `#1780 <https://github.com/mavlink/mavros/issues/1780>`_ from snktshrma/master
  guided_target: accept position-target-global-int messages
* plugins: add guided_target to accept offboard position targets
  Update guided_target.cpp
  Update guided_target.cpp
  Update mavros_plugins.xml
  Update CMakeLists.txt
  Added offboard_position.cpp
  Update apm_config.yaml
  Update offboard_position.cpp
  Update offboard_position.cpp
  Rename offboard_position.cpp to guided_target.cpp
  Update CMakeLists.txt
  Update mavros_plugins.xml
  Update apm_config.yaml
  Update guided_target.cpp
* Merge pull request `#1744 <https://github.com/mavlink/mavros/issues/1744>`_ from amilcarlucas/pr_gimbal_diagnostics_fixes
  mount_control.cpp: detect MOUNT_ORIENTATION stale messages
* mount_control.cpp: detect MOUNT_ORIENTATION stale messages
  correct MountConfigure response success
  correct constructor initialization order
  some gimbals send negated/inverted angle measurements, correct that to obey the MAVLink frame convention using run-time parameters
* Merge pull request `#1735 <https://github.com/mavlink/mavros/issues/1735>`_ from clydemcqueen/fix_1734
  Fix crash in vision_pose plugin
* Remove unrelated log message
* Initialize last_transform_stamp with RCL_ROS_TIME, fixes `#1734 <https://github.com/mavlink/mavros/issues/1734>`_
* Merge pull request `#1727 <https://github.com/mavlink/mavros/issues/1727>`_ from BV-OpenSource/pr-cellular-status
  Pr cellular status
* uncrustify cellular_status.cpp
* Add CellularStatus plugin and message
* Merge pull request `#1702 <https://github.com/mavlink/mavros/issues/1702>`_ from amilcarlucas/mount_diagnostics
  Mount control plugin: add configurable diagnostics
* mount_control.cpp: use mount_nh for params to keep similarities with other plugins
  set diag settings before add()
* Mount control plugin: add configurable diagnostics
* Merge pull request `#1700 <https://github.com/mavlink/mavros/issues/1700>`_ from oroelipas/fix-obstacle-distance
  Fix obstacle distance
* Bugfix: increment_f had no value asigned when input LaserScan was bigger than obstacle.distances.size()
* Bugfix: wrong interpolation when the reduction ratio (scale_factor) is not integer.
* Contributors: Clyde McQueen, Dr.-Ing. Amilcar do Carmo Lucas, Rui Mendes, Vladimir Ermakov, oroel

2.1.1 (2022-03-02)
------------------
* plugins: Fix misprint
  Fix `#1709 <https://github.com/mavlink/mavros/issues/1709>`_
* Contributors: Vladimir Ermakov

2.1.0 (2022-02-02)
------------------
* plugins: fix topic names to use  prefix for namespaced ones
* plugins: fix topic names to use  prefix for namespaced ones
* ci: fix several lint warnings
* extras: terrain: fix copy-paste artifact
* extras: port terrain plugin
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
* Merge pull request `#1677 <https://github.com/mavlink/mavros/issues/1677>`_ from AndersonRayner/add_terrain
  Add plugin for reporting terrain height estimate from the FCU
* Removed CamelCase for class members.  Publish to "report"
* More explicitly state "TerrainReport" to allow for future extension of the plugin to support other terrain messages
* Fixed callback name to match `handle\_{MESSAGE_NAME.lower()}` convention
* Fixed topic names to match more closely what other plugins use.  Fixed a typo.
* Add plugin for reporting terrain height estimate from FCU
* 1.12.2
* update changelog
* Merge pull request `#1675 <https://github.com/mavlink/mavros/issues/1675>`_ from BOB4Drone/ros2
  fix bof
* Update mag_calibration_status.cpp
* fix code style
  fix code style
* Update mag_calibration_status.cpp
* fix misprint and use size()
  fix misprint and use size()
* fix bof
  fix `#1668 <https://github.com/mavlink/mavros/issues/1668>`_
* extras: trajectory: backport `#1667 <https://github.com/mavlink/mavros/issues/1667>`_
* extras: trajectory: make linter happy after `#1667 <https://github.com/mavlink/mavros/issues/1667>`_
* Merge pull request `#1667 <https://github.com/mavlink/mavros/issues/1667>`_ from BOB4Drone/ros2
  fix bof
* fix bof
* Merge branch 'master' into ros2
  * master:
  1.12.1
  update changelog
  mavconn: fix connection issue introduced by `#1658 <https://github.com/mavlink/mavros/issues/1658>`_
  mavros_extras: Fix some warnings
  mavros: Fix some warnings
* 1.12.1
* update changelog
* Merge pull request `#1660 <https://github.com/mavlink/mavros/issues/1660>`_ from scoutdi/fix-warnings
  Fix warnings
* mavros_extras: Fix some warnings
* extras: fix parameter name
* extras: fix topic names
* Contributors: BOB4Drone, Morten Fyhn Amundsen, Vladimir Ermakov, matt

2.0.5 (2021-11-28)
------------------
* extras: make cpplint happy
* extras: fix most of build errors of SSP
* extras: servo_state_publisher ported. almost...
* extras: start porting servo_state_publisher
* extras: make cpplint happy
* extras: fix some linter errors.
  Do you know how to make me mad? Just let ament_uncrustify and
  ament_cpplint require opposite requirements!
* fix some build warnings; drop old copter vis
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
* plugin: fix some compile warnings
* cmake: require C++20 to build all modules
* extras: port distance_sensor plugin
* extras: fix camera plugin
* extras: port camera plugin
* lib: ignore MAVPACKED-related warnings from mavlink
* extras: distance_sensor: revert back to zero quaternion
  Fix `#1653 <https://github.com/mavlink/mavros/issues/1653>`_
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
* extras: landing_target: fix misprint
* plugin: initialize quaternions with identity
  Eigen::Quaternion[d|f] () does not initialize with zeroes or identity.
  So we must initialize with identity vector objects that can be left
  unassigned.
  Related to `#1652 <https://github.com/mavlink/mavros/issues/1652>`_
* Merge pull request `#1651 <https://github.com/mavlink/mavros/issues/1651>`_ from Jaeyoung-Lim/pr-image-capture-plugin
  Add camera plugin for interfacing with mavlink camera protocol
* Merge pull request `#1652 <https://github.com/mavlink/mavros/issues/1652>`_ from scoutdi/avoid-uninit-orientation
  distance_sensor: Initialize sensor orientation quaternion to zero
* Use meters for relative altitude
* distance_sensor: Initialize sensor orientation quaternion to zero
  Without this, you'll get random garbage data for the quaternion field
  of the DISTANCE_SENSOR MAVLink messages sent to the autopilot.
  The quaternion field should be set to zero when unused, according to the
  MAVLink message's field description.
* Address review comments
* Add camera plugin for interfacing with mavlink camera protocol
  Add camera image captured message for handling camera trigger information
* extras: port fake_gps
* extras: port tunnel
* extras: update metadata
* extras: port hil
* extras: fix odom
* extras: port odom
* extras: port px4flow
* extras: fix some linter warnings
* extras: fix some linter warnings
* extras: fix some linter warnings
* extras: fix some linter warnings
* extras: port wheel_odometry (partially)
* extras: port vision_speed
* extras: port vibration
* extras: port vfr_hud
* extras: port trajectory
* extras: port rangefinder
* extras: port onboard computer status, play_tune
* extras: fix linter warnings
* extras: port obstacle_distance
* extras: update metadata xml
* extras: port mount_control
* extras: fix build for Foxy
* extras: port mocap
* extras: port mag cal status
* extras: port log_transfer
* extras: fix rtcm seq
* extras: port gps_rtk
* extras: port gps_input
* extras: fixing some linter warnings
* extras: fixing some linter warnings
* Contributors: Jaeyoung-Lim, Morten Fyhn Amundsen, Vladimir Ermakov

2.0.4 (2021-11-04)
------------------
* Merge branch 'master' into ros2
  * master:
  1.10.0
  prepare release
* 1.10.0
* prepare release
* extras: remove safety_area as outdated
* extras: port esc_telemetry
* extras: port esc_status plugin
* extras: porting gps_status
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
* Tunnel: Check for invalid payload length
* mavros_extras: Create tunnel plugin
* Merge pull request `#1605 <https://github.com/mavlink/mavros/issues/1605>`_ from Peter010103/ros2
  mavros_extras: Ported vision_pose_estimate plugin for ROS2
* 1.9.0
* update changelog
* Merge pull request `#1621 <https://github.com/mavlink/mavros/issues/1621>`_ from amilcarlucas/pr/mount-control-spelling
  Spelling corrections
* Spelling corrections
* Merge pull request `#1615 <https://github.com/mavlink/mavros/issues/1615>`_ from amilcarlucas/pr/erase-logs
  This adds functionality to erase all logs on the SD card via mavlink
* Merge pull request `#1618 <https://github.com/mavlink/mavros/issues/1618>`_ from amilcarlucas/pr/GPS_INPUT-plugin
  Added GPS_INPUT plugin
* This adds functionality to erase all logs on the SD card via mavlink
* Added GPS_INPUT plugin
* Merge pull request `#1606 <https://github.com/mavlink/mavros/issues/1606>`_ from BV-OpenSource/master
  Add Mount angles message for communications with ardupilotmega.
* Merge branch 'master' into master
* Update esc_status plugin with datatype change on MAVLink.
  ESC_INFO MAVLink message was updated to have negative temperates and also at a different resolution. This commit updates those changes on this side.
* Convert status data from cdeg to rad.
* Publish quaternion information with Mount Status mavlink message.
* Add missing subscription.
* extras: port cam_imu_sync
* extras: re-generate cog
* extras: port debug_value
* Remove Mount_Status plugin. Add Status data to Mount_Control plugin. Remove Mount_Status message.
* extras: fix build, add UAS::send_massage(msg, compid)
* extras: port companion_process_status
* msgs: re-generate file lists
* style: apply ament_uncrustify --reformat
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
* extras: esc_telemetry: fix build
* extras: fix esc_telemetry centi-volt/amp conversion
* extras: uncrustify all plugins
* extras: reformat plugins xml
* extras: fix apm esc_telemetry
* actually allocate memory for the telemetry information
* fixed some compile errors
* added esc_telemetry plugin
* Add Mount angles message for communications with ardupilotmega.
* Added subscriber callback function for ROS2
* Added initialise function in vision_pose_estimate
* Boilerplate vision_pose_estimate plugin
* extras: landing_target: disable tf listener, it segfaults
* extras: regenerate plugins xml, ament_uncrustify
* mavros_extras: improve landing_target logging
* mavros_extras: ported landing_target plugin to ros2
* Reset calibration flag when re-calibrating. Prevent wrong data output.
* Delete debug files.
* Apply uncrustify changes.
* Set progress array to global to prevent erasing data.
* Move Compass calibration report to extras. Rewrite code based on instructions.
* extras: port 3dr radio
* extras: add urdf package
* extras: adsb: begin porting to ros2
* Contributors: Abhijith Thottumadayil Jagadeesh, André Filipe, David Jablonski, Dr.-Ing. Amilcar do Carmo Lucas, Karthik Desai, Morten Fyhn Amundsen, Peter010103, Ricardo Marques, Russell, Vladimir Ermakov

1.13.0 (2022-01-13)
-------------------
* Merge pull request `#1677 <https://github.com/mavlink/mavros/issues/1677>`_ from AndersonRayner/add_terrain
  Add plugin for reporting terrain height estimate from the FCU
* Removed CamelCase for class members.  Publish to "report"
* More explicitly state "TerrainReport" to allow for future extension of the plugin to support other terrain messages
* Fixed callback name to match `handle\_{MESSAGE_NAME.lower()}` convention
* Fixed topic names to match more closely what other plugins use.  Fixed a typo.
* Add plugin for reporting terrain height estimate from FCU
* Contributors: Vladimir Ermakov, matt

1.12.2 (2021-12-12)
-------------------
* extras: trajectory: backport `#1667 <https://github.com/mavlink/mavros/issues/1667>`_
* Contributors: Vladimir Ermakov

1.12.1 (2021-11-29)
-------------------
* Merge pull request `#1660 <https://github.com/mavlink/mavros/issues/1660>`_ from scoutdi/fix-warnings
  Fix warnings
* mavros_extras: Fix some warnings
* Contributors: Morten Fyhn Amundsen, Vladimir Ermakov

1.12.0 (2021-11-27)
-------------------
* extras: distance_sensor: revert back to zero quaternion
  Fix `#1653 <https://github.com/mavlink/mavros/issues/1653>`_
* Contributors: Vladimir Ermakov

1.11.1 (2021-11-24)
-------------------

1.11.0 (2021-11-24)
-------------------
* extras: landing_target: fix misprint
* plugin: initialize quaternions with identity
  Eigen::Quaternion[d|f] () does not initialize with zeroes or identity.
  So we must initialize with identity vector objects that can be left
  unassigned.
  Related to `#1652 <https://github.com/mavlink/mavros/issues/1652>`_
* Merge pull request `#1651 <https://github.com/mavlink/mavros/issues/1651>`_ from Jaeyoung-Lim/pr-image-capture-plugin
  Add camera plugin for interfacing with mavlink camera protocol
* Merge pull request `#1652 <https://github.com/mavlink/mavros/issues/1652>`_ from scoutdi/avoid-uninit-orientation
  distance_sensor: Initialize sensor orientation quaternion to zero
* Use meters for relative altitude
* distance_sensor: Initialize sensor orientation quaternion to zero
  Without this, you'll get random garbage data for the quaternion field
  of the DISTANCE_SENSOR MAVLink messages sent to the autopilot.
  The quaternion field should be set to zero when unused, according to the
  MAVLink message's field description.
* Address review comments
* Add camera plugin for interfacing with mavlink camera protocol
  Add camera image captured message for handling camera trigger information
* Contributors: Jaeyoung-Lim, Morten Fyhn Amundsen, Vladimir Ermakov

1.10.0 (2021-11-04)
-------------------
* Merge pull request `#1625 <https://github.com/mavlink/mavros/issues/1625>`_ from scoutdi/tunnel-plugin
  Plugin for TUNNEL messages
* Tunnel: Check for invalid payload length
* mavros_extras: Create tunnel plugin
* Contributors: Morten Fyhn Amundsen, Vladimir Ermakov

1.9.0 (2021-09-09)
------------------
* Merge pull request `#1621 <https://github.com/mavlink/mavros/issues/1621>`_ from amilcarlucas/pr/mount-control-spelling
  Spelling corrections
* Spelling corrections
* Merge pull request `#1615 <https://github.com/mavlink/mavros/issues/1615>`_ from amilcarlucas/pr/erase-logs
  This adds functionality to erase all logs on the SD card via mavlink
* Merge pull request `#1618 <https://github.com/mavlink/mavros/issues/1618>`_ from amilcarlucas/pr/GPS_INPUT-plugin
  Added GPS_INPUT plugin
* This adds functionality to erase all logs on the SD card via mavlink
* Added GPS_INPUT plugin
* Merge pull request `#1606 <https://github.com/mavlink/mavros/issues/1606>`_ from BV-OpenSource/master
  Add Mount angles message for communications with ardupilotmega.
* Merge branch 'master' into master
* Update esc_status plugin with datatype change on MAVLink.
  ESC_INFO MAVLink message was updated to have negative temperates and also at a different resolution. This commit updates those changes on this side.
* Convert status data from cdeg to rad.
* Publish quaternion information with Mount Status mavlink message.
* Add missing subscription.
* Remove Mount_Status plugin. Add Status data to Mount_Control plugin. Remove Mount_Status message.
* extras: esc_telemetry: fix build
* extras: fix esc_telemetry centi-volt/amp conversion
* extras: uncrustify all plugins
* extras: reformat plugins xml
* extras: fix apm esc_telemetry
* actually allocate memory for the telemetry information
* fixed some compile errors
* added esc_telemetry plugin
* Add Mount angles message for communications with ardupilotmega.
* Reset calibration flag when re-calibrating. Prevent wrong data output.
* Delete debug files.
* Apply uncrustify changes.
* Set progress array to global to prevent erasing data.
* Move Compass calibration report to extras. Rewrite code based on instructions.
* Contributors: Abhijith Thottumadayil Jagadeesh, André Filipe, Dr.-Ing. Amilcar do Carmo Lucas, Karthik Desai, Ricardo Marques, Russell, Vladimir Ermakov

1.8.0 (2021-05-05)
------------------
* extras: `#1370 <https://github.com/mavlink/mavros/issues/1370>`_: set obstacle aangle offset
* extras: distance_sensor: rename param for custom orientation, apply uncrustify
* distance_sensor: Add horizontal_fov_ratio, vertical_fov_ratio, sensor_orientation parameters
* distance_sensor: Fill horizontal_fov, vertical_fov, quaternion
* Contributors: Alexey Rogachevskiy, Vladimir Ermakov

1.7.1 (2021-04-05)
------------------

1.7.0 (2021-04-05)
------------------

1.6.0 (2021-02-15)
------------------
* Fixed a bug in mavros_extras/src/plugins/odom.cpp by switching lines 175 and 180.
  Rationale: The pose covariance matrix published to the /mavros/odometry/in topic is exclusively zeros. This is because the transformation matrix r_pose is initialised as zeros (line 140), then applied to the covariance matrix cov_pose (line 176) and then populated (line 180). Clearly the latter two steps should be the other way around, and the comments in the code appear to suggest that this was the intention, but that lines 175 and 180 were accidentally written the wrong way around. Having switched them, the pose covariance is now published to /mavros/odometry/in as expected.
  JohnG897
* Contributors: John Gifford

1.5.2 (2021-02-02)
------------------
* bugfix - add estimator type in odom message
  Add missing estimator_type field in  Odometry message.
  Issue `#1524 <https://github.com/mavlink/mavros/issues/1524>`_
* Contributors: Ashwin Varghese Kuruttukulam

1.5.1 (2021-01-04)
------------------
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
* mavros_extras: Fix member initialization order
* mavros_extras: Add override specifiers
* mavros_extras: distance_sensor: Don't publish data when orientation configuration does not match incomming data.
* fake_gps: Fix assignment typo
  This colon should probably be an equals sign.
  With the colon, this assignment becomes a label instead,
  and `_gps_rate` after the colon becomes an unused
  expression result.
* Contributors: Kristian Klausen, Morten Fyhn Amundsen

1.4.0 (2020-09-11)
------------------
* mavros: use mavlink::minimal:: after incompatible changes in mavlink package
  Incompatible change: https://github.com/mavlink/mavlink/pull/1463
  Fix: `#1483 <https://github.com/mavlink/mavros/issues/1483>`_, https://github.com/mavlink/mavlink/issues/1474
* play_tune: Assign tune format directly
* play_tune: Uncrustify
* play_tune: Use msg_set_target and set_string_z
* play_tune: Write new plugin
* Contributors: Morten Fyhn Amundsen, Vladimir Ermakov

1.3.0 (2020-08-08)
------------------
* Take into account message count for message size
* Add esc_status plugin.
* fake_gps.cpp: Implement GPS time data
* fake_gps.cpp: implement speed accuracy
* fake_gps.cpp: Added horiz_accuracy and vert_accuracy parameters
* fake_gps.cpp: Add mocap_withcovariance configuration parameter
* fake_gps.cpp: add initial support for GPS_INPUT MAVLink message
* fake_gps.cpp: uncrustify
* Add gps_status plugin to publish GPS_RAW and GPS_RTK messages from FCU.
  The timestamps for the gps_status topics take into account the mavlink time and uses the convienence function
* uncrustify gps_rtk plugin
* adding support for publishing rtkbaseline msgs over ROS
* Contributors: CSCE439, Dr.-Ing. Amilcar do Carmo Lucas, Ricardo Marques

1.2.0 (2020-05-22)
------------------
* Revert "extras: odom: Hardcode BODY_FRD enum_value to fix compilation"
  This reverts commit 1a898dea2202ee1af56d698bd40d40994346c5cb.
* extras: odom: Hardcode BODY_FRD enum_value to fix compilation
  That PR broke build: https://github.com/mavlink/mavlink/pull/1112
  TODO: find out what frame should be used
* Contributors: Vladimir Ermakov

1.1.0 (2020-04-04)
------------------
* Setting the same transparency for all elements
* Visualization of the direction of the device
* add support for bezier
* Contributors: Alamoris, Martina Rivizzigno

1.0.0 (2020-01-01)
------------------
* Change odometry subscription queue to 1 to avoid buffering.
* Contributors: James Goppert

0.33.4 (2019-12-12)
-------------------
* obstacle_distance: Fill both increment and increment_f fields
* obstacle_distance: Fix wrong angle increment
  The computation `req->angle_increment * RAD_TO_DEG` correctly computes
  angle increment in degrees as a float, but the `increment` field of the
  OBSTACLE_DISTANCE MAVLink message is a uint8, so the float value gets
  truncated. So if your real increment is 10 degrees, you may a floating
  point value of something like 9.999999, which results in the integer value
  9 getting written to the increment field.
  An improvement would be to round properly, with something like
  `static_cast<uint8_t>(increment_deg_float)`,
  but a better solution is to allow non-integer degree values for the
  increment, which is supported by the `increment_f` field. According
  to the MAVLink reference, increment_f is used instead of increment
  whenever increment_f is nonzero.
* Contributors: Morten Fyhn Amundsen

0.33.3 (2019-11-13)
-------------------
* package: fix 6fa58e59 - main package depends on trajectory_msgs, not extras
* Contributors: Vladimir Ermakov

0.33.2 (2019-11-13)
-------------------
* Add trajectory_msg as dependency
* Contributors: Jaeyoung-Lim

0.33.1 (2019-11-11)
-------------------
* Merge pull request `#1297 <https://github.com/mavlink/mavros/issues/1297>`_ from dayjaby/feature/mount_orientation
  adding mount orientation to mount_control plugin
* landing_target: Fix cartesian to displacement bug
  I think these four conditionals are buggy:
  The first is    (x and y) > 0
  and should be   (x > 0) and (y > 0)
  (This one actually works the way it's written.)
  The second is   (x < 0 and y) > 0
  and should be   (x < 0) and (y > 0)
  The third is    (x and y) < 0
  and should be   (x < 0) and (y < 0)
  The fourth is   (x < 0 and y) < 0
  and should be   (x > 0) and (y < 0)
* obstacle distance plugin: Add ROS param for mavlink frame
  Makes it possible to specify the 'frame' field in the MAVLink
  OBSTACLE_DISTANCE message sent by this plugin. Previously the
  frame was not defined, which means it defaulted to MAV_FRAME_GLOBAL.
  (See https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE)
  The default frame is therefore still MAV_FRAME_GLOBAL.
* resolved merge conflict
* adding mount orientation to mount_control plugin
* Contributors: David Jablonski, Morten Fyhn Amundsen, Vladimir Ermakov

0.33.0 (2019-10-10)
-------------------
* CleanUp
* Odom Plugin, enforcing ROS convetion, less options but clearer to use
* Fix service namespace with new nodehandle
* Add mount configure service
* use header.stamp to fill mavlink msg field time_usec
* use cog for copy
* adapt message and plugin after mavlink message merge
* rename message and adjust fields
* add component id to mavros message to distinguish ROS msgs from different systems
* component_status message and plugin draft
* Contributors: Jaeyoung-Lim, baumanta, kamilritz

0.32.2 (2019-09-09)
-------------------
* clean up
* fix obstacle distance plugin
* Contributors: baumanta

0.32.1 (2019-08-08)
-------------------

0.32.0 (2019-07-06)
-------------------
* use direclty radians in yaml files
* add mav_cmd associated with each point in trajectory plugin
* Fix typo
* Address comments
* Send messages from callback
* Use MountControl Msg
* Add mount control class template
* Contributors: Jaeyoung-Lim, Martina Rivizzigno

0.31.0 (2019-06-07)
-------------------
* landing_target: fix landing target pose input topic naming
* fix naming for file
* mavros_plugins: fix landing_target plugin init
* landing_target: change topic subscription naming
* extras: mavros_plugins.xml: fix malform on XML file
* landing_target: use m_uas
* visualization: set the frame and child frame id back to map and base_link
* general fixup to update the landing_target codebase
* extras: landing target: improve usability and flexibility
* ident correction
* landing_target: use Eigen::Quaterniond::Identity()
* visualization: small correction on cb
* landing_target: ident correct
* landing_target: ident correction
* renamed copter_visualization to just visualization
* landing_target: target orientation: assess it is not possible
* copter_visualization: add target_size and landing_target subscriber in copter_visualization node, so to publish a marker of the target
* uas_stringify: changed UAS::idx_frame() to UAS::frame_from_str()
* landing_target: removed child_frame_id
* landing_target: minor code tweak/restructure
* landing_target: small correction on math
* landing_target: uncrustify code
* landing_target: updated to TF2 and Eigen math
* landing_target: adapted to latest master code
* landing_target: corrected pkt parser order
* landing_target: added stringify usage on code
* landing_target: added timestamp and target size fields [!Won't compile unless a new mavlink release!]
* landing_target: removed PoseWithCovarianceStamped include
* landing_target: remove the need of local_position subscription
* landing_target: fixed local_position subscriber topic name
* landing_target: updated notation and applied correct math to conversions
* landing_target: first commit
* Contributors: TSC21

0.30.0 (2019-05-20)
-------------------
* Fixed an issue when the laser scan topic contains NaN values they where being sent as 0 distances. (`#1218 <https://github.com/mavlink/mavros/issues/1218>`_)
* extras `#1223 <https://github.com/mavlink/mavros/issues/1223>`_: Add eigen aligned allocators to plugin classes.
* gps_rtk: fix multi segment messages
* Update the readme
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Jaime Machuca, Vladimir Ermakov

0.29.2 (2019-03-06)
-------------------
* extras: odom: update velocity covariance fields from 'twist' to 'velocity_covariance'
* Contributors: TSC21

0.29.1 (2019-03-03)
-------------------
* All: catkin lint files
* cmake: fix `#1174 <https://github.com/mavlink/mavros/issues/1174>`_: add msg deps for package format 2
* mavros_extras: Convert source files to Unix line endings
* Contributors: Pierre Kancir, Vladimir Ermakov, sfalexrog

0.29.0 (2019-02-02)
-------------------
* obstacle_distance: align comments
* obstacle_distance: fixup items after peer review
  changes include using size_t instead of int for loop variables
  scale_factor calculation ensures argument are floating point
  remove unnecessary n variable
* obstacle_distance: combine sensor distances to fit within outgoing message
* gps_rtk: documentation fixes
* Fix broken documentation URLs
* added tf2_eigen to dependencies, so that building with catkin tools does not fail anymore
* Merge branch 'master' into param-timeout
* mavros_extras: Wheel odometry plugin updated according to the final mavlink WHEEL_DISTANCE message.
* mavros_extras: mavros_plugins.xml fix after bad merge.
* mavros_extras: Wheel odometry plugin, twist covariance matrix non-used diagonal elements zeroed.
* mavros_extras: Wheel odometry plugin, odometry error propagation added respecting kinematics.
* mavros_extras: Wheel odometry plugin travelled distance fixed.
* mavros_extras: Wheel odometry plugin y-speed covariance fixed.
* mavros_extras: Wheel odometry plugin updated to compute accurate speeds from distances using internal timesteps.
* mavros_extras: Wheel odometry plugin fixes after CR.
* mavros_msgs: Float32ArrayStamped replaced by WheelOdomStamped.
* mavros_extras: Wheel odometry plugin added.
* Contributors: Dr.-Ing. Amilcar do Carmo Lucas, Jan Heitmann, Pavlo Kolomiiets, Randy Mackay, Vladimir Ermakov

0.28.0 (2019-01-03)
-------------------
* odom: add ODOMETRY handler and publisher
* remove newlines after doxygen
* style clean up
* Use component_id to determine message sender
* send out companion status as heartbeat
* change message name from COMPANION_STATUS to COMPANION_PROCESS_STATUS
* change message to include pid
* Change from specific avoidance status message to a more generic companion status message
* add plugin to receive avoidance status message
* Contributors: TSC21, baumanta

0.27.0 (2018-11-12)
-------------------
* extras `#1110 <https://github.com/mavlink/mavros/issues/1110>`_ `#1111 <https://github.com/mavlink/mavros/issues/1111>`_: add eigen aligment to plugins with eigen-typed members
* Fix odom message to use covariance from msg
* Contributors: Dion Gonano, Vladimir Ermakov

0.26.3 (2018-08-21)
-------------------
* fixup! b43279058a3029c67ea75b1ecb86442c9dc991d4
* mavros_extras/log_transfer: Log transfer plugin
* Contributors: mlvov

0.26.2 (2018-08-08)
-------------------
* Fix namespace (std->extras)
* Changing the callback name to rtcm_cb
  Adding doxygen documentation
* Sort the plugins by alphabetical order
* Put back the casting
* Using size_t instead of int
  Using the same rtcm_data message
  Remove int casting
* Moving gps_rtk to mavros_extras
* Contributors: Alexis Paques

0.26.1 (2018-07-19)
-------------------
* trajectory: update plugin to match mavlink change from trajectory msg to
  trajectory_representation_waypoints
* Contributors: Martina

0.26.0 (2018-06-06)
-------------------
* odom: fix mapping for body frame
* Contributors: TSC21

0.25.1 (2018-05-14)
-------------------

0.25.0 (2018-05-11)
-------------------
* extras: Refactor Trajectory handle cb
* extras: Refactor Trajectory subscription callbacks
* trajectory: use lambda functions
* trajectory: add time_horizon for trajectory type Bezier
* trajectory: add time_horizon field
* trajectory: fix wrap_pi to have constant time execution
* trajectory: fix email
* trajectory: when receiving mavlink trajectory msg distinguish between types
  to fill correctly the mavros message
* trajectory: add path callback to support nav_msgs Path
* trajectory: update trajectory_call back so that it distinguish between
  trajectory types in copy the values
* rename ObstacleAvoidance plugin to Trajectory
* obstacle_avoidance: use cog to fill mavlink and ros messages
* obstacle_avoidance: uncrustify
* mavros_plugins: add obstacle avoidance plugin
* add obstacle_avoidance plugin
* CMakeLists: add obstacle_avoidance plugin
* extras: odom: explicitly set the lambda expression arg types
* extras: odom: use lambda expression to set the transform for twist
* extras: odom: change the way the rotation matrices are init
* extras: odom: set the frame_id to local frame only
* extras: odom: respect the Odometry msg frame spec
* extras: redo odom param processing
* extras: odom: remove unnecessary eigen_conversions/eigen_msg.h include
* extras: odom: fix underlying_type assignment
* extras: odom: update msg spec link
* extras: odom: move frame parsing to init()
* extras: odom: change tf exception handler
* extras: odom: improve way frame naming is handled
* extras: update odom plugin to send ODOMETRY msgs
* extras: smal style fix in vision pose est
* extras: add covariance parsing to vision_speed_estimate (`#996 <https://github.com/mavlink/mavros/issues/996>`_)
* Contributors: Martina, Nuno Marques, TSC21, Vladimir Ermakov

0.24.0 (2018-04-05)
-------------------
* extras: update vision_pose_estimate plugin so it can send the covariance matrix also
* px4flow: sending OPTICAL_FLOW_RAD messages
* Contributors: Oleg Kalachev, TSC21

0.23.3 (2018-03-09)
-------------------

0.23.2 (2018-03-07)
-------------------

0.23.1 (2018-02-27)
-------------------
* odom plugin: initialize matrix with zeros
* extras fix `#950 <https://github.com/mavlink/mavros/issues/950>`_: fix unit conversions
* Contributors: ChristophTobler, Vladimir Ermakov

0.23.0 (2018-02-03)
-------------------
* add MAV_DISTANCE_SENSOR enum to_string
* extras: plugins: obstacle_distance: update to new msg definition and crystalize
* extras: obstacle_distance: increase number of array elements
* extras: plugins: add obstacle_distance plugin
* Fix vision odom.
* Contributors: James Goppert, TSC21

0.22.0 (2017-12-11)
-------------------
* scripts: Use non global mavros-ns allow to work __ns parameter
* move member variable earth initialization
* Contributors: Shingo Matsuura, Vladimir Ermakov

0.21.5 (2017-11-16)
-------------------
* extras fix `#858 <https://github.com/mavlink/mavros/issues/858>`_: fix vector copy-paste error
* Contributors: Vladimir Ermakov

0.21.4 (2017-11-01)
-------------------
* ENU<->ECEF transforms fix. (`#847 <https://github.com/mavlink/mavros/issues/847>`_)
  * ENU<->ECEF transforms fix.
  * Changes after review. Unit tests added.
* Contributors: pavloblindnology

0.21.3 (2017-10-28)
-------------------
* mavteleop: Move from iteritems to items for python3 support
  Items work with python3 and python2.7
  Signed-off-by: Patrick Jose Pereira <patrickelectric@gmail.com>
* extras: Configurable base frame id on distance_sensor
  Fix `#835 <https://github.com/mavlink/mavros/issues/835>`_
* debug_msgs: fix typo
* debug_msgs: fix typo
* extras: Use cog to reduce common msg filler code
* add debug plugin
* Contributors: Nuno Marques, Patrick Jose Pereira, TSC21, Vladimir Ermakov

0.21.2 (2017-09-25)
-------------------
* odom: fix typo
* odom: general fixes and code tighting
* Use tf2 for odom plugin and set reasoable defaults for local pos cov.
* Contributors: James Goppert, TSC21

0.21.1 (2017-09-22)
-------------------

0.21.0 (2017-09-14)
-------------------
* IMU and attitude: general clean-up
* Using tabs as the file does
* Updating comments for PX4Flow
* Removing copter_visualization from the yaml files.
  Adding odometry to apm_config
  Changing frame_id to base_link for vibration
* Update the apm_config and px4flow_config files
* Update configuration from mavros_extras
* Contributors: Alexis Paques, TSC21

0.20.1 (2017-08-28)
-------------------

0.20.0 (2017-08-23)
-------------------
* Extras: Distance sensors add RADAR and UNKNOWN type
* Extras: distance sensor don't spam when message are bounce back from FCU
* Extras: add ardupilot rangefinder plugin
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
* extras: fake_gps: use another method to throttle incoming msgs
* extras: fake_gps: compute vector2d.norm()
* frame tf: move ENU<->ECEF transforms to ftf_frame_conversions.cpp
* extras: fake_gps: use rate instead of period
* extras: fake_gps: style fix
* extras: mocap_fake_gps->fake_gps: generalize plugin and use GeographicLib possibilites
* extras: odom: Minor fixes
* extras: Add odom plugin
* Contributors: James Goppert, Nuno Marques, TSC21, Vladimir Ermakov, khancyr

0.19.0 (2017-05-05)
-------------------
* extras: fix package link
* extras: Fix adsb plugin
* extras: Add ADSB plugin
* Add frame transform for vibration levels (`#690 <https://github.com/mavlink/mavros/issues/690>`_)
  * add frame transform for accel vibration levels
  * use vectorEigenToMsg
  * unscrustify
* Contributors: Nuno Marques, Vladimir Ermakov

0.18.7 (2017-02-24)
-------------------
* vision plugin : Add missing transform
* Contributors: Kabir Mohammed

0.18.6 (2017-02-07)
-------------------

0.18.5 (2016-12-12)
-------------------

0.18.4 (2016-11-11)
-------------------
* Code clean-up
* code style fix
* markup changes
* Fake gps plugin
* Update README for all packages
* Contributors: Vilhjalmur, Vladimir Ermakov, vilhjalmur89

0.18.3 (2016-07-07)
-------------------

0.18.2 (2016-06-30)
-------------------

0.18.1 (2016-06-24)
-------------------

0.18.0 (2016-06-23)
-------------------
* extras `#560 <https://github.com/mavlink/mavros/issues/560>`_: remove cv_bridge and image_transport deps
* extras: Update UAS
* extras:vision_speed_estimate: Update API
* extras:vision_pose_estimate: Update API
* extras:px4flow: Update API
* extras:mocap_pose_estimate: Update API
* extras:distance_sensor: Update API
* extras:cam_imu_sync: Update API
* extras: Automatic update by sed
* extras: prepare to update
* extras `#560 <https://github.com/mavlink/mavros/issues/560>`_: Remove image streaming over mavlink support.
  Use external RTP streamer, e.g. https://github.com/ProjectArtemis/gst_video_server
* Contributors: Vladimir Ermakov

0.17.3 (2016-05-20)
-------------------

0.17.2 (2016-04-29)
-------------------

0.17.1 (2016-03-28)
-------------------
* ran uncrustify
* fixed typos
* use CUBE_LIST for faster rendering
* limit track size
* use local variable
* fixed indentation
* added rc modes
* moved rc to rc_override_control()
* replaced tabulations with spaces (4)
* introducing RC modes
* fixed
* quality added
* added visualization for local setpoints
* Contributors: Joey Gong, francois

0.17.0 (2016-02-09)
-------------------
* rebased with master
* ran uncrustify
* removed duplicate include
* use MarkerArray for vehicle model
* Updated frame transformations and added odom publisher to local position plugin
* Contributors: Eddy, francois

0.16.6 (2016-02-04)
-------------------
* extras: uncrustify
* added tf
* comments
* configurable vehicle model
* Contributors: Vladimir Ermakov, francois

0.16.5 (2016-01-11)
-------------------

0.16.4 (2015-12-14)
-------------------

0.16.3 (2015-11-19)
-------------------

0.16.2 (2015-11-17)
-------------------

0.16.1 (2015-11-13)
-------------------

0.16.0 (2015-11-09)
-------------------
* gcs_bridge `#394 <https://github.com/mavlink/mavros/issues/394>`_: enable both UDPROS and TCPROS transports
* extras fix `#392 <https://github.com/mavlink/mavros/issues/392>`_: add additional subscription for PoseWithCovarianceStamped
* Contributors: Vladimir Ermakov

0.15.0 (2015-09-17)
-------------------
* extras `#387 <https://github.com/mavlink/mavros/issues/387>`_: fix header stamp in joint_states
* extras fix `#387 <https://github.com/mavlink/mavros/issues/387>`_: SSP node done.
* extras `#387 <https://github.com/mavlink/mavros/issues/387>`_: subscriber works, node almost done
* extras `#387 <https://github.com/mavlink/mavros/issues/387>`_: load URDF
* extras `#387 <https://github.com/mavlink/mavros/issues/387>`_: initial import of servo_status_publisher
* Contributors: Vladimir Ermakov

0.14.2 (2015-08-20)
-------------------
* extras: fix catkin lint warnings
* Contributors: Vladimir Ermakov

0.14.1 (2015-08-19)
-------------------

0.14.0 (2015-08-17)
-------------------
* extras: gcs node: replace deprecated copy function
* extras: scripts: use API from mavros module
* package: remove not exist dependency
* extras: vibration: Fix message include
* extras: px4flow: Fix message include
* extras: cam_imu_sync: Fix message include
* extras: update package description
* msgs: deprecate mavros::Mavlink and copy utils.
* msgs `#354 <https://github.com/mavlink/mavros/issues/354>`_: move all messages to mavros_msgs package.
* opencv 3.0/2.4 header compatibility
* fix orientation empty error
* Contributors: Vladimir Ermakov, andre-nguyen, v01d

0.13.1 (2015-08-05)
-------------------

0.13.0 (2015-08-01)
-------------------
* extras: mocap fix `#352 <https://github.com/mavlink/mavros/issues/352>`_: use new helper for quaternion.
* Merge pull request `#312 <https://github.com/mavlink/mavros/issues/312>`_ from mhkabir/cam_imu_sync
  Camera IMU synchronisation support added
* distance_sensor `#342 <https://github.com/mavlink/mavros/issues/342>`_: correct orientation parameter handling.
* distance_sensor: restructure orientation matching and verification
* lib `#319 <https://github.com/mavlink/mavros/issues/319>`_: Return quaternion from UAS::sensor_matching()
* launch fix `#340 <https://github.com/mavlink/mavros/issues/340>`_: update default component id of PX4.
* extras: distance_sensor `#71 <https://github.com/mavlink/mavros/issues/71>`_: Purt to TF2.
* plugin: Use UAS::syncronized_header() for reduce LOC.
* extras: vision_pose `#71 <https://github.com/mavlink/mavros/issues/71>`_: Use TF2 listener.
  Also `#319 <https://github.com/mavlink/mavros/issues/319>`_.
* launch: Update configs.
* extras: viz `#336 <https://github.com/mavlink/mavros/issues/336>`_: convert plugin to node.
* extras: vision_speed `#319 <https://github.com/mavlink/mavros/issues/319>`_: use eigen based transform
* extras: vibration: Use UAS::synchronized_header()
* extras: px4flow `#319 <https://github.com/mavlink/mavros/issues/319>`_: change transform_frame()
* extras: mocap `#319 <https://github.com/mavlink/mavros/issues/319>`_: use eigen based transform
* Camera IMU synchronisation support added
* Contributors: Mohammed Kabir, TSC21, Vladimir Ermakov

0.12.0 (2015-07-01)
-------------------
* coverity: make them happy
* frame_conversions: use inline functions to identify direction of conversion
* changed frame conversion func name; add 3x3 cov matrix frame conversion; general doxygen comment cleanup
* frame_conversions: added frame_conversion specific lib file; applied correct frame conversion between ENU<->NED
* vibration_plugin: changed vibration to Vector3
* vibration_plugin: msg reformulation
* vibration_plugin: first commit
* Changes some frames from world to body conversion for NED to ENU.
* mavros `#302 <https://github.com/vooon/mavros/issues/302>`_: fix style
* mavros fix `#301 <https://github.com/vooon/mavros/issues/301>`_: move sensor orientation util to UAS
* distance_sensor: typo; style fixe
* sensor_orientation: corrected rotation set sequence
* sensor_orientation: updated orientation enum; updated data type
* sensor_orientation: removed unecessary sum on setting rotation
* sensor_orientation: added sensor orientation matching helper func
* distance_sensor: minor correction
* distance_sensor: sensor position cond changed
* distance_sensor: tweak param check; cond routines
* distance_sensor: removed unnecessary comment line
* distance_sensor: ctor list update
* distance_sensor: define sensor position through param config
* distance_sensor: minor comment identation correction
* distance_sensor: tf::Transform creation optional
* distance_sensor: add tf_broadcaster between 'fcu' and the distance sensor
* distance_sensor: remove commented code
* distance_sensor: removed dbg msg
* distance_sensor: cov condition defined
* distance_sensor: covariance condition changed
* distance_sensor: conditional state change
* distance_sensor: covariance condition set - correction
* distance_sensor: covariance condition set
* distance_sensor: ctor list update (corrected)
* distance_sensor: ctor list update
* distance_sensor: ctor list update
* distance_sensor: small correction
* distance_sensor: uncrustify
* distance_sensor: array limiting; cast correction; other minor correc
* distance_sensor: travis build correction
* distance_sensor: uncrustify distance_sensor.cpp
* distance_sensor: small corrections on variable definitions, method calls
* distance_sensor: small enhancements
* distance_sensor `#292 <https://github.com/vooon/mavros/issues/292>`_: uncrustify
* distance_sensor `#292 <https://github.com/vooon/mavros/issues/292>`_: fix travis build.
* distance_sensor `#292 <https://github.com/vooon/mavros/issues/292>`_: implement message handling
* distance_sensor `#292 <https://github.com/vooon/mavros/issues/292>`_: parse mapping configuration.
* distance_sensor: remove DistanceSensor.msg from CMakeList
* distance_sensor: removed DistanceSensor.msg
* distance_sensor:
  -> use std Range.msg
  -> published frame_id in topics are dinamic - depend on type and id of the sensor
* distance_sensor: comment correction
* distance_sensor: minor correction
* distance_sensor: minor fixes that include use Range.msg to Laser data
* distance_sensor: add plugin file
* distance_sensor plugin: first commit!
* Contributors: TSC21, Tony Baltovski, Vladimir Ermakov

0.11.2 (2015-04-26)
-------------------
* gcs bridge fix `#277 <https://github.com/vooon/mavros/issues/277>`_: add link diagnostics
* Contributors: Vladimir Ermakov

0.11.1 (2015-04-06)
-------------------
* mavftpfuse `#129 <https://github.com/vooon/mavros/issues/129>`_: done!
  Fix `#129 <https://github.com/vooon/mavros/issues/129>`_.
* mavftpfuse `#129 <https://github.com/vooon/mavros/issues/129>`_: cache file attrs
* mavftpfuse `#129 <https://github.com/vooon/mavros/issues/129>`_: initial import
* Contributors: Vladimir Ermakov

0.11.0 (2015-03-24)
-------------------
* extras: vision_pose `#247 <https://github.com/vooon/mavros/issues/247>`_: rename topic
* extras: launch `#257 <https://github.com/vooon/mavros/issues/257>`_: use white list for px4flow.
  Also updates config `#211 <https://github.com/vooon/mavros/issues/211>`_.
* uncrustify and fix `#207 <https://github.com/vooon/mavros/issues/207>`_
* uncrustify extras
* package: update lic
* license `#242 <https://github.com/vooon/mavros/issues/242>`_: update mavros_extras headers
* plugin api `#241 <https://github.com/vooon/mavros/issues/241>`_: move diag updater to UAS.
* plugin api `#241 <https://github.com/vooon/mavros/issues/241>`_: remove global private node handle.
  Now all plugins should define their local node handle (see dummy.cpp).
  Also partially does `#233 <https://github.com/vooon/mavros/issues/233>`_ (unmerge setpoint topic namespace).
* plugin api `#241 <https://github.com/vooon/mavros/issues/241>`_: remove `get_name()`
* Add BSD license option `#220 <https://github.com/vooon/mavros/issues/220>`_
* uncrustify: mocap plugin
* Switched from mavlink VICON_POSITION_ESTIMATE to ATT_POS_MOCAP.
* Contributors: Tony Baltovski, Vladimir Ermakov

0.10.2 (2015-02-25)
-------------------
* launch: Fix vim modelines `#213 <https://github.com/vooon/mavros/issues/213>`_
* Contributors: Vladimir Ermakov

0.10.1 (2015-02-02)
-------------------
* Fix @mhkabir name in contributors.
* Updated mavros_extra README to explain the vision_estimate plugin should be used for the mocap data currently.
* Update px4flow.cpp
* plguin: px4flow: Remove all ref to old message
* Merge remote-tracking branch 'upstream/master' into optflow_rad
  Conflicts:
  mavros_extras/CMakeLists.txt
* Update
* Clean up
* New interface commit
* Add new interface. Raw message only for now. Removed the tx functionality as it doesn't make much sense.
* Contributors: Mohammed Kabir, Tony Baltovski, Vladimir Ermakov

0.10.0 (2015-01-24)
-------------------
* mocap_pose_estimate: Switched from pose to poseStamped.
* Contributors: Tony Baltovski

0.9.4 (2015-01-06)
------------------

0.9.3 (2014-12-30)
------------------
* Initiliser fix
* plugin: visualisation - Fixes CI build
* plugin: visualisation
* plugin: visualization minor patch
* plugin: visualization finshed
* Contributors: Mohammed Kabir

0.9.2 (2014-11-04)
------------------

0.9.1 (2014-11-03)
------------------

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
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Fix headers used in mavros. Add readme.
* Update repo links.
  Package moved to mavlink organization.
* Contributors: Vladimir Ermakov

0.8.0 (2014-09-22)
------------------
* Revert "Update package.xml format to REP140 (2)."
  This reverts commit 81286eb84090a95759591cfab89dd9718ff35b7e.
  ROS Hydro don't fully support REP140: rospack can't find plugin
  descriptions.
  Fix `#151 <https://github.com/vooon/mavros/issues/151>`_.
* Added arming/disarming for att mode.
* Added arming and disarming via mavteleop.
* extras: mocap: Fix param/topic namespace.
  Fix `#150 <https://github.com/vooon/mavros/issues/150>`_.
* extras: launch: Use includes.
  Fix `#144 <https://github.com/vooon/mavros/issues/144>`_.
* Update package.xml format to REP140 (2).
  Fix `#104 <https://github.com/vooon/mavros/issues/104>`_.
* extras: launch: Fix typos.
* extras: launch: Add teleop launch script.
* extras: mavteleop: Dirty implementation of position control mode.
  Issue `#133 <https://github.com/vooon/mavros/issues/133>`_.
* extras: mavteleop: Implement velocity setpoint control.
  Issue `#133 <https://github.com/vooon/mavros/issues/133>`_.
* extras: mavteleop: Implement attitude control mode.
  Issue `#133 <https://github.com/vooon/mavros/issues/133>`_.
* extras: Use cmake modules.
  Issue `#139 <https://github.com/vooon/mavros/issues/139>`_.
* Update doxygen documentation.
  Add split lines in UAS, and make UAS.connection atomic.
  Add rosdoc configuration for mavros_extras.
* scripts: mavsetp: corrected msg API; mavteleop: added prefix to rc override
* scripts: Initial import mavteleop
  Now it's just proof of concept.
  Implemented only RC override of RPYT channels.
  Issue `#133 <https://github.com/vooon/mavros/issues/133>`_.
* node: Catch URL open exception.
  Also update connection pointer type.
* Contributors: Nuno Marques, Tony Baltovski, Vladimir Ermakov

0.7.1 (2014-08-25)
------------------
* plugins: Change UAS FCU link name.
  Reduce smart pointer count, that hold fcu link object.
* Plugins: finish moving plugins
* Closes `#122 <https://github.com/vooon/mavros/issues/122>`_, closes `#123 <https://github.com/vooon/mavros/issues/123>`_; plugins: move mocap & vision plugins to extras, change vision plugins name
* launch: Add example launch for `#103 <https://github.com/vooon/mavros/issues/103>`_.
* extras: image_pub: Update plugin API.
* extras: px4flow: Update plugin API.
* plugins: disable most of plugins
* extras: init ctor
* extras: Fix package URLs
* test: temporary travis hack (manually download latest mavlink deb)
* Update readme
* Contributors: Nuno Marques, Vladimir Ermakov

0.7.0 (2014-08-12)
------------------
* move exras to subdirectory, `#101 <https://github.com/vooon/mavros/issues/101>`_
* Contributors: Vladimir Ermakov, Mohammed Kabir
