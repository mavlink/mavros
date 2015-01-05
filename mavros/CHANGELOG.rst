^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.4 (2015-01-06)
------------------
* plugin: sys_time: enable EMA
* Contributors: M.H.Kabir

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
* Contributors: Kabir Mohammed, M.H.Kabir, Mohammed Kabir, Vladimir Ermakov

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
* Contributors: Vladimir Ermakov, M.H.Kabir, Nuno Marques, Glenn Gregory

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

