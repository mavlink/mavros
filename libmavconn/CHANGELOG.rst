^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libmavconn
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2023-09-09)
------------------
* fix ament_cpplint
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
* Merge pull request `#1865 <https://github.com/mavlink/mavros/issues/1865>`_ from scoutdi/warnings
  Fix / suppress some build warnings
* Suppress warnings from included headers
* 1.16.0
* update changelog
* Contributors: Morten Fyhn Amundsen, Vladimir Ermakov

2.5.0 (2023-05-05)
------------------

2.4.0 (2022-12-30)
------------------
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
* Merge pull request `#1794 <https://github.com/mavlink/mavros/issues/1794>`_ from rossizero/master
  libmavconn: fix MAVLink v1.0 output selection
* libmavconn: fix MAVLink v1.0 output selection
  Fix `#1787 <https://github.com/mavlink/mavros/issues/1787>`_
* Contributors: Vladimir Ermakov, rosrunne

2.3.0 (2022-09-24)
------------------
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
* libmavconn: fix MAVLink v1.0 output selection
  Fix `#1787 <https://github.com/mavlink/mavros/issues/1787>`_
* Merge pull request `#1775 <https://github.com/mavlink/mavros/issues/1775>`_ from acxz/find-geographiclib
  use already installed FindGeographicLib.cmake
* use already installed FindGeographicLib.cmake
* Contributors: Vladimir Ermakov, acxz

2.2.0 (2022-06-27)
------------------
* Merge pull request `#1720 <https://github.com/mavlink/mavros/issues/1720>`_ from SylvainPastor/fix/libmavconn/udp/deadlocks
  libmavconn: fix UDP deadlocks
* libmavconn: fix UDP deadlock
  Same problems as for the TCP:
  - `#1682 <https://github.com/mavlink/mavros/issues/1682>`_: fix std::system_error when tcp interface loses connection
  - `#1679 <https://github.com/mavlink/mavros/issues/1679>`_: fix deadlock when call close()
* Contributors: Sylvain Pastor, Vladimir Ermakov

2.1.1 (2022-03-02)
------------------

2.1.0 (2022-02-02)
------------------
* lib: fix reorder
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
* Merge pull request `#1682 <https://github.com/mavlink/mavros/issues/1682>`_ from SylvainPastor/fix/libmavconn/tcp/resource_deadlock
  fix std::system_error when tcp interface loses connection
* fix code style divergences
* fix std::system_error when tcp interface loses connection
  When the tcp connection is lost (remote TCP server stopped), an 'End of file' error is caught
  by the io_thread in the do_recv() that calls the close() function.
  In the close() function, we stop the io_service and wait for the end of the io_thread which
  causes an std::system_error exception (cause: Resource deadlock avoided).
  Error:   mavconn: tcp0: receive: End of file at line 250 in libmavconn/src/tcp.cpp
  terminate called after throwing an instance of 'std::system_error'
  what():  Resource deadlock avoided
  Aborted (core dumped)
  fix:
  - close() function: stop io_service if current thread id != io_thread id
  - ~MAVConnTCPClient(): stop io_service and io_thread if thread is running
* Merge pull request `#1679 <https://github.com/mavlink/mavros/issues/1679>`_ from SylvainPastor/libmavconn/fix-tcp-deadlock-when-close
  libmavconn: fix deadlock when call close()
* fix deadlock when call close()
  When calling the close() function (by a different thread), a lock (mutex) is taken at
  the start of this function which closes the socket and waits the end of io_service thread.
  Closing the socket causes the 'Operation aborted' error in do_recv() function called by
  io_service thread which in turn calls the close() function: sthis->close().
  This causes a 'deadlock'.
  fix: Reduce the scope of the lock in the close() function so that it is released before
  waiting for the thread to end.
* 1.12.2
* update changelog
* lib: fix linter errors
* Merge branch 'master' into ros2
  * master:
  1.12.1
  update changelog
  mavconn: fix connection issue introduced by `#1658 <https://github.com/mavlink/mavros/issues/1658>`_
  mavros_extras: Fix some warnings
  mavros: Fix some warnings
* 1.12.1
* update changelog
* mavconn: fix connection issue introduced by `#1658 <https://github.com/mavlink/mavros/issues/1658>`_
* Contributors: Sylvain Pastor, Vladimir Ermakov

2.0.5 (2021-11-28)
------------------
* extras: fix some linter errors.
  Do you know how to make me mad? Just let ament_uncrustify and
  ament_cpplint require opposite requirements!
* lib: fix linter errors
* fix some build warnings; drop old copter vis
* lib: fix merge artifact
* Merge branch 'master' into ros2
  * master:
  1.12.0
  update changelog
  Fix multiple bugs
  lib: fix mission frame debug print
  extras: distance_sensor: revert back to zero quaternion
* 1.12.0
* update changelog
* Merge pull request `#1658 <https://github.com/mavlink/mavros/issues/1658>`_ from asherikov/as_bugfixes
  Fix multiple bugs
* Fix multiple bugs
  - fix bad_weak_ptr on connect and disconnect
  - introduce new API to avoid thread race when assigning callbacks
  - fix uninitialized variable in TCP client constructor which would
  randomly block TCP server
  This is an API breaking change: if client code creates connections using
  make_shared<>() instead of open_url(), it is now necessary to call new
  connect() method explicitly.
* cmake: require C++20 to build all modules
* lib: ignore MAVPACKED-related warnings from mavlink
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
* Contributors: Alexander Sherikov, Vladimir Ermakov

2.0.4 (2021-11-04)
------------------
* Merge branch 'master' into ros2
  * master:
  1.10.0
  prepare release
* 1.10.0
* prepare release
* mavconn: update to use std::error_code
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
* Merge pull request `#1626 <https://github.com/mavlink/mavros/issues/1626>`_ from valbok/crash_on_shutdown
  Show ENOTCONN error instead of crash on socket's shutdown
* Show ENOTCONN error instead of crash
  When a client suddenly drops the connection,
  socket.shutdown() will throw an exception:
  boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >
  what():  shutdown: Transport endpoint is not connected
  Showing an error in this common case looks more reasonable than crashing.
* 1.9.0
* update changelog
* Contributors: Val Doroshchuk, Vladimir Ermakov

2.0.3 (2021-06-20)
------------------

2.0.2 (2021-06-20)
------------------
* lib: yet another fix of cmake module
* lib: fix lint error
* lib: fix cmake lint error
* Contributors: Vladimir Ermakov

2.0.1 (2021-06-06)
------------------
* Merge branch 'master' into ros2
  * master:
  readme: update
  1.8.0
  update changelog
  Create semgrep-analysis.yml
  Create codeql-analysis.yml
* 1.8.0
* update changelog
* Contributors: Vladimir Ermakov

2.0.0 (2021-05-28)
------------------
* pylib: fix flake8
* libmavconn: fix uncrustify test error
* Merge branch 'master' into ros2
  * master:
  1.7.1
  update changelog
  re-generate all pymavlink enums
  1.7.0
  update changelog
* router: rename mavlink to/from to source/sink, i think that terms more descriptive
* mavros: fix cmake to build libmavros
* lib: make ament_lint_cmake happy
* msgs: add linter
* lib: fix all ament_cpplint errors
* lib: make cpplint happy
* lib: make ament_uncrustify happy, update BSD license text to one known by ament_copyright
* lib: try to fix ament_copyright lint
* lib: port cpp, update license headers for ament_copyright
* lib: port to standalone asio
* lib: remove boost usage from headers
* lib: update code style
* lib: rename cpp headers
* lib: provide copy of em_expand()
* lib: update readme
* libmavconn: start porintg, will use plain asio, without boost
* Merge pull request `#1186 <https://github.com/mavlink/mavros/issues/1186>`_ from PickNikRobotics/ros2
  mavros_msgs Ros2
* Merge branch 'ros2' into ros2
* msgs: start porting to ROS2
* disable all packages but messages
* Contributors: Mikael Arguedas, Vladimir Ermakov

1.17.0 (2023-09-09)
-------------------
* Merge pull request `#1865 <https://github.com/mavlink/mavros/issues/1865>`_ from scoutdi/warnings
  Fix / suppress some build warnings
* Suppress warnings from included headers
* Contributors: Morten Fyhn Amundsen, Vladimir Ermakov

1.16.0 (2023-05-05)
-------------------

1.15.0 (2022-12-30)
-------------------
* Merge pull request `#1794 <https://github.com/mavlink/mavros/issues/1794>`_ from rossizero/master
  libmavconn: fix MAVLink v1.0 output selection
* libmavconn: fix MAVLink v1.0 output selection
  Fix `#1787 <https://github.com/mavlink/mavros/issues/1787>`_
* Contributors: Vladimir Ermakov, rosrunne

1.14.0 (2022-09-24)
-------------------
* libmavconn: fix MAVLink v1.0 output selection
  Fix `#1787 <https://github.com/mavlink/mavros/issues/1787>`_
* Merge pull request `#1775 <https://github.com/mavlink/mavros/issues/1775>`_ from acxz/find-geographiclib
  use already installed FindGeographicLib.cmake
* use already installed FindGeographicLib.cmake
* Contributors: Vladimir Ermakov, acxz

1.13.0 (2022-01-13)
-------------------

1.12.2 (2021-12-12)
-------------------

1.12.1 (2021-11-29)
-------------------
* mavconn: fix connection issue introduced by `#1658 <https://github.com/mavlink/mavros/issues/1658>`_
* Contributors: Vladimir Ermakov

1.12.0 (2021-11-27)
-------------------
* Merge pull request `#1658 <https://github.com/mavlink/mavros/issues/1658>`_ from asherikov/as_bugfixes
  Fix multiple bugs
* Fix multiple bugs
  - fix bad_weak_ptr on connect and disconnect
  - introduce new API to avoid thread race when assigning callbacks
  - fix uninitialized variable in TCP client constructor which would
  randomly block TCP server
  This is an API breaking change: if client code creates connections using
  make_shared<>() instead of open_url(), it is now necessary to call new
  connect() method explicitly.
* Contributors: Alexander Sherikov, Vladimir Ermakov

1.11.1 (2021-11-24)
-------------------

1.11.0 (2021-11-24)
-------------------

1.10.0 (2021-11-04)
-------------------
* Merge pull request `#1626 <https://github.com/mavlink/mavros/issues/1626>`_ from valbok/crash_on_shutdown
  Show ENOTCONN error instead of crash on socket's shutdown
* Show ENOTCONN error instead of crash
  When a client suddenly drops the connection,
  socket.shutdown() will throw an exception:
  boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >
  what():  shutdown: Transport endpoint is not connected
  Showing an error in this common case looks more reasonable than crashing.
* Contributors: Val Doroshchuk, Vladimir Ermakov

1.9.0 (2021-09-09)
------------------

1.8.0 (2021-05-05)
------------------

1.7.1 (2021-04-05)
------------------

1.7.0 (2021-04-05)
------------------

1.6.0 (2021-02-15)
------------------

1.5.2 (2021-02-02)
------------------

1.5.1 (2021-01-04)
------------------
* Fix test for renaming of HEARTBEAT
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
* libmavconn: Fix build warnings
* Contributors: Morten Fyhn Amundsen

1.4.0 (2020-09-11)
------------------
* Dispatch GCS IP address
* Contributors: Morten Fyhn Amundsen

1.3.0 (2020-08-08)
------------------
* allow mavros to compile in CI environment
* Contributors: Marcelino

1.2.0 (2020-05-22)
------------------

1.1.0 (2020-04-04)
------------------

1.0.0 (2020-01-01)
------------------

0.33.4 (2019-12-12)
-------------------
* add macro for get_io_service() to work with boost>1.70
* Contributors: acxz

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
* libmavconn: simplify parse_buffer, and fix dropped_packets and parse_error counters
  Currently the dropped_packets & parse_error counters are always published as 0 in mavros_diag.cpp.
  This seems to be caused by using the wrong status struct.
  Seems like mavros was editing m_status after mavlink_frame_char_buffer. This struct
  seems to be the parsing state, and looks like it shouldn't be modified by the caller (for example
  status->parse_error is zeroed out in the end of mavlink_frame_char_buffer).
  Also, the crc & signature checks done in mavros seems redundant.
  r_mavlink_status seems to be the struct that holds the mavlink connection information, therefore I
  changed get_status to return it instead.
  This fixes `#1285 <https://github.com/mavlink/mavros/issues/1285>`_.
* Contributors: Koby Aizer

0.32.2 (2019-09-09)
-------------------

0.32.1 (2019-08-08)
-------------------

0.32.0 (2019-07-06)
-------------------

0.31.0 (2019-06-07)
-------------------
* readme: fix udp-pb formatting
* Contributors: Vladimir Ermakov

0.30.0 (2019-05-20)
-------------------

0.29.2 (2019-03-06)
-------------------

0.29.1 (2019-03-03)
-------------------
* All: catkin lint files
* Contributors: Pierre Kancir

0.29.0 (2019-02-02)
-------------------
* Merge branch 'master' into param-timeout
* libmavconn: Fix building without installation. Detect CI environment
* ci:test: temporary disable failed udp bind test
* mavconn:pkg: Move generated files to build tree
* Contributors: Vladimir Ermakov

0.28.0 (2019-01-03)
-------------------
* libmavconn: add the possibility to set the source component ID through the send_message method
* Contributors: TSC21

0.27.0 (2018-11-12)
-------------------
* bind should be called after reuse_address is set
* Contributors: Shahar Kosti

0.26.3 (2018-08-21)
-------------------
* Prevent MAVConnTCPClient::do_recv and MAVConnTCPServer::do_accept from running after destruction has begun
* libmavconn/CMakeLists.txt: link mavconn-test against pthread
* Contributors: mlvov

0.26.2 (2018-08-08)
-------------------

0.26.1 (2018-07-19)
-------------------

0.26.0 (2018-06-06)
-------------------
* libmavconn: add scheme for permanent UDP broadcasting
* test python 3 f-string formatting
* Contributors: Oleg Kalachev, Vladimir Ermakov

0.25.1 (2018-05-14)
-------------------
* lib `#1026 <https://github.com/mavlink/mavros/issues/1026>`_: fix logInform compat
* lib `#1026 <https://github.com/mavlink/mavros/issues/1026>`_: add compat header for older console-bridge
* Contributors: Vladimir Ermakov

0.25.0 (2018-05-11)
-------------------
* lib: console-bridge uses macroses...
* lib: fixing console-bridge now prefixed
* Contributors: Vladimir Ermakov

0.24.0 (2018-04-05)
-------------------
* libmavconn: make serial.cpp more portable
* libmavconn : enable low-latency mode on Linux
  Some common USB-UART convertors like the FTDI accumulates individual bytes from the serial link
  in order to send them in a single USB packet (Nagling). This commit sets the ASYNC_LOW_LATENCY flag,
  which the FTDI kernel driver interprets as a request to drop the Nagling timer to 1ms (i.e send all
  accumulated bytes after 1ms.)
  This reduces average link RTT to under 5ms at 921600 baud, and enables the use of mavros in
  systems where low latency is required to get good performance for e.g estimation and controls.
* Contributors: Mohammed Kabir, Vladimir Ermakov

0.23.3 (2018-03-09)
-------------------
* libmavconn: better preprocessor conditions for serial workaround
* libmavconn : fix hardware flow control setting for Boost < v1.66
  This commit fixes handling of hardware flow control. Due to bugs in Boost, set_option() would not work for flow control settings. This is fixed in Boost v1.66. Relevant Boost commit : https://github.com/boostorg/asio/commit/619cea4356
* lib cmake: disable debug message
* lib: simplify geolib cmake module, try to fix CI
* Contributors: Mohammed Kabir, Vladimir Ermakov

0.23.2 (2018-03-07)
-------------------
* mavconn: small style fix
* Libmavconn : Set the serial port on Raw mode to prevent EOF error
* Libmavconn: ensure the ports are cleanly closed before end connexions.
* Contributors: Pierre Kancir, Vladimir Ermakov

0.23.1 (2018-02-27)
-------------------
* compile also with boost >= 1.66.0
  In boost 1.66.0, which includes boost-asio 1.12.0, the asio
  interfaces have been changed to follow the "C++ Extensions for
  Networking" Technical Specification [1]. As a consequence,
  resolvers now produce ranges rather than iterators.
  In boost < 1.66.0, resolver.resolve returns an iterator that must
  be passed to `std::for_each`. As this iterator in boost < 1.66.0
  does not provide begin() and end() member functions, it cannot be
  simply turned into a proper range.
  For boost >= 1.66.0, resolver.resolve returns a range, which
  can be just iterated through with `for (auto v : _)` syntax.
  As it is not possible to have one way to iterate through the result
  independent of the boost version, a preprocessing directive selects
  the proper synactic iteration construction depending on the provided
  boost-asio library version [2].
  This way, this commit is backwards compatible with boost < 1.66.0
  and compiles properly with boost >= 1.66.0.
  The issue was identified in a build with the cross-compilation tool
  chain provided in the meta-ros OpenEmbedded layer [3].
  [1] http://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/net_ts.html
  [2] https://github.com/boostorg/asio/commit/0c9cbdfbf217146c096265b5eb56089e8cebe608
  [3] http://github.com/bmwcarit/meta-ros
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@gmail.com>
* Contributors: Lukas Bulwahn

0.23.0 (2018-02-03)
-------------------
* libmavconn: warn->debug table entry message
* Contributors: Anthony Lamping

0.22.0 (2017-12-11)
-------------------

0.21.5 (2017-11-16)
-------------------

0.21.4 (2017-11-01)
-------------------
* cmake: do not warn about datasets, only abuse CI where that messages threated as a problem.
* Contributors: Vladimir Ermakov

0.21.3 (2017-10-28)
-------------------

0.21.2 (2017-09-25)
-------------------

0.21.1 (2017-09-22)
-------------------

0.21.0 (2017-09-14)
-------------------

0.20.1 (2017-08-28)
-------------------
* lib: Fix compilation with mavlink 2017.8.26
* Contributors: Vladimir Ermakov

0.20.0 (2017-08-23)
-------------------
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
* Move FindGeographicLib.cmake to libmavconn, that simplify installation, simplify datasets instattator
* Contributors: Nuno Marques, Vladimir Ermakov

0.19.0 (2017-05-05)
-------------------

0.18.7 (2017-02-24)
-------------------
* readme: Add serial-hwfc:// proto
* libmavconn `#649 <https://github.com/mavlink/mavros/issues/649>`_: Add serial-hwfc:// proto (serial + hardware flow control)
  Note: not all platforms support setting
  Boost::asio::serial_port_base::flow_control::hardware option.
* Contributors: Vladimir Ermakov

0.18.6 (2017-02-07)
-------------------
* lib `#626 <https://github.com/mavlink/mavros/issues/626>`_: Porting of PR `#650 <https://github.com/mavlink/mavros/issues/650>`_ - Fix OSX pthread set name.
* Contributors: Fadri Furrer

0.18.5 (2016-12-12)
-------------------

0.18.4 (2016-11-11)
-------------------
* Update README for all packages
* Contributors: Vladimir Ermakov

0.18.3 (2016-07-07)
-------------------
* libmavconn: Enable autoquad dialect. It fixed in mavlink 2016.7.7
* Contributors: Vladimir Ermakov

0.18.2 (2016-06-30)
-------------------
* Revert "libmavconn: Update console_bridge macroses."
  This reverts commit 73fd7f755ed919bc3c170574f514ba6525cd31a2.
  It breaks Travis builds for Indigo and Jade.
* libmavconn: Update console_bridge macroses.
  https://github.com/ros/console_bridge/issues/18
* libmavconn: tcp: enable_shared_from_this
* libmavconn: udp: enable_shared_from_this
* libmavconn: serial: enable_shared_from_this
* libmavconn: std::deque automatically free buffers
* libmavconn fix `#567 <https://github.com/mavlink/mavros/issues/567>`_: Fix tcp server stat calculation
* libmavconn: Fix debug log conn_id
* Contributors: Vladimir Ermakov

0.18.1 (2016-06-24)
-------------------

0.18.0 (2016-06-23)
-------------------
* libmavconn: Fix _KiB literal
* readme `#544 <https://github.com/mavlink/mavros/issues/544>`_: add udp-b://@ URL
* libmavconn fix `#544 <https://github.com/mavlink/mavros/issues/544>`_: New URL for UDP Broadcast (for GCS discovery)
  Broadcast v4 address used until GCS respond.
  udp-b://[bind_host][:bind_port]@[:remote_port]
* libmavconn: fix context.py.in
* libmavconn: Add protocol version selection helpers
* libmavconn: Use monotonic id for logging. Looks better than this ptr.
* node: Update plugin loading and message routing
* node: Rename plugib base class - API incompatible to old class
* labmavconn: remove set_thread_name(), add utils::format()
* libmavconn: APM dialect should be second
* libmavconn fix `#522 <https://github.com/mavlink/mavros/issues/522>`_: place generated files in source tree.
* libmavconn: Use EmPy to generate dialect-enabling files
* libmavconn: update copyright year
* libmavconn: update unit test
* libmavconn: Replace sig-slot with simple std::function() callbacks
* libmavconn: Limit send_message() queue maximum size.
* libmavconn:udp: try to make STL container handle allocations
* libmavconn: Use std::call_once() for init
* libmavconn: Leak in send_message() when it called from self IO thread (such as message_received event)
* libmavconn: update unit test
* libmavconn: support C++ serialization. Warn: RX leaks somewhere.
* libmavconn: Use MAVLink2 C++11
* labmavconn: trying to merge all dialects
* libmavconn: std::thread are invalidated before set_thread_name() called. Result is SIGSEGV
* labmavconn: finding sigsegv
* libmavconn: uncrustify
* libmavconn `#543 <https://github.com/mavlink/mavros/issues/543>`_: remove boost::signals2 (TCP)
* libmavconn `#543 <https://github.com/mavlink/mavros/issues/543>`_: remove boost::signals2 (UDP)
* libmavconn `#543 <https://github.com/mavlink/mavros/issues/543>`_: remove boost.signals2 (serial)
* libmavconn: uncrustify all
* mavconn: Import Simple Signal library (with some minor modifications).
  Source file can be found here:
  https://testbit.eu/cpp11-signal-system-performance/
* Contributors: Vladimir Ermakov

0.17.3 (2016-05-20)
-------------------
* libmavconn `#543 <https://github.com/mavlink/mavros/issues/543>`_: support build with mavlink 2.0 capable mavgen
* Contributors: Vladimir Ermakov

0.17.2 (2016-04-29)
-------------------

0.17.1 (2016-03-28)
-------------------
* MAVConnSerial: Stop io_service before closing serial device (Fixes `#130 <https://github.com/mavlink/mavros/issues/130>`_)
  The serial device was closed before calling io_service.stop() so io_service::run() never returned, leading to hang on join in MAVConnSerial::close()

  .. code-block::

    Backtrace:
    #0  0x00007f80217e966b in pthread_join (threadid=140188059690752, thread_return=0x0) at pthread_join.c:92
    #1  0x00007f80215602d7 in std::thread::join() ()
    #2  0x00007f8020ccc674 in mavconn::MAVConnSerial::close() ()
    #3  0x00007f8020ccc6f5 in mavconn::MAVConnSerial::~MAVConnSerial() ()
    #4  0x00007f8020cc7b2e in boost::detail::sp_counted_impl_pd<mavconn::MAVConnSerial*, boost::detail::sp_ms_deleter<mavconn::MAVConnSerial> >::dispose() ()
    #5  0x000000000040ee0a in boost::detail::sp_counted_base::release() [clone .part.27] [clone .constprop.472] ()
    #6  0x000000000041eb22 in mavros::MavRos::~MavRos() ()
    #7  0x000000000040eb38 in main ()
* Contributors: Kartik Mohta

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
* libmavconn `#452 <https://github.com/mavlink/mavros/issues/452>`_: remove pixhawk, add paparazzi dialects.
  Mavlink package provide information about known dialects,
  so we do not touch mavlink_dialect.h selection ifs.
* Contributors: Vladimir Ermakov

0.16.3 (2015-11-19)
-------------------

0.16.2 (2015-11-17)
-------------------

0.16.1 (2015-11-13)
-------------------

0.16.0 (2015-11-09)
-------------------

0.15.0 (2015-09-17)
-------------------

0.14.2 (2015-08-20)
-------------------

0.14.1 (2015-08-19)
-------------------

0.14.0 (2015-08-17)
-------------------

0.13.1 (2015-08-05)
-------------------

0.13.0 (2015-08-01)
-------------------
* libmavconn: simpify exception code.
* Contributors: Vladimir Ermakov

0.12.0 (2015-07-01)
-------------------
* libmavconn: UDP: Do not exit on Network unreachable error.
  Requested by @mhkabir, idea given by @adamantivm in
  https://github.com/algron/mavros/commit/48fa19f58786387b4aee804e0687d6d39a127806
* Contributors: Vladimir Ermakov

0.11.2 (2015-04-26)
-------------------
* libmavconn fix `#269 <https://github.com/vooon/mavros/issues/269>`_: override default channel getter helpers
  Default inlined mavlink getter helpers cause issue, when each
  plugin has it's own sequence number.
* libmavconn `#269 <https://github.com/vooon/mavros/issues/269>`_: add seq number to debug
* Contributors: Vladimir Ermakov

0.11.1 (2015-04-06)
-------------------

0.11.0 (2015-03-24)
-------------------
* readme: fix links
* license `#242 <https://github.com/vooon/mavros/issues/242>`_: add license files
* license `#242 <https://github.com/vooon/mavros/issues/242>`_: update libmavconn headers
* libmavconn: Fix logging (now all connections use same log name)
  Before i got several names: URL, serial0..
  But severity only changes if i changed first registered tag (URL).
  Now all debug will be enabled by one tag: `ros.rosconsole_bridge.mavconn`
  And because its only used for debugging that was ok.
* Contributors: Vladimir Ermakov

0.10.2 (2015-02-25)
-------------------
* mavconn: fix readme link
* mavconn: Licensed under BSD 3-clause too, update headers for LGPLv3.
  PX4 team asked me to support BSD license.
* Contributors: Vladimir Ermakov

0.10.1 (2015-02-02)
-------------------
* libmavconn: Workaround for gcc 4.6 <chrono>.
* libmavconn: Use C++11 for lists for_each
* Contributors: Vladimir Ermakov

0.10.0 (2015-01-24)
-------------------
* libmavconn `#154 <https://github.com/vooon/mavros/issues/154>`_: Stat sum for tcp server mode.
* libmavconn `#154 <https://github.com/vooon/mavros/issues/154>`_: Add IO usage statistics.
  TODO: tcp-l.
* libmavconn: Fix coverity CID 85784 (use of freed object)
* Contributors: Vladimir Ermakov

0.9.4 (2015-01-06)
------------------

0.9.3 (2014-12-30)
------------------
* mavconn: Add ASLUAV dialect selection.
* Contributors: Vladimir Ermakov

0.9.2 (2014-11-04)
------------------
* Fix libmavconn include destination.
  Before that change headers installed in include/libmavconn (package name)
  and it broke release builds for 0.9.1 and 0.8.4.
  Strange that prerelease build runs without errors.
  Issue `#162 <https://github.com/vooon/mavros/issues/162>`_.
* Contributors: Vladimir Ermakov

0.9.1 (2014-11-03)
------------------
* Fix libmavconn deps.
  Releases 0.9 and 0.8.3 ar broken because i forgot to add mavlink dep.
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
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: try to fix hydro build
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Move mavconn tests.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Fix headers used in mavros. Add readme.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Fix mavros build.
* mavconn `#161 <https://github.com/vooon/mavros/issues/161>`_: Move library to its own package
  Also rosconsole replaced by console_bridge, so now library can be used
  without ros infrastructure.
* Contributors: Vladimir Ermakov
