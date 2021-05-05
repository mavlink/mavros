^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libmavconn
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
