^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libmavconn
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
