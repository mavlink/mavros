^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libmavconn
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
