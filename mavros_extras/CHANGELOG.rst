^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_extras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.5 (2017-02-07)
-------------------

0.17.4 (2016-06-23)
-------------------

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
