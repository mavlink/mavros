^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.5 (2017-02-07)
-------------------

0.17.4 (2016-06-23)
-------------------
* Adding anchor to the HIL_CONTROLS message reference link
* Utilizing synchronise_stamp and adding reference to MAVLINK msg documentation
* Added a plugin that publishes HIL_CONTROLS as ROS messages
* Contributors: Pavel

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
