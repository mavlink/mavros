^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
