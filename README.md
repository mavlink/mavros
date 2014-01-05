MAVROS
======

MAVLink extendable communication node for ROS
with UDP proxy for Ground Control Station (e.g. [QGroundControl][1]).


Feutures
--------

  * Communication with autopilot via serial port (e.g. [ArduPilot][2])
  * UDP proxy for Ground Control Station
  * [mavlink\_ros][3] compatible ROS topics (Mavlink.msg)
  * Plugin system for ROS-MAVLink translation


Limitations
-----------

Only for linux. Depends on [Boost library][4] >= 1.49.
Catkin build system required (tested with ROS Hydro Medusa).


Testing
-------

Simple communication library test (serial-udp bridge):

    catkin_make mavudpproxy && ./devel/lib/mavros/mavudpproxy /dev/ttyACM0 115200


Links
-----

  * [MAVLink][5] -- communication protocol
  * [mavlink\_ros][3] -- original ROS node (few messages, no proxy)
  * [ArduPilot][2] -- tested autopilot APM:Plane (default command set)
  * [QGroundControl][1] -- tested ground control station for linux
  * [DroidPlanner][6] -- tested GCS for Android


[1]: http://qgroundcontrol.org/
[2]: http://ardupilot.com/
[3]: https://github.com/mavlink/mavlink_ros
[4]: http://www.boost.org/
[5]: http://mavlink.org/mavlink/start
[6]: https://github.com/arthurbenemann/droidplanner/
