mavros extras
=============

Some extra plugins and nodes for [mavros][mr].


servo\_state\_publisher
-----------------------

Convert `mavros_msgs/RCOut` to `sensor_msgs/JointState` messages.
It is required to bind URDF model and real servos.


px4flow
-------

Plugin for mavros. Publish `OPTICAL_FLOW_RAD` data.


mocap\_pose\_estimate
----------------------

Plugin for mavros, to send mocap data to FCU.  Currently, not used by the FCU.
Data can be send via `vision_position` plugin.


px-ros-pkg replacement
----------------------

Use `roslaunch mavros_extras px4flow.launch` for that.


[mr]: https://github.com/mavlink/mavros
