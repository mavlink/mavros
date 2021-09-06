mavros extras
=============

Some extra plugins and nodes for [mavros][mr].


ADSB
----

Publish/subscribe to the location and information of an ADS-B vehicle.


cam\_imu\_sync
--------------

Publish camera trigger data for synchronisation of IMU and camera frames.


debug\_value
------------

Subs/Pubs debug msgs from and to the FCU.


distance\_sensor
----------------

Publish DISTANCE\_SENSOR message data from FCU or connected sensors in companion computer.


fake\_gps
---------

Sends fake GPS from local position estimation source data (motion capture, vision) to FCU.


gps\_input
-----------

Send GPS\_INPUT messages to the FCU.


gps\_status
-----------

Publish GPS\_RAW and GPS\_RTK messages from FCU.


gps\_rtk
--------

Sends the RTCM messages to the FCU for the RTK Fix.


log\_transfer
-------------

Expose firmware functionality, that is related to log transfer


mocap\_pose\_estimate
---------------------

Send motion capture pose estimate to FCU.  Currently, not used by the FCU.
Data can be send via `vision_position` plugin.


obstacle\_distance
------------------

Send obstacle distance report to the FCU.


odom
----

Send odometry to FCU from another estimator.


px4flow
-------

Publish `OPTICAL_FLOW_RAD` data from FCU or PX4Flow module.


rangefinder
-----------

Publish RANGEFINDER message data from FCU sensors in companion computer.


trajectory
----------

Receive planned path from the FCU and send back corrected path (collision free, smoothed) to the FCU.


wheel\_odometry
---------------

Compute and publish wheel odometry coming from FCU sensors.


vibration
---------

Publish VIBRATION message data from FCU.


vision\_pose\_estimate
----------------------

Send vision pose estimate to FCU.


vision\_speed\_estimate
-----------------------

Send vision speed estimate to FCU.


companion\_process\_status
--------------------------

Send companion process status report to the FCU.


servo\_state\_publisher
-----------------------

Convert `mavros_msgs/RCOut` to `sensor_msgs/JointState` messages.
It is required to bind URDF model and real servos.


px-ros-pkg replacement
----------------------

Use `roslaunch mavros_extras px4flow.launch` for that.


[mr]: https://github.com/mavlink/mavros
