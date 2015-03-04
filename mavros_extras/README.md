mavros extras
=============

Some extra plugins and nodes for [mavros][mr].


px4flow
-------

Plugin for mavros. Publish `OPTICAL_FLOW_RAD` data.


image\_pub
----------

Plugin for mavros, publish images from mavlink device.


gcs\_image\_bridge
------------------

Variation of `gcs_bridge` that additionally sends image stream to GCS.


mocap\_pose\_estimate
----------------------

Plugin for mavros, to send mocap data to FCU.  Currently, not used by the FCU.
Data can be send via `vision_position` plugin.


px-ros-pkg replacement
----------------------

Use `roslaunch mavros_extras px4flow.launch` for that.


[mr]: https://github.com/mavlink/mavros
