# gps_status

- File: `mavros_extras/src/plugins/gps_status.cpp`
- Class: `mavros::extra_plugins::GpsStatusPlugin`
- Namespace: `gpsstatus`
- Brief: Mavlink GPS status plugin.


This plugin publishes GPS sensor data from a Mavlink compatible FCU to ROS.

## Publishers
- `~/gps1/raw` ([mavros_msgs::msg::GPSRAW](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GPSRAW.html))
- `~/gps2/raw` ([mavros_msgs::msg::GPSRAW](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GPSRAW.html))
- `~/gps1/rtk` ([mavros_msgs::msg::GPSRTK](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GPSRTK.html))
- `~/gps2/rtk` ([mavros_msgs::msg::GPSRTK](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GPSRTK.html))


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`GPS_RAW_INT`](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) [handler: handle_gps_raw_int, dialect: common, msg_id: 24, id: `mavlink::common::msg::GPS_RAW_INT::MSG_ID`]
- [`GPS2_RAW`](https://mavlink.io/en/messages/common.html#GPS2_RAW) [handler: handle_gps2_raw, dialect: common, msg_id: 124, id: `mavlink::common::msg::GPS2_RAW::MSG_ID`]
- [`GPS_RTK`](https://mavlink.io/en/messages/common.html#GPS_RTK) [handler: handle_gps_rtk, dialect: common, msg_id: 127, id: `mavlink::common::msg::GPS_RTK::MSG_ID`]
- [`GPS2_RTK`](https://mavlink.io/en/messages/common.html#GPS2_RTK) [handler: handle_gps2_rtk, dialect: common, msg_id: 128, id: `mavlink::common::msg::GPS2_RTK::MSG_ID`]


## MAVLink Publications
- None
