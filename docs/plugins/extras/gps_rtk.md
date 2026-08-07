# gps_rtk

- File: `mavros_extras/src/plugins/gps_rtk.cpp`
- Class: `mavros::extra_plugins::GpsRtkPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: GPS RTK plugin


1. Publish the RTCM messages from ROS to the FCU
2. Publish RTK baseline data from the FCU to ROS

## Publishers
- `~/rtk_baseline` [type: [mavros_msgs::msg::RTKBaseline](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RTKBaseline.html), qos: [QoS(1)](../qos.md#qos_1_)] - Publish RTK baseline data from MAVLink GPS_RTK.

## Subscribers
- `~/send_rtcm` [type: [mavros_msgs::msg::RTCM](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RTCM.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to RTCM to send as GPS_RTCM_DATA to the FCU.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`GPS_RTK`](https://mavlink.io/en/messages/common.html#GPS_RTK) [handler: handle_baseline_msg, dialect: common, msg_id: 127, id: `mavlink::common::msg::GPS_RTK::MSG_ID`]


## MAVLink Publications
- [`GPS_RTCM_DATA`](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) [arg: `msg`, dialect: common, msg_id: 233, id: `mavlink::common::msg::GPS_RTCM_DATA::MSG_ID`]
