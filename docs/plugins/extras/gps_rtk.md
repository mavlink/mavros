# gps_rtk

- File: `mavros_extras/src/plugins/gps_rtk.cpp`
- Class: `mavros::extra_plugins::GpsRtkPlugin`
- Namespace: `gps_rtk`
- Brief: GPS RTK plugin


1. Publish the RTCM messages from ROS to the FCU 2. Publish RTK baseline data from the FCU to ROS

## Publishers
- `~/rtk_baseline` (mavros_msgs::msg::RTKBaseline) - TODO(vooon): set QoS for latched topic


## Subscribers
- `~/send_rtcm` (mavros_msgs::msg::RTCM)


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`GPS_RTK`](https://mavlink.io/en/messages/common.html#GPS_RTK) [handler: handle_baseline_msg, dialect: common, msg_id: 127, id: `mavlink::common::msg::GPS_RTK::MSG_ID`]


## MAVLink Publications
- [`GPS_RTCM_DATA`](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) [arg: `rtcm_data`, dialect: common, msg_id: 233, id: `mavlink::common::msg::GPS_RTCM_DATA::MSG_ID`]
- [`GPS_RTCM_DATA`](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) [arg: `rtcm_data`, dialect: common, msg_id: 233, id: `mavlink::common::msg::GPS_RTCM_DATA::MSG_ID`]
