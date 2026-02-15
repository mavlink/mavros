# px4flow

- File: `mavros_extras/src/plugins/px4flow.cpp`
- Class: `mavros::extra_plugins::PX4FlowPlugin`
- Namespace: `px4flow`
- Brief: PX4 Optical Flow plugin


This plugin can publish data from PX4Flow camera to ROS

## Publishers
- `~/raw/optical_flow_rad` ([mavros_msgs::msg::OpticalFlowRad](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlowRad.html))
- `~/ground_distance` ([sensor_msgs::msg::Range](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Range.html))
- `~/temperature` ([sensor_msgs::msg::Temperature](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Temperature.html))


## Subscribers
- `~/raw/send` ([mavros_msgs::msg::OpticalFlowRad](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlowRad.html))


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"px4flow"`]
- `ranger_fov` [type: double, default: `0.119428926`] - This is a narrow beam (60cm wide at 5 meters, but also at 1 meter). 6.8 degrees at 5 meters, 31 degrees at 1 meter
- `ranger_min_range` [type: double, default: `0.3`]
- `ranger_max_range` [type: double, default: `5.0`]


## MAVLink Subscriptions
- [`OPTICAL_FLOW_RAD`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) [handler: handle_optical_flow_rad, dialect: common, msg_id: 106, id: `mavlink::common::msg::OPTICAL_FLOW_RAD::MSG_ID`]


## MAVLink Publications
- [`OPTICAL_FLOW_RAD`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) [arg: `flow_rad_msg`, dialect: common, msg_id: 106, id: `mavlink::common::msg::OPTICAL_FLOW_RAD::MSG_ID`]
