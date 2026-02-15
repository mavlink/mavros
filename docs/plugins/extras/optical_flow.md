# optical_flow

- File: `mavros_extras/src/plugins/optical_flow.cpp`
- Class: `mavros::extra_plugins::OpticalFlowPlugin`
- Namespace: `optical_flow`
- Brief: Optical Flow custom plugin


This plugin can publish data from OpticalFlow camera to ROS

## Publishers
- `~/raw/optical_flow` (mavros_msgs::msg::OpticalFlow)
- `~/ground_distance` (sensor_msgs::msg::Range)


## Subscribers
- `~/raw/send` (mavros_msgs::msg::OpticalFlow)


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"optical_flow"`]
- `ranger_fov` [type: double, default: `0.119428926`] - This is a narrow beam (60cm wide at 5 meters, but also at 1 meter). 6.8 degrees at 5 meters, 31 degrees at 1 meter
- `ranger_min_range` [type: double, default: `0.3`]
- `ranger_max_range` [type: double, default: `5.0`]


## MAVLink Subscriptions
- [`OPTICAL_FLOW`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW) [handler: handle_optical_flow, dialect: common, msg_id: 100, id: `mavlink::common::msg::OPTICAL_FLOW::MSG_ID`]


## MAVLink Publications
- [`OPTICAL_FLOW`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW) [arg: `flow_msg`, dialect: common, msg_id: 100, id: `mavlink::common::msg::OPTICAL_FLOW::MSG_ID`]
