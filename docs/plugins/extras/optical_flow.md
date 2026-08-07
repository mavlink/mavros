# optical_flow

- File: `mavros_extras/src/plugins/optical_flow.cpp`
- Class: `mavros::extra_plugins::OpticalFlowPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Optical Flow custom plugin


This plugin can publish data from OpticalFlow camera to ROS

## Publishers
- `~/raw/optical_flow` [type: [mavros_msgs::msg::OpticalFlow](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlow.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish optical flow from MAVLink OPTICAL_FLOW.
- `~/ground_distance` [type: [sensor_msgs::msg::Range](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Range.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish ground distance as Range from OPTICAL_FLOW.

## Subscribers
- `~/raw/send` [type: [mavros_msgs::msg::OpticalFlow](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlow.html), qos: [QoS(1)](../qos.md#qos_1_)] - Subscribe to OpticalFlow to send as OPTICAL_FLOW to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"optical_flow"`] - Frame id for published optical flow messages.
- `ranger_fov` [type: double, default: `0.119428926`] - Default rangefinder field of view [rad].
- `ranger_min_range` [type: double, default: `0.3`] - Minimum rangefinder range [m].
- `ranger_max_range` [type: double, default: `5.0`] - Maximum rangefinder range [m].


## MAVLink Subscriptions
- [`OPTICAL_FLOW`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW) [handler: handle_optical_flow, dialect: common, msg_id: 100, id: `mavlink::common::msg::OPTICAL_FLOW::MSG_ID`]


## MAVLink Publications
- [`OPTICAL_FLOW`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW) [arg: `msg`, dialect: common, msg_id: 100, id: `mavlink::common::msg::OPTICAL_FLOW::MSG_ID`]
