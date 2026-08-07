# px4flow

- File: `mavros_extras/src/plugins/px4flow.cpp`
- Class: `mavros::extra_plugins::PX4FlowPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: PX4 Optical Flow plugin


This plugin can publish data from PX4Flow camera to ROS

## Publishers
- `~/raw/optical_flow_rad` [type: [mavros_msgs::msg::OpticalFlowRad](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlowRad.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish optical flow from MAVLink OPTICAL_FLOW_RAD.
- `~/ground_distance` [type: [sensor_msgs::msg::Range](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Range.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish ground distance as Range from OPTICAL_FLOW_RAD.
- `~/temperature` [type: [sensor_msgs::msg::Temperature](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Temperature.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish optical flow temperature as Temperature.

## Subscribers
- `~/raw/send` [type: [mavros_msgs::msg::OpticalFlowRad](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlowRad.html), qos: [QoS(1)](../qos.md#qos_1_)] - Subscribe to OpticalFlowRad to send as OPTICAL_FLOW_RAD to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"px4flow"`] - Frame id for published PX4Flow messages.
- `ranger_fov` [type: double, default: `0.119428926`] - Default rangefinder field of view [rad].
- `ranger_min_range` [type: double, default: `0.3`] - Minimum rangefinder range [m].
- `ranger_max_range` [type: double, default: `5.0`] - Maximum rangefinder range [m].


## MAVLink Subscriptions
- [`OPTICAL_FLOW_RAD`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) [handler: handle_optical_flow_rad, dialect: common, msg_id: 106, id: `mavlink::common::msg::OPTICAL_FLOW_RAD::MSG_ID`]


## MAVLink Publications
- [`OPTICAL_FLOW_RAD`](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) [arg: `msg`, dialect: common, msg_id: 106, id: `mavlink::common::msg::OPTICAL_FLOW_RAD::MSG_ID`]
