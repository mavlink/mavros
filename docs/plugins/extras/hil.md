# hil

- File: `mavros_extras/src/plugins/hil.cpp`
- Class: `mavros::extra_plugins::HilPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Hil plugin


## Publishers
- `~/controls` [type: [mavros_msgs::msg::HilControls](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilControls.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish HIL_CONTROLS from the FCU.
- `~/actuator_controls` [type: [mavros_msgs::msg::HilActuatorControls](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilActuatorControls.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish HIL_ACTUATOR_CONTROLS from the FCU.

## Subscribers
- `~/state` [type: [mavros_msgs::msg::HilStateQuaternion](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilStateQuaternion.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to receive HIL_STATE_QUATERNION from simulation.
- `~/gps` [type: [mavros_msgs::msg::HilGPS](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilGPS.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to receive HIL_GPS from simulation.
- `~/imu_ned` [type: [mavros_msgs::msg::HilSensor](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilSensor.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to receive HIL_SENSOR from simulation.
- `~/optical_flow` [type: [mavros_msgs::msg::OpticalFlowRad](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlowRad.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to receive HIL_OPTICAL_FLOW from simulation.
- `~/rc_inputs` [type: [mavros_msgs::msg::RCIn](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RCIn.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to receive HIL_RC_INPUTS_RAW from simulation.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`HIL_CONTROLS`](https://mavlink.io/en/messages/common.html#HIL_CONTROLS) [handler: handle_hil_controls, dialect: common, msg_id: 91, id: `mavlink::common::msg::HIL_CONTROLS::MSG_ID`]
- [`HIL_ACTUATOR_CONTROLS`](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) [handler: handle_hil_actuator_controls, dialect: common, msg_id: 93, id: `mavlink::common::msg::HIL_ACTUATOR_CONTROLS::MSG_ID`]


## MAVLink Publications
- [`HIL_GPS`](https://mavlink.io/en/messages/common.html#HIL_GPS) [arg: `msg`, dialect: common, msg_id: 113, id: `mavlink::common::msg::HIL_GPS::MSG_ID`]
- [`HIL_OPTICAL_FLOW`](https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW) [arg: `msg`, dialect: common, msg_id: 114, id: `mavlink::common::msg::HIL_OPTICAL_FLOW::MSG_ID`]
- [`HIL_RC_INPUTS_RAW`](https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW) [arg: `msg`, dialect: common, msg_id: 92, id: `mavlink::common::msg::HIL_RC_INPUTS_RAW::MSG_ID`]
- [`HIL_SENSOR`](https://mavlink.io/en/messages/common.html#HIL_SENSOR) [arg: `msg`, dialect: common, msg_id: 107, id: `mavlink::common::msg::HIL_SENSOR::MSG_ID`]
- [`HIL_STATE_QUATERNION`](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) [arg: `msg`, dialect: common, msg_id: 115, id: `mavlink::common::msg::HIL_STATE_QUATERNION::MSG_ID`]
