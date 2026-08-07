# hil

- File: `mavros_extras/src/plugins/hil.cpp`
- Class: `mavros::extra_plugins::HilPlugin`
- Namespace: `hil`
- Brief: Hil plugin


## Publishers
- `~/controls` ([mavros_msgs::msg::HilControls](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilControls.html)) - Publish HIL_CONTROLS from the FCU.
- `~/actuator_controls` ([mavros_msgs::msg::HilActuatorControls](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilActuatorControls.html)) - Publish HIL_ACTUATOR_CONTROLS from the FCU.


## Subscribers
- `~/state` ([mavros_msgs::msg::HilStateQuaternion](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilStateQuaternion.html)) - Subscribe to receive HIL_STATE_QUATERNION from simulation.
- `~/gps` ([mavros_msgs::msg::HilGPS](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilGPS.html)) - Subscribe to receive HIL_GPS from simulation.
- `~/imu_ned` ([mavros_msgs::msg::HilSensor](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HilSensor.html)) - Subscribe to receive HIL_SENSOR from simulation.
- `~/optical_flow` ([mavros_msgs::msg::OpticalFlowRad](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpticalFlowRad.html)) - Subscribe to receive HIL_OPTICAL_FLOW from simulation.
- `~/rc_inputs` ([mavros_msgs::msg::RCIn](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RCIn.html)) - Subscribe to receive HIL_RC_INPUTS_RAW from simulation.


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
- [`HIL_STATE_QUATERNION`](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) [arg: `state_quat`, dialect: common, msg_id: 115, id: `mavlink::common::msg::HIL_STATE_QUATERNION::MSG_ID`]
- [`HIL_GPS`](https://mavlink.io/en/messages/common.html#HIL_GPS) [arg: `gps`, dialect: common, msg_id: 113, id: `mavlink::common::msg::HIL_GPS::MSG_ID`]
- [`HIL_SENSOR`](https://mavlink.io/en/messages/common.html#HIL_SENSOR) [arg: `sensor`, dialect: common, msg_id: 107, id: `mavlink::common::msg::HIL_SENSOR::MSG_ID`]
- [`HIL_OPTICAL_FLOW`](https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW) [arg: `of`, dialect: common, msg_id: 114, id: `mavlink::common::msg::HIL_OPTICAL_FLOW::MSG_ID`]
- [`HIL_RC_INPUTS_RAW`](https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW) [arg: `rcin`, dialect: common, msg_id: 92, id: `mavlink::common::msg::HIL_RC_INPUTS_RAW::MSG_ID`]
