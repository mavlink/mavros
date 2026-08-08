# actuator_control

- File: `mavros/src/plugins/actuator_control.cpp`
- Class: `mavros::std_plugins::ActuatorControlPlugin`
- Namespace: `mavros::std_plugins`
- Brief: ActuatorControl plugin


Sends actuator controls to FCU controller.

## Publishers
- `target_actuator_control` [type: [mavros_msgs::msg::ActuatorControl](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ActuatorControl.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish actuator control target (ACTUATOR_CONTROL_TARGET).

## Subscribers
- `actuator_control` [type: [mavros_msgs::msg::ActuatorControl](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ActuatorControl.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Send actuator control commands to FCU (SET_ACTUATOR_CONTROL_TARGET).

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`ACTUATOR_CONTROL_TARGET`](https://mavlink.io/en/messages/common.html#ACTUATOR_CONTROL_TARGET) [handler: handle_actuator_control_target, dialect: common, msg_id: 140, id: `mavlink::common::msg::ACTUATOR_CONTROL_TARGET::MSG_ID`]


## MAVLink Publications
- [`SET_ACTUATOR_CONTROL_TARGET`](https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET) [arg: `msg`, dialect: common, msg_id: 139, id: `mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET::MSG_ID`]
