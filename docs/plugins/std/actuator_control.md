# actuator_control

- File: `mavros/src/plugins/actuator_control.cpp`
- Class: `mavros::std_plugins::ActuatorControlPlugin`
- Namespace: `actuator_control`
- Brief: ActuatorControl plugin


Sends actuator controls to FCU controller.

## Publishers
- `target_actuator_control` (mavros_msgs::msg::ActuatorControl)


## Subscribers
- `actuator_control` (mavros_msgs::msg::ActuatorControl)


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`ACTUATOR_CONTROL_TARGET`](https://mavlink.io/en/messages/common.html#ACTUATOR_CONTROL_TARGET) [handler: handle_actuator_control_target, dialect: common, msg_id: 140, id: `mavlink::common::msg::ACTUATOR_CONTROL_TARGET::MSG_ID`]


## MAVLink Publications
- [`SET_ACTUATOR_CONTROL_TARGET`](https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET) [arg: `act`, dialect: common, msg_id: 139, id: `mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET::MSG_ID`]
