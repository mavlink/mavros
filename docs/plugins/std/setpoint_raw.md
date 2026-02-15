# setpoint_raw

- File: `mavros/src/plugins/setpoint_raw.cpp`
- Class: `mavros::std_plugins::SetpointRawPlugin`
- Namespace: `setpoint_raw`
- Brief: Setpoint RAW plugin


Send position setpoints and publish current state (return loop). User can decide what set of filed needed for operation via IGNORE bits.

## Publishers
- `~/target_local` (mavros_msgs::msg::PositionTarget)
- `~/target_global` (mavros_msgs::msg::GlobalPositionTarget)
- `~/target_attitude` (mavros_msgs::msg::AttitudeTarget)


## Subscribers
- `~/local` (mavros_msgs::msg::PositionTarget)
- `~/global` (mavros_msgs::msg::GlobalPositionTarget)
- `~/attitude` (mavros_msgs::msg::AttitudeTarget)


## Services
- None


## Clients
- None


## Parameters
- `thrust_scaling` [default: `NAN`]


## MAVLink Subscriptions
- [`POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED) [handler: handle_position_target_local_ned, dialect: common, msg_id: 85, id: `mavlink::common::msg::POSITION_TARGET_LOCAL_NED::MSG_ID`]
- [`POSITION_TARGET_GLOBAL_INT`](https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT) [handler: handle_position_target_global_int, dialect: common, msg_id: 87, id: `mavlink::common::msg::POSITION_TARGET_GLOBAL_INT::MSG_ID`]
- [`ATTITUDE_TARGET`](https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET) [handler: handle_attitude_target, dialect: common, msg_id: 83, id: `mavlink::common::msg::ATTITUDE_TARGET::MSG_ID`]


## MAVLink Publications
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `sp`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
- [`SET_POSITION_TARGET_GLOBAL_INT`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT) [arg: `sp`, dialect: common, msg_id: 86, id: `mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT::MSG_ID`]
- [`SET_ATTITUDE_TARGET`](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) [arg: `sp`, dialect: common, msg_id: 82, id: `mavlink::common::msg::SET_ATTITUDE_TARGET::MSG_ID`]
