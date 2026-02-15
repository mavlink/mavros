# command

- File: `mavros/src/plugins/command.cpp`
- Class: `mavros::std_plugins::CommandPlugin`
- Namespace: `cmd`
- Brief: Command plugin.


Send any command via COMMAND_LONG

## Publishers
- None


## Subscribers
- None


## Services
- `~/command` (mavros_msgs::srv::CommandLong)
- `~/command_int` (mavros_msgs::srv::CommandInt)
- `~/arming` (mavros_msgs::srv::CommandBool)
- `~/set_home` (mavros_msgs::srv::CommandHome)
- `~/takeoff` (mavros_msgs::srv::CommandTOL)
- `~/takeoff_local` (mavros_msgs::srv::CommandTOLLocal)
- `~/land` (mavros_msgs::srv::CommandTOL)
- `~/land_local` (mavros_msgs::srv::CommandTOLLocal)
- `~/trigger_control` (mavros_msgs::srv::CommandTriggerControl)
- `~/trigger_interval` (mavros_msgs::srv::CommandTriggerInterval)
- `~/vtol_transition` (mavros_msgs::srv::CommandVtolTransition)


## Clients
- None


## Parameters
- `command_ack_timeout` [type: double, default: `command_ack_timeout_dt.seconds()`]
- `use_comp_id_system_control` [type: bool, default: `false`]


## MAVLink Subscriptions
- [`COMMAND_ACK`](https://mavlink.io/en/messages/common.html#COMMAND_ACK) [handler: handle_command_ack, dialect: common, msg_id: 77, id: `mavlink::common::msg::COMMAND_ACK::MSG_ID`]


## MAVLink Publications
- [`COMMAND_LONG`](https://mavlink.io/en/messages/common.html#COMMAND_LONG) [arg: `cmd`, dialect: common, msg_id: 76, id: `mavlink::common::msg::COMMAND_LONG::MSG_ID`]
- [`COMMAND_INT`](https://mavlink.io/en/messages/common.html#COMMAND_INT) [arg: `cmd`, dialect: common, msg_id: 75, id: `mavlink::common::msg::COMMAND_INT::MSG_ID`]
