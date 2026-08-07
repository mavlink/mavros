# command

- File: `mavros/src/plugins/command.cpp`
- Class: `mavros::std_plugins::CommandPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Command plugin.


Send any command via COMMAND_LONG. Implements the
[MAVLink Command Protocol](https://mavlink.io/en/services/command.html).

## Publishers
- None

## Subscribers
- None

## Services
- `~/command` [type: [mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Send any MAVLink command via COMMAND_LONG and wait for the ACK.
- `~/command_int` [type: [mavros_msgs::srv::CommandInt](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandInt.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Send any MAVLink command via COMMAND_INT (no ACK wait).
- `~/arming` [type: [mavros_msgs::srv::CommandBool](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandBool.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Arm or disarm the motors (MAV_CMD_COMPONENT_ARM_DISARM).
- `~/set_home` [type: [mavros_msgs::srv::CommandHome](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandHome.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Set the home position (MAV_CMD_DO_SET_HOME).
- `~/takeoff` [type: [mavros_msgs::srv::CommandTOL](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandTOL.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Take off to a given altitude/location (MAV_CMD_NAV_TAKEOFF).
- `~/takeoff_local` [type: [mavros_msgs::srv::CommandTOLLocal](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandTOLLocal.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Take off in local frame (MAV_CMD_NAV_TAKEOFF_LOCAL).
- `~/land` [type: [mavros_msgs::srv::CommandTOL](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandTOL.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Land at a given location (MAV_CMD_NAV_LAND).
- `~/land_local` [type: [mavros_msgs::srv::CommandTOLLocal](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandTOLLocal.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Land in local frame (MAV_CMD_NAV_LAND_LOCAL).
- `~/trigger_control` [type: [mavros_msgs::srv::CommandTriggerControl](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandTriggerControl.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Enable/disable/reset the camera trigger (MAV_CMD_DO_TRIGGER_CONTROL).
- `~/trigger_interval` [type: [mavros_msgs::srv::CommandTriggerInterval](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandTriggerInterval.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Set the camera trigger interval (MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL).
- `~/vtol_transition` [type: [mavros_msgs::srv::CommandVtolTransition](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandVtolTransition.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Request a VTOL transition (MAV_CMD_DO_VTOL_TRANSITION).

## Clients
- None


## Parameters
- `command_ack_timeout` [type: double, default: `command_ack_timeout_dt.seconds()`]
- `use_comp_id_system_control` [type: bool, default: `false`]


## MAVLink Subscriptions
- [`COMMAND_ACK`](https://mavlink.io/en/messages/common.html#COMMAND_ACK) [handler: handle_command_ack, dialect: common, msg_id: 77, id: `mavlink::common::msg::COMMAND_ACK::MSG_ID`]


## MAVLink Publications
- [`COMMAND_INT`](https://mavlink.io/en/messages/common.html#COMMAND_INT) [arg: `msg`, dialect: common, msg_id: 75, id: `mavlink::common::msg::COMMAND_INT::MSG_ID`]
- [`COMMAND_LONG`](https://mavlink.io/en/messages/common.html#COMMAND_LONG) [arg: `msg`, dialect: common, msg_id: 76, id: `mavlink::common::msg::COMMAND_LONG::MSG_ID`]
