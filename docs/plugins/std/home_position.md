# home_position

- File: `mavros/src/plugins/home_position.cpp`
- Class: `mavros::std_plugins::HomePositionPlugin`
- Namespace: `mavros::std_plugins`
- Brief: home position plugin.


Publishes home position.

## Publishers
- `~/home` [type: [mavros_msgs::msg::HomePosition](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HomePosition.html), qos: [StateQoS](../qos.md#stateqos "StateQoS QoS profile")] - Publish home position (HOME_POSITION).

## Subscribers
- `~/set` [type: [mavros_msgs::msg::HomePosition](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HomePosition.html), qos: [QoS(10)](../qos.md#qos_10_)] - Set home position (SET_HOME_POSITION).

## Services
- `~/req_update` [type: [std_srvs::srv::Trigger](https://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html)] - Request home position update (MAV_CMD_GET_HOME_POSITION).

## Clients
- `cmd/command` [type: [mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html)] - Client to request home position via command (MAV_CMD_GET_HOME_POSITION).


## Parameters
- None


## MAVLink Subscriptions
- [`HOME_POSITION`](https://mavlink.io/en/messages/common.html#HOME_POSITION) [handler: handle_home_position, dialect: common, msg_id: 242, id: `mavlink::common::msg::HOME_POSITION::MSG_ID`]


## MAVLink Publications
- [`SET_HOME_POSITION`](https://mavlink.io/en/messages/common.html#SET_HOME_POSITION) [arg: `msg`, dialect: common, msg_id: 243, id: `mavlink::common::msg::SET_HOME_POSITION::MSG_ID`]
