# home_position

- File: `mavros/src/plugins/home_position.cpp`
- Class: `mavros::std_plugins::HomePositionPlugin`
- Namespace: `home_position`
- Brief: home position plugin.


Publishes home position.

## Publishers
- `~/home` ([mavros_msgs::msg::HomePosition](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HomePosition.html))


## Subscribers
- `~/set` ([mavros_msgs::msg::HomePosition](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HomePosition.html))


## Services
- `~/req_update` ([std_srvs::srv::Trigger](https://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html))


## Clients
- `cmd/command` ([mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html))


## Parameters
- None


## MAVLink Subscriptions
- [`HOME_POSITION`](https://mavlink.io/en/messages/common.html#HOME_POSITION) [handler: handle_home_position, dialect: common, msg_id: 242, id: `mavlink::common::msg::HOME_POSITION::MSG_ID`]


## MAVLink Publications
- [`SET_HOME_POSITION`](https://mavlink.io/en/messages/common.html#SET_HOME_POSITION) [arg: `hp`, dialect: common, msg_id: 243, id: `mavlink::common::msg::SET_HOME_POSITION::MSG_ID`]
