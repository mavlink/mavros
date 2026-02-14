# home_position

- File: `mavros/src/plugins/home_position.cpp`
- Class: `mavros::std_plugins::HomePositionPlugin`
- Namespace: `home_position`
- Brief: home position plugin.


Publishes home position.

## Publishers
- `~/home` (mavros_msgs::msg::HomePosition)


## Subscribers
- `~/set` (mavros_msgs::msg::HomePosition)


## Services
- `~/req_update` (std_srvs::srv::Trigger)


## Clients
- `cmd/command` (mavros_msgs::srv::CommandLong)


## Parameters
- None


## MAVLink Subscriptions
- `HOME_POSITION` [handler: handle_home_position, dialect: common, msg_id: 242, id: `mavlink::common::msg::HOME_POSITION::MSG_ID`]


## MAVLink Publications
- `SET_HOME_POSITION` [arg: `hp`, dialect: common, msg_id: 243, id: `mavlink::common::msg::SET_HOME_POSITION::MSG_ID`] - [[[end]]] (sum: CGXV/MH3oV)
