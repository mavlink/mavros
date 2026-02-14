# cellular_status

- File: `mavros_extras/src/plugins/cellular_status.cpp`
- Class: `mavros::extra_plugins::CellularStatusPlugin`
- Namespace: `cellular_status`
- Brief: Cellular status plugin.


Users must publish to the topic the CellularStatus message and it will be relayed to the mavlink components.

## Publishers
- None


## Subscribers
- `~/status` (mavros_msgs::msg::CellularStatus)


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- `CELLULAR_STATUS` [arg: `cs`, dialect: common, msg_id: 334, id: `mavlink::common::msg::CELLULAR_STATUS::MSG_ID`]
