# open_drone_id

- File: `mavros_extras/src/plugins/open_drone_id.cpp`
- Class: `mavros::extra_plugins::OpenDroneIDPlugin`
- Namespace: `open_drone_id`
- Brief: Open Drone ID plugin


Sends Open Drone ID data to the FCU

## Publishers
- None


## Subscribers
- `~/basic_id` ([mavros_msgs::msg::OpenDroneIDBasicID](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpenDroneIDBasicID.html))
- `~/operator_id` ([mavros_msgs::msg::OpenDroneIDOperatorID](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpenDroneIDOperatorID.html))
- `~/self_id` ([mavros_msgs::msg::OpenDroneIDSelfID](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpenDroneIDSelfID.html))
- `~/system` ([mavros_msgs::msg::OpenDroneIDSystem](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpenDroneIDSystem.html))
- `~/system_update` ([mavros_msgs::msg::OpenDroneIDSystemUpdate](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OpenDroneIDSystemUpdate.html))


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`OPEN_DRONE_ID_BASIC_ID`](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID) [arg: `basic_id`, dialect: common, msg_id: 12900, id: `mavlink::common::msg::OPEN_DRONE_ID_BASIC_ID::MSG_ID`]
- [`OPEN_DRONE_ID_OPERATOR_ID`](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_OPERATOR_ID) [arg: `operator_id`, dialect: common, msg_id: 12905, id: `mavlink::common::msg::OPEN_DRONE_ID_OPERATOR_ID::MSG_ID`]
- [`OPEN_DRONE_ID_SELF_ID`](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SELF_ID) [arg: `self_id`, dialect: common, msg_id: 12903, id: `mavlink::common::msg::OPEN_DRONE_ID_SELF_ID::MSG_ID`]
- [`OPEN_DRONE_ID_SYSTEM`](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM) [arg: `system`, dialect: common, msg_id: 12904, id: `mavlink::common::msg::OPEN_DRONE_ID_SYSTEM::MSG_ID`]
- [`OPEN_DRONE_ID_SYSTEM_UPDATE`](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SYSTEM_UPDATE) [arg: `system_update`, dialect: common, msg_id: 12919, id: `mavlink::common::msg::OPEN_DRONE_ID_SYSTEM_UPDATE::MSG_ID`]
