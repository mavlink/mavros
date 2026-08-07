# esc_status

- File: `mavros_extras/src/plugins/esc_status.cpp`
- Class: `mavros::extra_plugins::ESCStatusPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: ESC status plugin


## Publishers
- `~/info` [type: [mavros_msgs::msg::ESCInfo](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ESCInfo.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish ESC information from MAVLink ESC_INFO.
- `~/status` [type: [mavros_msgs::msg::ESCStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ESCStatus.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish ESC status from MAVLink ESC_STATUS.

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`ESC_INFO`](https://mavlink.io/en/messages/common.html#ESC_INFO) [handler: handle_esc_info, dialect: common, msg_id: 290, id: `mavlink::common::msg::ESC_INFO::MSG_ID`]
- [`ESC_STATUS`](https://mavlink.io/en/messages/common.html#ESC_STATUS) [handler: handle_esc_status, dialect: common, msg_id: 291, id: `mavlink::common::msg::ESC_STATUS::MSG_ID`]


## MAVLink Publications
- None
