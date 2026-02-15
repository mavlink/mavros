# esc_status

- File: `mavros_extras/src/plugins/esc_status.cpp`
- Class: `mavros::extra_plugins::ESCStatusPlugin`
- Namespace: `esc_status`
- Brief: ESC status plugin


## Publishers
- `~/info` (mavros_msgs::msg::ESCInfo)
- `~/status` (mavros_msgs::msg::ESCStatus)


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
