# log_transfer

- File: `mavros_extras/src/plugins/log_transfer.cpp`
- Class: `mavros::extra_plugins::LogTransferPlugin`
- Namespace: `log_transfer`
- Brief: Log Transfer plugin


## Publishers
- `~/raw/log_entry` (mavros_msgs::msg::LogEntry)
- `~/raw/log_data` (mavros_msgs::msg::LogData)


## Subscribers
- None


## Services
- `~/raw/log_request_list` (mavros_msgs::srv::LogRequestList)
- `~/raw/log_request_data` (mavros_msgs::srv::LogRequestData)
- `~/raw/log_request_end` (mavros_msgs::srv::LogRequestEnd)
- `~/raw/log_request_erase` (std_srvs::srv::Trigger)


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`LOG_ENTRY`](https://mavlink.io/en/messages/common.html#LOG_ENTRY) [handler: handle_log_entry, dialect: common, msg_id: 118, id: `mavlink::common::msg::LOG_ENTRY::MSG_ID`]
- [`LOG_DATA`](https://mavlink.io/en/messages/common.html#LOG_DATA) [handler: handle_log_data, dialect: common, msg_id: 120, id: `mavlink::common::msg::LOG_DATA::MSG_ID`]


## MAVLink Publications
- [`LOG_REQUEST_LIST`](https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST) [arg: `msg`, dialect: common, msg_id: 117, id: `mavlink::common::msg::LOG_REQUEST_LIST::MSG_ID`]
- [`LOG_REQUEST_DATA`](https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA) [arg: `msg`, dialect: common, msg_id: 119, id: `mavlink::common::msg::LOG_REQUEST_DATA::MSG_ID`]
- [`LOG_REQUEST_END`](https://mavlink.io/en/messages/common.html#LOG_REQUEST_END) [arg: `msg`, dialect: common, msg_id: 122, id: `mavlink::common::msg::LOG_REQUEST_END::MSG_ID`]
- [`LOG_ERASE`](https://mavlink.io/en/messages/common.html#LOG_ERASE) [arg: `msg`, dialect: common, msg_id: 121, id: `mavlink::common::msg::LOG_ERASE::MSG_ID`]
