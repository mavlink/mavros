# sys_status

- File: `mavros/src/plugins/sys_status.cpp`
- Class: `mavros::std_plugins::SystemStatusPlugin`
- Namespace: `sys`
- Brief: System status plugin.


Required by all plugins.

## Publishers
- `state` (mavros_msgs::msg::State)
- `extended_state` (mavros_msgs::msg::ExtendedState)
- `sys_status` (mavros_msgs::msg::SysStatus)
- `estimator_status` (mavros_msgs::msg::EstimatorStatus)
- `battery` (BatteryMsg)
- `statustext/recv` (mavros_msgs::msg::StatusText)
- `status_event` (mavros_msgs::msg::StatusEvent)


## Subscribers
- `statustext/send` (mavros_msgs::msg::StatusText)


## Services
- `set_mode` (mavros_msgs::srv::SetMode)
- `set_stream_rate` (mavros_msgs::srv::StreamRate)
- `set_message_interval` (mavros_msgs::srv::MessageInterval)
- `vehicle_info_get` (mavros_msgs::srv::VehicleInfoGet)


## Clients
- `cmd/command` (mavros_msgs::srv::CommandLong)


## Parameters
- `conn_timeout` [type: double, default: `10.0`]
- `min_voltage` [default: `std::vector<double>({10.0})`]
- `disable_diag` [type: bool, default: `false`]
- `heartbeat_mav_type` [default: `utils::enum_to_name(
        conn_heartbeat_mav_type)`]
- `heartbeat_rate` [type: double, default: `1.0`]


## MAVLink Subscriptions
- [`HEARTBEAT`](https://mavlink.io/en/messages/minimal.html#HEARTBEAT) [handler: handle_heartbeat, dialect: minimal, msg_id: 0, id: `mavlink::minimal::msg::HEARTBEAT::MSG_ID`]
- [`SYS_STATUS`](https://mavlink.io/en/messages/common.html#SYS_STATUS) [handler: handle_sys_status, dialect: common, msg_id: 1, id: `mavlink::common::msg::SYS_STATUS::MSG_ID`]
- [`STATUSTEXT`](https://mavlink.io/en/messages/common.html#STATUSTEXT) [handler: handle_statustext, dialect: common, msg_id: 253, id: `mavlink::common::msg::STATUSTEXT::MSG_ID`]
- [`EVENT`](https://mavlink.io/en/messages/common.html#EVENT) [handler: handle_event, dialect: common, msg_id: 410, id: `mavlink::common::msg::EVENT::MSG_ID`]
- [`MEMINFO`](https://mavlink.io/en/messages/ardupilotmega.html#MEMINFO) [handler: handle_meminfo, dialect: ardupilotmega, msg_id: 152, id: `mavlink::ardupilotmega::msg::MEMINFO::MSG_ID`]
- [`HWSTATUS`](https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS) [handler: handle_hwstatus, dialect: ardupilotmega, msg_id: 165, id: `mavlink::ardupilotmega::msg::HWSTATUS::MSG_ID`]
- [`AUTOPILOT_VERSION`](https://mavlink.io/en/messages/standard.html#AUTOPILOT_VERSION) [handler: handle_autopilot_version, dialect: standard, msg_id: 148, id: `mavlink::standard::msg::AUTOPILOT_VERSION::MSG_ID`]
- [`EXTENDED_SYS_STATE`](https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE) [handler: handle_extended_sys_state, dialect: common, msg_id: 245, id: `mavlink::common::msg::EXTENDED_SYS_STATE::MSG_ID`]
- [`BATTERY_STATUS`](https://mavlink.io/en/messages/common.html#BATTERY_STATUS) [handler: handle_battery_status, dialect: common, msg_id: 147, id: `mavlink::common::msg::BATTERY_STATUS::MSG_ID`]
- [`ESTIMATOR_STATUS`](https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS) [handler: handle_estimator_status, dialect: common, msg_id: 230, id: `mavlink::common::msg::ESTIMATOR_STATUS::MSG_ID`]


## MAVLink Publications
- [`HEARTBEAT`](https://mavlink.io/en/messages/minimal.html#HEARTBEAT) [arg: `hb`, dialect: minimal, msg_id: 0, id: `mavlink::minimal::msg::HEARTBEAT::MSG_ID`]
- [`STATUSTEXT`](https://mavlink.io/en/messages/common.html#STATUSTEXT) [arg: `statustext`, dialect: common, msg_id: 253, id: `mavlink::common::msg::STATUSTEXT::MSG_ID`]
- [`REQUEST_DATA_STREAM`](https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM) [arg: `rq`, dialect: common, msg_id: 66, id: `mavlink::common::msg::REQUEST_DATA_STREAM::MSG_ID`]
- [`SET_MODE`](https://mavlink.io/en/messages/common.html#SET_MODE) [arg: `sm`, dialect: common, msg_id: 11, id: `mavlink::common::msg::SET_MODE::MSG_ID`]
