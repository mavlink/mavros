# sys_status

- File: `mavros/src/plugins/sys_status.cpp`
- Class: `mavros::std_plugins::SystemStatusPlugin`
- Namespace: `mavros::std_plugins`
- Brief: System status plugin.


Required by all plugins.



Implements the
[MAVLink Heartbeat/Connection Protocol](https://mavlink.io/en/services/heartbeat.html).

## Publishers
- `state` [type: [mavros_msgs::msg::State](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/State.html), qos: [StateQoS](../qos.md#stateqos "StateQoS QoS profile")] - Publish connection, armed and mode state (HEARTBEAT).
- `extended_state` [type: [mavros_msgs::msg::ExtendedState](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ExtendedState.html), qos: [StateQoS](../qos.md#stateqos "StateQoS QoS profile")] - Publish VTOL and landed state (EXTENDED_SYS_STATE).
- `sys_status` [type: [mavros_msgs::msg::SysStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/SysStatus.html), qos: [StateQoS](../qos.md#stateqos "StateQoS QoS profile")] - Publish system and battery status (SYS_STATUS).
- `estimator_status` [type: [mavros_msgs::msg::EstimatorStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/EstimatorStatus.html), qos: [StateQoS](../qos.md#stateqos "StateQoS QoS profile")] - Publish estimator status flags (ESTIMATOR_STATUS).
- `battery` [type: [sensor_msgs::msg::BatteryState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/BatteryState.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish battery state (BATTERY_STATUS).
- `statustext/recv` [type: [mavros_msgs::msg::StatusText](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/StatusText.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish status text received from the FCU (STATUSTEXT).
- `status_event` [type: [mavros_msgs::msg::StatusEvent](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/StatusEvent.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish status events received from the FCU (EVENT).

## Subscribers
- `statustext/send` [type: [mavros_msgs::msg::StatusText](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/StatusText.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Send status text to the FCU (STATUSTEXT).

## Services
- `set_mode` [type: [mavros_msgs::srv::SetMode](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/SetMode.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Change the flight mode (MAV_CMD_DO_SET_MODE / SET_MODE).
- `set_stream_rate` [type: [mavros_msgs::srv::StreamRate](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/StreamRate.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Set the stream rate of a MAVLink message (REQUEST_DATA_STREAM).
- `set_message_interval` [type: [mavros_msgs::srv::MessageInterval](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/MessageInterval.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Set the interval of a MAVLink message (MAV_CMD_SET_MESSAGE_INTERVAL).
- `vehicle_info_get` [type: [mavros_msgs::srv::VehicleInfoGet](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/VehicleInfoGet.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Query information about the FCU and connected vehicles.

## Clients
- `cmd/command` [type: [mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html)]
- `cmd/command` [type: [mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html)]


## Parameters
- `conn_timeout` [type: double, default: `10.0`] - Connection timeout in seconds before the link is considered lost.
- `min_voltage` [default: `std::vector<double>({10.0})`] - Minimum battery voltage per battery instance before a warning is raised.
- `disable_diag` [type: bool, default: `false`] - Disable publishing diagnostic updates.
- `heartbeat_mav_type` [default: `utils::enum_to_name(
        conn_heartbeat_mav_type)`] - MAV type of the heartbeat sent by the GCS.
- `heartbeat_rate` [type: double, default: `1.0`] - Rate (Hz) at which the GCS heartbeat is sent to the FCU.


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
- [`HEARTBEAT`](https://mavlink.io/en/messages/minimal.html#HEARTBEAT) [arg: `msg`, dialect: minimal, msg_id: 0, id: `mavlink::minimal::msg::HEARTBEAT::MSG_ID`]
- [`REQUEST_DATA_STREAM`](https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM) [arg: `msg`, dialect: common, msg_id: 66, id: `mavlink::common::msg::REQUEST_DATA_STREAM::MSG_ID`]
- [`SET_MODE`](https://mavlink.io/en/messages/common.html#SET_MODE) [arg: `msg`, dialect: common, msg_id: 11, id: `mavlink::common::msg::SET_MODE::MSG_ID`]
- [`STATUSTEXT`](https://mavlink.io/en/messages/common.html#STATUSTEXT) [arg: `msg`, dialect: common, msg_id: 253, id: `mavlink::common::msg::STATUSTEXT::MSG_ID`]
