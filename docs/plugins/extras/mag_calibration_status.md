# mag_calibration_status

- File: `mavros_extras/src/plugins/mag_calibration_status.cpp`
- Class: `mavros::extra_plugins::MagCalStatusPlugin`
- Namespace: `mag_calibration`
- Brief: MagCalStatus plugin.


Example and "how to" for users.

## Publishers
- `~/status` (std_msgs::msg::UInt8) - TODO(vooon): use QoS for "latched" topics
- `~/report` (mavros_msgs::msg::MagnetometerReporter)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `MAG_CAL_PROGRESS` [handler: handle_status, dialect: ardupilotmega, msg_id: 191, id: `mavlink::ardupilotmega::msg::MAG_CAL_PROGRESS::MSG_ID`]
- `MAG_CAL_REPORT` [handler: handle_report, dialect: common, msg_id: 192, id: `mavlink::common::msg::MAG_CAL_REPORT::MSG_ID`]


## MAVLink Publications
- None
