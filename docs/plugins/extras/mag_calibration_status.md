# mag_calibration_status

- File: `mavros_extras/src/plugins/mag_calibration_status.cpp`
- Class: `mavros::extra_plugins::MagCalStatusPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: MagCalStatus plugin.


Example and "how to" for users.

## Publishers
- `~/status` [type: [std_msgs::msg::UInt8](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt8.html), qos: [QoS(2)](../qos.md#qos_2_)] - Publish magnetometer calibration progress from MAVLink MAG_CAL_PROGRESS.
- `~/report` [type: [mavros_msgs::msg::MagnetometerReporter](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/MagnetometerReporter.html), qos: [QoS(2)](../qos.md#qos_2_)] - Publish magnetometer calibration report from MAVLink MAG_CAL_REPORT.

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`MAG_CAL_PROGRESS`](https://mavlink.io/en/messages/ardupilotmega.html#MAG_CAL_PROGRESS) [handler: handle_status, dialect: ardupilotmega, msg_id: 191, id: `mavlink::ardupilotmega::msg::MAG_CAL_PROGRESS::MSG_ID`] - Send progress of magnetometer calibration
- [`MAG_CAL_REPORT`](https://mavlink.io/en/messages/common.html#MAG_CAL_REPORT) [handler: handle_report, dialect: common, msg_id: 192, id: `mavlink::common::msg::MAG_CAL_REPORT::MSG_ID`] - Send report after calibration is done


## MAVLink Publications
- None
