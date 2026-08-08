# mount_control

- File: `mavros_extras/src/plugins/mount_control.cpp`
- Class: `mavros::extra_plugins::MountControlPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Mount Control plugin


Publishes Mission commands to control the camera or antenna mount. Implements the
superseded [MAVLink Gimbal Protocol v1](https://mavlink.io/en/services/gimbal.html).

## Publishers
- `~/orientation` [type: [geometry_msgs::msg::Quaternion](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Quaternion.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish mount orientation as quaternion from MAVLink MOUNT_ORIENTATION.
- `~/status` [type: [geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish mount status from MAVLink MOUNT_STATUS.

## Subscribers
- `~/command` [type: [mavros_msgs::msg::MountControl](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/MountControl.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to MountControl to send as MAV_CMD_DO_MOUNT_CONTROL to the FCU.

## Services
- `~/configure` [type: [mavros_msgs::srv::MountConfigure](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/MountConfigure.html)] - Configure the mount (MAV_CMD_DO_MOUNT_CONFIGURE).

## Clients
- `cmd/command` [type: [mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html)]


## Parameters
- `negate_measured_roll` [type: bool, default: `false`] - Negate the measured roll angle.
- `negate_measured_pitch` [type: bool, default: `false`] - Negate the measured pitch angle.
- `negate_measured_yaw` [type: bool, default: `false`] - Negate the measured yaw angle.
- `debounce_s` [type: double, default: `4.0`] - Mount diagnostic error debounce time [s].
- `err_threshold_deg` [type: double, default: `10.0`] - Mount diagnostic angle error threshold [deg].
- `disable_diag` [type: bool, default: `false`] - Disable mount diagnostic updater.


## MAVLink Subscriptions
- [`MOUNT_ORIENTATION`](https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION) [handler: handle_mount_orientation, dialect: common, msg_id: 265, id: `mavlink::common::msg::MOUNT_ORIENTATION::MSG_ID`]
- [`MOUNT_STATUS`](https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_STATUS) [handler: handle_mount_status, dialect: ardupilotmega, msg_id: 158, id: `mavlink::ardupilotmega::msg::MOUNT_STATUS::MSG_ID`]


## MAVLink Publications
- [`COMMAND_LONG`](https://mavlink.io/en/messages/common.html#COMMAND_LONG) [arg: `msg`, dialect: common, msg_id: 76, id: `mavlink::common::msg::COMMAND_LONG::MSG_ID`]
