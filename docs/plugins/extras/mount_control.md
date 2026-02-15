# mount_control

- File: `mavros_extras/src/plugins/mount_control.cpp`
- Class: `mavros::extra_plugins::MountControlPlugin`
- Namespace: `mount_control`
- Brief: Mount Control plugin


Publishes Mission commands to control the camera or antenna mount. @see command_cb()

## Publishers
- `~/orientation` ([geometry_msgs::msg::Quaternion](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Quaternion.html))
- `~/status` ([geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html))


## Subscribers
- `~/command` ([mavros_msgs::msg::MountControl](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/MountControl.html))


## Services
- `~/configure` ([mavros_msgs::srv::MountConfigure](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/MountConfigure.html))


## Clients
- `cmd/command` ([mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html))


## Parameters
- `negate_measured_roll` [type: bool, default: `false`]
- `negate_measured_pitch` [type: bool, default: `false`]
- `negate_measured_yaw` [type: bool, default: `false`]
- `debounce_s` [type: double, default: `4.0`]
- `err_threshold_deg` [type: double, default: `10.0`]
- `disable_diag` [type: bool, default: `false`]


## MAVLink Subscriptions
- [`MOUNT_ORIENTATION`](https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION) [handler: handle_mount_orientation, dialect: common, msg_id: 265, id: `mavlink::common::msg::MOUNT_ORIENTATION::MSG_ID`]
- [`MOUNT_STATUS`](https://mavlink.io/en/messages/ardupilotmega.html#MOUNT_STATUS) [handler: handle_mount_status, dialect: ardupilotmega, msg_id: 158, id: `mavlink::ardupilotmega::msg::MOUNT_STATUS::MSG_ID`]


## MAVLink Publications
- [`COMMAND_LONG`](https://mavlink.io/en/messages/common.html#COMMAND_LONG) [arg: `cmd`, dialect: common, msg_id: 76, id: `mavlink::common::msg::COMMAND_LONG::MSG_ID`]
