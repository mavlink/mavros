# gimbal_control

- File: `mavros_extras/src/plugins/gimbal_control.cpp`
- Class: `mavros::extra_plugins::GimbalControlPlugin`
- Namespace: `gimbal_control`
- Brief: Gimbal Control Plugin


Adds support for Mavlink Gimbal Protocol v2. Also publishes gimbal pose to TF when parameter tf_send==true

## Publishers
- `~/device/attitude_status` ([mavros_msgs::msg::GimbalDeviceAttitudeStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalDeviceAttitudeStatus.html))
- `~/manager/status` ([mavros_msgs::msg::GimbalManagerStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerStatus.html))
- `~/manager/info` ([mavros_msgs::msg::GimbalManagerInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerInformation.html))
- `~/device/info` ([mavros_msgs::msg::GimbalDeviceInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalDeviceInformation.html))


## Subscribers
- `~/device/set_attitude` ([mavros_msgs::msg::GimbalDeviceSetAttitude](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalDeviceSetAttitude.html))
- `~/manager/set_attitude` ([mavros_msgs::msg::GimbalManagerSetAttitude](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerSetAttitude.html))
- `~/manager/set_pitchyaw` ([mavros_msgs::msg::GimbalManagerSetPitchyaw](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerSetPitchyaw.html))
- `~/manager/set_manual_control` ([mavros_msgs::msg::GimbalManagerSetPitchyaw](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerSetPitchyaw.html))


## Services
- `~/device/get_info` ([mavros_msgs::srv::GimbalGetInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalGetInformation.html))
- `~/manager/get_info` ([mavros_msgs::srv::GimbalGetInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalGetInformation.html))
- `~/manager/configure` ([mavros_msgs::srv::GimbalManagerConfigure](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerConfigure.html))
- `~/manager/pitchyaw` ([mavros_msgs::srv::GimbalManagerPitchyaw](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerPitchyaw.html))
- `~/manager/set_roi` ([mavros_msgs::srv::GimbalManagerSetRoi](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerSetRoi.html))
- `~/manager/camera_track` ([mavros_msgs::srv::GimbalManagerCameraTrack](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerCameraTrack.html))


## Clients
- `cmd/command` ([mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html))


## Parameters
- `frame_id` [default: `"base_link_frd"`]
- `tf.send` [type: bool, default: `false`] - Important tf subsection Report the transform from base_link to gimbal here.
- `tf.frame_id` [default: `"base_link_frd"`]


## MAVLink Subscriptions
- [`GIMBAL_DEVICE_ATTITUDE_STATUS`](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS) [handler: handle_gimbal_attitude_status, dialect: common, msg_id: 285, id: `mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS::MSG_ID`]
- [`GIMBAL_MANAGER_STATUS`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS) [handler: handle_manager_status, dialect: common, msg_id: 281, id: `mavlink::common::msg::GIMBAL_MANAGER_STATUS::MSG_ID`]
- [`GIMBAL_DEVICE_INFORMATION`](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION) [handler: handle_device_information, dialect: common, msg_id: 283, id: `mavlink::common::msg::GIMBAL_DEVICE_INFORMATION::MSG_ID`]
- [`GIMBAL_MANAGER_INFORMATION`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION) [handler: handle_manager_information, dialect: common, msg_id: 280, id: `mavlink::common::msg::GIMBAL_MANAGER_INFORMATION::MSG_ID`]


## MAVLink Publications
- [`GIMBAL_DEVICE_SET_ATTITUDE`](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE) [arg: `msg`, dialect: common, msg_id: 284, id: `mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE::MSG_ID`]
- [`GIMBAL_MANAGER_SET_ATTITUDE`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE) [arg: `msg`, dialect: common, msg_id: 282, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE::MSG_ID`]
- [`GIMBAL_MANAGER_SET_PITCHYAW`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW) [arg: `msg`, dialect: common, msg_id: 287, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW::MSG_ID`]
- [`GIMBAL_MANAGER_SET_PITCHYAW`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW) [arg: `msg`, dialect: common, msg_id: 287, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW::MSG_ID`]
