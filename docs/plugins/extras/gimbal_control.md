# gimbal_control

- File: `mavros_extras/src/plugins/gimbal_control.cpp`
- Class: `mavros::extra_plugins::GimbalControlPlugin`
- Namespace: `gimbal_control`
- Brief: Gimbal Control Plugin


Adds support for Mavlink Gimbal Protocol v2. Also publishes gimbal pose to TF when parameter tf_send==true

## Publishers
- `~/device/attitude_status` (mavros_msgs::msg::GimbalDeviceAttitudeStatus)
- `~/manager/status` (mavros_msgs::msg::GimbalManagerStatus)
- `~/manager/info` (mavros_msgs::msg::GimbalManagerInformation)
- `~/device/info` (mavros_msgs::msg::GimbalDeviceInformation)


## Subscribers
- `~/device/set_attitude` (mavros_msgs::msg::GimbalDeviceSetAttitude)
- `~/manager/set_attitude` (mavros_msgs::msg::GimbalManagerSetAttitude)
- `~/manager/set_pitchyaw` (mavros_msgs::msg::GimbalManagerSetPitchyaw)
- `~/manager/set_manual_control` (mavros_msgs::msg::GimbalManagerSetPitchyaw)


## Services
- `~/device/get_info` (mavros_msgs::srv::GimbalGetInformation)
- `~/manager/get_info` (mavros_msgs::srv::GimbalGetInformation)
- `~/manager/configure` (mavros_msgs::srv::GimbalManagerConfigure)
- `~/manager/pitchyaw` (mavros_msgs::srv::GimbalManagerPitchyaw)
- `~/manager/set_roi` (mavros_msgs::srv::GimbalManagerSetRoi)
- `~/manager/camera_track` (mavros_msgs::srv::GimbalManagerCameraTrack)


## Clients
- `cmd/command` (mavros_msgs::srv::CommandLong)


## Parameters
- `frame_id` [default: `"base_link_frd"`]
- `tf.send` [type: bool, default: `false`] - Important tf subsection Report the transform from base_link to gimbal here.
- `tf.frame_id` [default: `"base_link_frd"`]


## MAVLink Subscriptions
- `GIMBAL_DEVICE_ATTITUDE_STATUS` [handler: handle_gimbal_attitude_status, dialect: common, msg_id: 285, id: `mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS::MSG_ID`]
- `GIMBAL_MANAGER_STATUS` [handler: handle_manager_status, dialect: common, msg_id: 281, id: `mavlink::common::msg::GIMBAL_MANAGER_STATUS::MSG_ID`]
- `GIMBAL_DEVICE_INFORMATION` [handler: handle_device_information, dialect: common, msg_id: 283, id: `mavlink::common::msg::GIMBAL_DEVICE_INFORMATION::MSG_ID`]
- `GIMBAL_MANAGER_INFORMATION` [handler: handle_manager_information, dialect: common, msg_id: 280, id: `mavlink::common::msg::GIMBAL_MANAGER_INFORMATION::MSG_ID`]


## MAVLink Publications
- `GIMBAL_DEVICE_SET_ATTITUDE` [arg: `msg`, dialect: common, msg_id: 284, id: `mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE::MSG_ID`]
- `GIMBAL_MANAGER_SET_ATTITUDE` [arg: `msg`, dialect: common, msg_id: 282, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE::MSG_ID`]
- `GIMBAL_MANAGER_SET_PITCHYAW` [arg: `msg`, dialect: common, msg_id: 287, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW::MSG_ID`]
- `GIMBAL_MANAGER_SET_PITCHYAW` [arg: `msg`, dialect: common, msg_id: 287, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW::MSG_ID`]
