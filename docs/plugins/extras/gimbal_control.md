# gimbal_control

- File: `mavros_extras/src/plugins/gimbal_control.cpp`
- Class: `mavros::extra_plugins::GimbalControlPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Gimbal Control Plugin


Implements the [MAVLink Gimbal Protocol v2](https://mavlink.io/en/services/gimbal_v2.html).
Also publishes gimbal pose to TF when parameter tf_send==true

## Publishers
- `~/device/attitude_status` [type: [mavros_msgs::msg::GimbalDeviceAttitudeStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalDeviceAttitudeStatus.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publishers Publish gimbal device attitude status (GIMBAL_DEVICE_ATTITUDE_STATUS).
- `~/manager/status` [type: [mavros_msgs::msg::GimbalManagerStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerStatus.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish gimbal manager status (GIMBAL_MANAGER_STATUS).
- `~/manager/info` [type: [mavros_msgs::msg::GimbalManagerInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerInformation.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish gimbal manager information (GIMBAL_MANAGER_INFORMATION).
- `~/device/info` [type: [mavros_msgs::msg::GimbalDeviceInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalDeviceInformation.html), qos: [QoS(10)](../qos.md#qos_10_)] - --Not successfully validated-- Publish gimbal device information (GIMBAL_DEVICE_INFORMATION).

## Subscribers
- `~/device/set_attitude` [type: [mavros_msgs::msg::GimbalDeviceSetAttitude](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalDeviceSetAttitude.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribers --Not successfully validated-- Set gimbal device attitude (GIMBAL_DEVICE_SET_ATTITUDE).
- `~/manager/set_attitude` [type: [mavros_msgs::msg::GimbalManagerSetAttitude](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerSetAttitude.html), qos: [QoS(10)](../qos.md#qos_10_)] - --Not successfully validated-- Set gimbal manager attitude (GIMBAL_MANAGER_SET_ATTITUDE).
- `~/manager/set_pitchyaw` [type: [mavros_msgs::msg::GimbalManagerSetPitchyaw](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerSetPitchyaw.html), qos: [QoS(10)](../qos.md#qos_10_)] - --Not successfully validated-- Set gimbal manager pitch/yaw (GIMBAL_MANAGER_SET_PITCHYAW).
- `~/manager/set_manual_control` [type: [mavros_msgs::msg::GimbalManagerSetPitchyaw](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GimbalManagerSetPitchyaw.html), qos: [QoS(10)](../qos.md#qos_10_)] - --Not successfully validated-- also note that the message is the same as pitchyaw and will likely change Set gimbal manager manual control (GIMBAL_MANAGER_SET_PITCHYAW).

## Services
- `~/device/get_info` [type: [mavros_msgs::srv::GimbalGetInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalGetInformation.html)] - Services --Not successfully validated-- Request gimbal device information (GIMBAL_DEVICE_INFORMATION).
- `~/manager/get_info` [type: [mavros_msgs::srv::GimbalGetInformation](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalGetInformation.html)] - Request gimbal manager information (GIMBAL_MANAGER_INFORMATION).
- `~/manager/configure` [type: [mavros_msgs::srv::GimbalManagerConfigure](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerConfigure.html)] - Configure gimbal manager (MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE).
- `~/manager/pitchyaw` [type: [mavros_msgs::srv::GimbalManagerPitchyaw](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerPitchyaw.html)] - Set gimbal manager pitch/yaw (MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW).
- `~/manager/set_roi` [type: [mavros_msgs::srv::GimbalManagerSetRoi](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerSetRoi.html)] - Set gimbal manager region of interest (MAV_CMD_DO_SET_ROI_*).
- `~/manager/camera_track` [type: [mavros_msgs::srv::GimbalManagerCameraTrack](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/GimbalManagerCameraTrack.html)] - --Not successfully validated-- Control gimbal camera tracking (MAV_CMD_CAMERA_TRACK_*).

## Clients
- `cmd/command` [type: [mavros_msgs::srv::CommandLong](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/CommandLong.html), qos: ] - Client to send MAVLink commands (mavros/cmd/command).


## Parameters
- `frame_id` [type: string, default: `"base_link_frd"`] - Frame id used for topic headers.
- `tf.send` [type: bool, default: `false`] - Important tf subsection Report the transform from base_link to gimbal here. Enable publishing gimbal pose to TF.
- `tf.frame_id` [type: string, default: `"base_link_frd"`] - TF frame id for gimbal pose.


## MAVLink Subscriptions
- [`GIMBAL_DEVICE_ATTITUDE_STATUS`](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS) [handler: handle_gimbal_attitude_status, dialect: common, msg_id: 285, id: `mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS::MSG_ID`]
- [`GIMBAL_MANAGER_STATUS`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS) [handler: handle_manager_status, dialect: common, msg_id: 281, id: `mavlink::common::msg::GIMBAL_MANAGER_STATUS::MSG_ID`]
- [`GIMBAL_DEVICE_INFORMATION`](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION) [handler: handle_device_information, dialect: common, msg_id: 283, id: `mavlink::common::msg::GIMBAL_DEVICE_INFORMATION::MSG_ID`]
- [`GIMBAL_MANAGER_INFORMATION`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION) [handler: handle_manager_information, dialect: common, msg_id: 280, id: `mavlink::common::msg::GIMBAL_MANAGER_INFORMATION::MSG_ID`]


## MAVLink Publications
- [`GIMBAL_DEVICE_SET_ATTITUDE`](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE) [arg: `msg`, dialect: common, msg_id: 284, id: `mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE::MSG_ID`]
- [`GIMBAL_MANAGER_SET_ATTITUDE`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE) [arg: `msg`, dialect: common, msg_id: 282, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE::MSG_ID`]
- [`GIMBAL_MANAGER_SET_PITCHYAW`](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW) [arg: `msg`, dialect: common, msg_id: 287, id: `mavlink::common::msg::GIMBAL_MANAGER_SET_PITCHYAW::MSG_ID`]
