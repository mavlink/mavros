# mocap_pose_estimate

- File: `mavros_extras/src/plugins/mocap_pose_estimate.cpp`
- Class: `mavros::extra_plugins::MocapPoseEstimatePlugin`
- Namespace: `mavros::extra_plugins`
- Brief: MocapPoseEstimate plugin


Sends motion capture data to FCU.

## Publishers
- None

## Subscribers
- `~/tf` [type: [geometry_msgs::msg::TransformStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TransformStamped.html), qos: [QoS(1)](../qos.md#qos_1_)] - Subscribe to motion capture pose as TransformStamped (VICON).
- `~/pose` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [QoS(1)](../qos.md#qos_1_)] - Subscribe to motion capture pose as PoseStamped (Optitrack).

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`ATT_POS_MOCAP`](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) [arg: `msg`, dialect: common, msg_id: 138, id: `mavlink::common::msg::ATT_POS_MOCAP::MSG_ID`]
