# mocap_pose_estimate

- File: `mavros_extras/src/plugins/mocap_pose_estimate.cpp`
- Class: `mavros::extra_plugins::MocapPoseEstimatePlugin`
- Namespace: `mocap`
- Brief: MocapPoseEstimate plugin


Sends motion capture data to FCU.

## Publishers
- None


## Subscribers
- `~/tf` (geometry_msgs::msg::TransformStamped)
- `~/pose` (geometry_msgs::msg::PoseStamped)


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- `ATT_POS_MOCAP` [arg: `pos`, dialect: common, msg_id: 138, id: `mavlink::common::msg::ATT_POS_MOCAP::MSG_ID`]
