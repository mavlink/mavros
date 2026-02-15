# mocap_pose_estimate

- File: `mavros_extras/src/plugins/mocap_pose_estimate.cpp`
- Class: `mavros::extra_plugins::MocapPoseEstimatePlugin`
- Namespace: `mocap`
- Brief: MocapPoseEstimate plugin


Sends motion capture data to FCU.

## Publishers
- None


## Subscribers
- `~/tf` ([geometry_msgs::msg::TransformStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TransformStamped.html))
- `~/pose` ([geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html))


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`ATT_POS_MOCAP`](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) [arg: `pos`, dialect: common, msg_id: 138, id: `mavlink::common::msg::ATT_POS_MOCAP::MSG_ID`]
