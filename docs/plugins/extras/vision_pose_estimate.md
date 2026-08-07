# vision_pose

- File: `mavros_extras/src/plugins/vision_pose_estimate.cpp`
- Class: `mavros::extra_plugins::VisionPoseEstimatePlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Vision pose estimate plugin


Send pose estimation from various vision estimators
to FCU position and attitude estimators.



## Publishers
- None

## Subscribers
- `~/pose` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to vision pose as PoseStamped.
- `~/pose_cov` [type: [geometry_msgs::msg::PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to vision pose with covariance as PoseWithCovarianceStamped.

## Services
- None

## Clients
- None


## Parameters
- `tf/listen` [type: bool, default: `false`] - tf params Listen to vision pose from TF (else subscribe to ~/pose).
- `tf/frame_id` [type: string, default: `"map"`] - TF frame id for vision pose source.
- `tf/child_frame_id` [type: string, default: `"vision_estimate"`] - TF child frame id for vision pose source.
- `tf/rate_limit` [type: double, default: `10.0`] - Vision pose send rate limit [Hz].


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`VISION_POSITION_ESTIMATE`](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) [arg: `msg`, dialect: common, msg_id: 102, id: `mavlink::common::msg::VISION_POSITION_ESTIMATE::MSG_ID`]
