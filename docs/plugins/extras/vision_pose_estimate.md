# vision_pose

- File: `mavros_extras/src/plugins/vision_pose_estimate.cpp`
- Class: `mavros::extra_plugins::VisionPoseEstimatePlugin`
- Namespace: `vision_pose`
- Brief: Vision pose estimate plugin


Send pose estimation from various vision estimators to FCU position and attitude estimators.

## Publishers
- None


## Subscribers
- `~/pose` (geometry_msgs::msg::PoseStamped)
- `~/pose_cov` (geometry_msgs::msg::PoseWithCovarianceStamped)


## Services
- None


## Clients
- None


## Parameters
- `tf/listen` [type: bool, default: `false`] - tf params
- `tf/frame_id` [default: `"map"`]
- `tf/child_frame_id` [default: `"vision_estimate"`]
- `tf/rate_limit` [type: double, default: `10.0`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- `VISION_POSITION_ESTIMATE` [arg: `vp`, dialect: common, msg_id: 102, id: `mavlink::common::msg::VISION_POSITION_ESTIMATE::MSG_ID`]
