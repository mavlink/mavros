# vision_speed

- File: `mavros_extras/src/plugins/vision_speed_estimate.cpp`
- Class: `mavros::extra_plugins::VisionSpeedEstimatePlugin`
- Namespace: `vision_speed`
- Brief: Vision speed estimate plugin


Send velocity estimation from various vision estimators to FCU position and attitude estimators.

## Publishers
- None


## Subscribers
- `~/speed_twist_cov` ([geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html))
- `~/speed_twist` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html))
- `~/speed_vector` ([geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html))


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`VISION_SPEED_ESTIMATE`](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE) [arg: `vs`, dialect: common, msg_id: 103, id: `mavlink::common::msg::VISION_SPEED_ESTIMATE::MSG_ID`]
