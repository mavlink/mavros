# vision_speed

- File: `mavros_extras/src/plugins/vision_speed_estimate.cpp`
- Class: `mavros::extra_plugins::VisionSpeedEstimatePlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Vision speed estimate plugin


Send velocity estimation from various vision estimators
to FCU position and attitude estimators.

## Publishers
- None

## Subscribers
- `~/speed_twist_cov` [type: [geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to vision speed as TwistWithCovarianceStamped.
- `~/speed_twist` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to vision speed as TwistStamped.
- `~/speed_vector` [type: [geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to vision speed as Vector3Stamped.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`VISION_SPEED_ESTIMATE`](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE) [arg: `msg`, dialect: common, msg_id: 103, id: `mavlink::common::msg::VISION_SPEED_ESTIMATE::MSG_ID`]
