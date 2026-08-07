# sim_state

- File: `mavros_extras/src/plugins/sim_state.cpp`
- Class: `mavros::extra_plugins::SimStatePlugin`
- Namespace: `mavros::extra_plugins`
- Brief: SIM_STATE plugin.


Adds support for MAVLink SIM_STATE (id 108) messages and republishes fields to ROS 2 topics.
Intended for simulation use as a high-accuracy ground-truth feed when developing autonomy.
Currently verified with ArduCopter SITL.

## Publishers
- `~/attitude` [type: [sensor_msgs::msg::Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish IMU attitude (orientation + angular velocity) in ENU/base_link from MAVLink SIM_STATE.
- `~/acceleration` [type: [geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish linear acceleration (m/s^2) in ENU/map from MAVLink SIM_STATE.
- `~/velocity_body` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish body-frame velocity (linear + angular) in base_link from MAVLink SIM_STATE.
- `~/velocity_local` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish local-frame velocity (linear + angular) in ENU/map from MAVLink SIM_STATE.
- `~/global_position` [type: [sensor_msgs::msg::NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish global position (WGS84 NavSatFix) from MAVLink SIM_STATE.

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`SIM_STATE`](https://mavlink.io/en/messages/common.html#SIM_STATE) [handler: handle_sim_state, dialect: common, msg_id: 108, id: `mavlink::common::msg::SIM_STATE::MSG_ID`]


## MAVLink Publications
- None
