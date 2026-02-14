# sim_state

- File: `mavros_extras/src/plugins/sim_state.cpp`
- Class: `mavros::extra_plugins::SimStatePlugin`
- Namespace: `sim_state`
- Brief: SIM_STATE plugin.


Adds support for MAVLink SIM_STATE (id 108) messages and republishes fields to ROS 2 topics. Intended for simulation use as a high-accuracy ground-truth feed when developing autonomy. Currently verified with ArduCopter SITL. Published topics (relative to plugin namespace): - ~/attitude (sensor_msgs/Imu): orientation (ENU/base_link) and angular velocity - ~/acceleration (geometry_msgs/Vector3Stamped): linear accel (m/s^2) in ENU/map - ~/velocity_body (geometry_msgs/TwistStamped): linear+angular velocity in base_link - ~/velocity_local (geometry_msgs/TwistStamped): linear+angular velocity in ENU/map - ~/global_position (sensor_msgs/NavSatFix): WGS84 lat/lon/alt with optional covariance

## Publishers
- `~/attitude` (sensor_msgs::msg::Imu) - IMU attitude publisher (~/attitude): orientation and angular velocity in ENU/base_link
- `~/acceleration` (geometry_msgs::msg::Vector3Stamped) - Linear acceleration publisher (~/acceleration): ENU/map, units m/s^2
- `~/velocity_body` (geometry_msgs::msg::TwistStamped) - Body-frame twist publisher (~/velocity_body): base_link linear+angular velocity
- `~/velocity_local` (geometry_msgs::msg::TwistStamped) - Local-frame twist publisher (~/velocity_local): ENU/map linear+angular velocity
- `~/global_position` (sensor_msgs::msg::NavSatFix) - Global position publisher (~/global_position): WGS84 NavSatFix


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `SIM_STATE` [handler: handle_sim_state, dialect: common, msg_id: 108, id: `mavlink::common::msg::SIM_STATE::MSG_ID`]


## MAVLink Publications
- None
