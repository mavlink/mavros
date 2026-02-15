# imu

- File: `mavros/src/plugins/imu.cpp`
- Class: `mavros::std_plugins::IMUPlugin`
- Namespace: `imu`
- Brief: IMU and attitude data publication plugin


## Publishers
- `~/data` (sensor_msgs::msg::Imu)
- `~/data_raw` (sensor_msgs::msg::Imu)
- `~/mag` (sensor_msgs::msg::MagneticField)
- `~/temperature_imu` (sensor_msgs::msg::Temperature)
- `~/temperature_baro` (sensor_msgs::msg::Temperature)
- `~/static_pressure` (sensor_msgs::msg::FluidPressure)
- `~/diff_pressure` (sensor_msgs::msg::FluidPressure)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"base_link"`] - Additionally, it is reported the orientation of the vehicle to describe the transformation from the ENU frame to the base_link frame (ENU <-> base_link). THIS ORIENTATION IS NOT THE SAME AS THAT REPORTED BY THE FCU (NED <-> aircraft).
- `linear_acceleration_stdev` [type: double, default: `0.0003`]
- `angular_velocity_stdev` [default: `0.02 * (M_PI / 180.0)`]
- `orientation_stdev` [type: double, default: `1.0`]
- `magnetic_stdev` [type: double, default: `0.0`]


## MAVLink Subscriptions
- [`ATTITUDE`](https://mavlink.io/en/messages/common.html#ATTITUDE) [handler: handle_attitude, dialect: common, msg_id: 30, id: `mavlink::common::msg::ATTITUDE::MSG_ID`]
- [`ATTITUDE_QUATERNION`](https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION) [handler: handle_attitude_quaternion, dialect: common, msg_id: 31, id: `mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID`]
- [`HIGHRES_IMU`](https://mavlink.io/en/messages/common.html#HIGHRES_IMU) [handler: handle_highres_imu, dialect: common, msg_id: 105, id: `mavlink::common::msg::HIGHRES_IMU::MSG_ID`]
- [`RAW_IMU`](https://mavlink.io/en/messages/common.html#RAW_IMU) [handler: handle_raw_imu, dialect: common, msg_id: 27, id: `mavlink::common::msg::RAW_IMU::MSG_ID`]
- [`SCALED_IMU`](https://mavlink.io/en/messages/common.html#SCALED_IMU) [handler: handle_scaled_imu, dialect: common, msg_id: 26, id: `mavlink::common::msg::SCALED_IMU::MSG_ID`]
- [`SCALED_PRESSURE`](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE) [handler: handle_scaled_pressure, dialect: common, msg_id: 29, id: `mavlink::common::msg::SCALED_PRESSURE::MSG_ID`]


## MAVLink Publications
- None
