# imu

- File: `mavros/src/plugins/imu.cpp`
- Class: `mavros::std_plugins::IMUPlugin`
- Namespace: `mavros::std_plugins`
- Brief: IMU and attitude data publication plugin


## Publishers
- `~/data` [type: [sensor_msgs::msg::Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish fused IMU data (ATTITUDE / ATTITUDE_QUATERNION).
- `~/data_raw` [type: [sensor_msgs::msg::Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish raw IMU and acceleration data (RAW_IMU / SCALED_IMU / HIGHRES_IMU).
- `~/mag` [type: [sensor_msgs::msg::MagneticField](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/MagneticField.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish filtered magnetic field data (RAW_IMU / SCALED_IMU / HIGHRES_IMU).
- `~/temperature_imu` [type: [sensor_msgs::msg::Temperature](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Temperature.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish IMU temperature (HIGHRES_IMU).
- `~/temperature_baro` [type: [sensor_msgs::msg::Temperature](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Temperature.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish barometer temperature (SCALED_PRESSURE).
- `~/static_pressure` [type: [sensor_msgs::msg::FluidPressure](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/FluidPressure.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish absolute (static) pressure (SCALED_PRESSURE / HIGHRES_IMU).
- `~/diff_pressure` [type: [sensor_msgs::msg::FluidPressure](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/FluidPressure.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish differential pressure (SCALED_PRESSURE / HIGHRES_IMU).

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"base_link"`] - Coordinate frame used for the published IMU topics.
- `linear_acceleration_stdev` [type: double, default: `0.0003`] - Standard deviation of the linear acceleration.
- `angular_velocity_stdev` [default: `0.02 * (M_PI / 180.0)`] - Standard deviation of the angular velocity.
- `orientation_stdev` [type: double, default: `1.0`] - Standard deviation of the orientation.
- `magnetic_stdev` [type: double, default: `0.0`] - Standard deviation of the magnetic field.


## MAVLink Subscriptions
- [`ATTITUDE`](https://mavlink.io/en/messages/common.html#ATTITUDE) [handler: handle_attitude, dialect: common, msg_id: 30, id: `mavlink::common::msg::ATTITUDE::MSG_ID`]
- [`ATTITUDE_QUATERNION`](https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION) [handler: handle_attitude_quaternion, dialect: common, msg_id: 31, id: `mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID`]
- [`HIGHRES_IMU`](https://mavlink.io/en/messages/common.html#HIGHRES_IMU) [handler: handle_highres_imu, dialect: common, msg_id: 105, id: `mavlink::common::msg::HIGHRES_IMU::MSG_ID`]
- [`RAW_IMU`](https://mavlink.io/en/messages/common.html#RAW_IMU) [handler: handle_raw_imu, dialect: common, msg_id: 27, id: `mavlink::common::msg::RAW_IMU::MSG_ID`]
- [`SCALED_IMU`](https://mavlink.io/en/messages/common.html#SCALED_IMU) [handler: handle_scaled_imu, dialect: common, msg_id: 26, id: `mavlink::common::msg::SCALED_IMU::MSG_ID`]
- [`SCALED_PRESSURE`](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE) [handler: handle_scaled_pressure, dialect: common, msg_id: 29, id: `mavlink::common::msg::SCALED_PRESSURE::MSG_ID`]


## MAVLink Publications
- None
