# QoS profiles

This page lists every QoS profile used by the MAVROS plugins. Standard `rclcpp::*` profiles link to the rclcpp API docs.

## Inline

| Id | Config | Topics |
|----|--------|--------|
| `qos_1_` | `QoS(1)` | `cellular_status`, `gps_input`, `gps_rtk`, `mocap_pose_estimate`, `odometry`, `open_drone_id`, `optical_flow`, `play_tune`, `px4flow` |

| `qos_10_` | `QoS(10)` | `adsb`, `cam_imu_sync`, `camera`, `companion_process_status`, `debug_value`, `esc_status`, `esc_telemetry`, `fake_gps`, `gimbal_control`, `gps_rtk`, `gps_status`, `guided_target`, `hil`, `home_position`, `landing_target`, `manual_control`, `mount_control`, `nav_controller_output`, `obstacle_distance_3d`, `obstacle_distance`, `odometry`, `onboard_computer_status`, `optical_flow`, `px4flow`, `rangefinder`, `rc_io`, `sim_state`, `terrain`, `trajectory`, `tunnel`, `vfr_hud`, `vibration`, `vision_pose`, `vision_speed`, `wheel_odometry` |

| `qos_1000_` | `QoS(1000)` | `log_transfer` |

| `qos_2_` | `QoS(2)` | `mag_calibration_status` |

| `qos_64_` | `QoS(64)` | `terrain` |

| `home_position-state_qos` | `rclcpp::QoS(10).transient_local()` | `home_position`, `sys_status` |

## Named

| Id | Profile | Topics |
|----|---------|--------|
| `latchedstateqos` | `LatchedStateQoS` | `geofence`, `global_position`, `guided_target`, `rallypoint`, `waypoint` |

| `parametereventsqos` | [`ParameterEventsQoS`](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1ParameterEventsQoS.html) | `param` |

| `parametersqos` | [`ParametersQoS`](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1ParametersQoS.html) | `param` |

| `sensordataqos` | [`SensorDataQoS`](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1SensorDataQoS.html) | `actuator_control`, `altitude`, `distance_sensor`, `global_position`, `imu`, `landing_target`, `local_position`, `setpoint_accel`, `setpoint_position`, `setpoint_raw`, `setpoint_trajectory`, `setpoint_velocity`, `sys_status`, `sys_time`, `tdr_radio`, `wind_estimation` |

| `servicesqos` | [`ServicesQoS`](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1ServicesQoS.html) | `command`, `geofence`, `rallypoint`, `sys_status`, `waypoint` |
