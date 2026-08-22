# QoS profiles

This page lists every distinct QoS profile used by the MAVROS plugins.

!!! tip "See also"
    [ROS 2 QoS documentation](https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

Standard `rclcpp::*` profiles link to the rclcpp API docs.



## Named


### LatchedStateQoS {#latchedstateqos}


| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 1 | Reliable | Transient local | Default | Default | System default |

Used by: *std*: [`geofence`](std/geofence.md), [`global_position`](std/global_position.md), [`rallypoint`](std/rallypoint.md), [`waypoint`](std/waypoint.md); *extras*: [`guided_target`](extras/guided_target.md)


### ParameterEventsQoS {#parametereventsqos}


See [rclcpp::ParameterEventsQoS](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1ParameterEventsQoS.html).


| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 1000 | Reliable | Volatile | Default | Default | System default |

Used by: *std*: [`param`](std/param.md)


### ParametersQoS {#parametersqos}


See [rclcpp::ParametersQoS](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1ParametersQoS.html).


| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 1000 | Reliable | Volatile | Default | Default | System default |

Used by: *std*: [`param`](std/param.md)


### SensorDataQoS {#sensordataqos}


See [rclcpp::SensorDataQoS](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1SensorDataQoS.html).


| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 5 | Best effort | Volatile | Default | Default | System default |

Used by: *std*: [`actuator_control`](std/actuator_control.md), [`altitude`](std/altitude.md), [`global_position`](std/global_position.md), [`imu`](std/imu.md), [`local_position`](std/local_position.md), [`setpoint_accel`](std/setpoint_accel.md), [`setpoint_position`](std/setpoint_position.md), [`setpoint_raw`](std/setpoint_raw.md), [`setpoint_trajectory`](std/setpoint_trajectory.md), [`setpoint_velocity`](std/setpoint_velocity.md), [`sys_status`](std/sys_status.md), [`sys_time`](std/sys_time.md), [`wind_estimation`](std/wind_estimation.md); *extras*: [`distance_sensor`](extras/distance_sensor.md), [`landing_target`](extras/landing_target.md), [`tdr_radio`](extras/3dr_radio.md)


### ServicesQoS {#servicesqos}


See [rclcpp::ServicesQoS](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1ServicesQoS.html).


| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 10 | Reliable | Volatile | Default | Default | System default |

Used by: *std*: [`command`](std/command.md), [`geofence`](std/geofence.md), [`rallypoint`](std/rallypoint.md), [`sys_status`](std/sys_status.md), [`waypoint`](std/waypoint.md)


### StateQoS {#stateqos}


| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 10 | Reliable | Transient local | Default | Default | System default |

Used by: *std*: [`home_position`](std/home_position.md), [`sys_status`](std/sys_status.md)



## Inline


### QoS(1) {#qos_1_}

| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 1 | Reliable | Volatile | Default | Default | System default |

Used by: *extras*: [`cellular_status`](extras/cellular_status.md), [`gps_input`](extras/gps_input.md), [`gps_rtk`](extras/gps_rtk.md), [`mocap_pose_estimate`](extras/mocap_pose_estimate.md), [`odometry`](extras/odom.md), [`open_drone_id`](extras/open_drone_id.md), [`optical_flow`](extras/optical_flow.md), [`play_tune`](extras/play_tune.md), [`px4flow`](extras/px4flow.md)


### QoS(10) {#qos_10_}

| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 10 | Reliable | Volatile | Default | Default | System default |

Used by: *std*: [`home_position`](std/home_position.md), [`manual_control`](std/manual_control.md), [`nav_controller_output`](std/nav_controller_output.md), [`rc_io`](std/rc_io.md); *extras*: [`adsb`](extras/adsb.md), [`cam_imu_sync`](extras/cam_imu_sync.md), [`camera`](extras/camera.md), [`companion_process_status`](extras/companion_process_status.md), [`debug_value`](extras/debug_value.md), [`esc_status`](extras/esc_status.md), [`esc_telemetry`](extras/esc_telemetry.md), [`fake_gps`](extras/fake_gps.md), [`gimbal_control`](extras/gimbal_control.md), [`gps_rtk`](extras/gps_rtk.md), [`gps_status`](extras/gps_status.md), [`guided_target`](extras/guided_target.md), [`hil`](extras/hil.md), [`landing_target`](extras/landing_target.md), [`mount_control`](extras/mount_control.md), [`obstacle_distance`](extras/obstacle_distance.md), [`obstacle_distance_3d`](extras/obstacle_distance_3d.md), [`odometry`](extras/odom.md), [`onboard_computer_status`](extras/onboard_computer_status.md), [`optical_flow`](extras/optical_flow.md), [`px4flow`](extras/px4flow.md), [`rangefinder`](extras/rangefinder.md), [`sim_state`](extras/sim_state.md), [`terrain`](extras/terrain.md), [`trajectory`](extras/trajectory.md), [`tunnel`](extras/tunnel.md), [`vfr_hud`](extras/vfr_hud.md), [`vibration`](extras/vibration.md), [`vision_pose`](extras/vision_pose_estimate.md), [`vision_speed`](extras/vision_speed_estimate.md), [`wheel_odometry`](extras/wheel_odometry.md)


### QoS(1000) {#qos_1000_}

| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 1000 | Reliable | Volatile | Default | Default | System default |

Used by: *extras*: [`log_transfer`](extras/log_transfer.md)


### QoS(2) {#qos_2_}

| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 2 | Reliable | Volatile | Default | Default | System default |

Used by: *extras*: [`mag_calibration_status`](extras/mag_calibration_status.md)


### QoS(64) {#qos_64_}

| History | Depth | Reliability | Durability | Deadline | Lifespan | Liveliness |
|---|---|---|---|---|---|---|
| Keep last | 64 | Reliable | Volatile | Default | Default | System default |

Used by: *extras*: [`terrain`](extras/terrain.md)
