# MAVROS plugins

MAVROS is split into the core plugins shipped in `mavros` and the optional
plugins shipped in `mavros_extras`. Each plugin page documents its ROS API
(publishers, subscribers, services, clients, parameters) and the MAVLink
messages it subscribes to and publishes.

## Standard plugins (`mavros`)

| Plugin | Brief | Pub | Sub | Srv | Client | MAVLink sub/pub |
|--------|-------|-----|-----|-----|--------|-----------------|
| [`actuator_control`](std/actuator_control.md) | ActuatorControl plugin | 1 | 1 | — | — | 1/1 |
| [`altitude`](std/altitude.md) | Altitude plugin. | 1 | — | — | — | 1/0 |
| [`command`](std/command.md) | Command plugin. | — | — | 11 | — | 1/2 |
| [`dummy`](std/dummy.md) | Dummy plugin. | — | — | — | — | 4/0 |
| [`ftp`](std/ftp.md) | FTP plugin. | — | — | 12 | — | 1/1 |
| [`geofence`](std/geofence.md) | Geofence manipulation plugin | 1 | — | 3 | — | 6/10 |
| [`global_position`](std/global_position.md) | Global position plugin. | 9 | 2 | — | — | 4/1 |
| [`home_position`](std/home_position.md) | home position plugin. | 1 | 1 | 1 | 1 | 1/1 |
| [`imu`](std/imu.md) | IMU and attitude data publication plugin | 7 | — | — | — | 6/0 |
| [`local_position`](std/local_position.md) | Local position plugin. | 7 | — | — | — | 2/0 |
| [`manual_control`](std/manual_control.md) | Manual Control plugin | 1 | 1 | — | — | 1/1 |
| [`nav_controller_output`](std/nav_controller_output.md) | nav controller output plugin. | 1 | — | — | — | 1/0 |
| [`param`](std/param.md) | Parameter manipulation plugin | 2 | — | 8 | — | 1/3 |
| [`rallypoint`](std/rallypoint.md) | Rallypoint manipulation plugin | 1 | — | 3 | — | 6/10 |
| [`rc_io`](std/rc_io.md) | RC IO plugin | 2 | 1 | — | — | 3/1 |
| [`setpoint_accel`](std/setpoint_accel.md) | Setpoint acceleration/force plugin | — | 1 | — | — | 0/1 |
| [`setpoint_attitude`](std/setpoint_attitude.md) | Setpoint attitude plugin | — | 3 | — | — | 0/1 |
| [`setpoint_position`](std/setpoint_position.md) | Setpoint position plugin | — | 5 | — | — | 0/2 |
| [`setpoint_raw`](std/setpoint_raw.md) | Setpoint RAW plugin | 3 | 3 | — | — | 3/3 |
| [`setpoint_trajectory`](std/setpoint_trajectory.md) | Setpoint TRAJECTORY plugin | 1 | 1 | 1 | — | 0/1 |
| [`setpoint_velocity`](std/setpoint_velocity.md) | Setpoint velocity plugin | — | 2 | — | — | 0/1 |
| [`sys_status`](std/sys_status.md) | System status plugin. | 7 | 1 | 4 | 2 | 10/4 |
| [`sys_time`](std/sys_time.md) | System time plugin | 2 | — | — | — | 2/2 |
| [`waypoint`](std/waypoint.md) | Mission manipulation plugin | 2 | — | 4 | — | 8/10 |
| [`wind_estimation`](std/wind_estimation.md) | Wind estimation plugin. | 1 | — | — | — | 2/0 |

## Extra plugins (`mavros_extras`)

| Plugin | Brief | Pub | Sub | Srv | Client | MAVLink sub/pub |
|--------|-------|-----|-----|-----|--------|-----------------|
| [`adsb`](extras/adsb.md) | ADS-B Vehicle plugin | 1 | 1 | — | — | 1/1 |
| [`cam_imu_sync`](extras/cam_imu_sync.md) | Camera IMU synchronisation plugin | 1 | — | — | — | 1/0 |
| [`camera`](extras/camera.md) | Camera plugin plugin | 1 | — | — | — | 1/0 |
| [`cellular_status`](extras/cellular_status.md) | Cellular status plugin. | — | 1 | — | — | 0/1 |
| [`companion_process_status`](extras/companion_process_status.md) | Obstacle companion process status plugin | — | 1 | — | — | 0/1 |
| [`debug_value`](extras/debug_value.md) | Plugin for Debug msgs from MAVLink API | 5 | 1 | — | — | 5/5 |
| [`distance_sensor`](extras/distance_sensor.md) | Distance sensor plugin | 1 | 1 | — | — | 1/1 |
| [`esc_status`](extras/esc_status.md) | ESC status plugin | 2 | — | — | — | 2/0 |
| [`esc_telemetry`](extras/esc_telemetry.md) | ESC telemetry plugin | 1 | — | — | — | 3/0 |
| [`fake_gps`](extras/fake_gps.md) | Fake GPS plugin. | — | 4 | — | — | 0/2 |
| [`gimbal_control`](extras/gimbal_control.md) | Gimbal Control Plugin | 4 | 4 | 6 | 1 | 4/3 |
| [`gps_input`](extras/gps_input.md) | GPS_INPUT GPS plugin. | — | 1 | — | — | 0/1 |
| [`gps_rtk`](extras/gps_rtk.md) | GPS RTK plugin | 1 | 1 | — | — | 1/1 |
| [`gps_status`](extras/gps_status.md) | Mavlink GPS status plugin. | 4 | — | — | — | 4/0 |
| [`guided_target`](extras/guided_target.md) | guided target plugin | 1 | 1 | — | — | 1/0 |
| [`hil`](extras/hil.md) | Hil plugin | 2 | 5 | — | — | 2/5 |
| [`landing_target`](extras/landing_target.md) | Landing Target plugin | 2 | 2 | — | — | 1/1 |
| [`log_transfer`](extras/log_transfer.md) | Log Transfer plugin | 2 | — | 4 | — | 2/4 |
| [`mag_calibration_status`](extras/mag_calibration_status.md) | MagCalStatus plugin. | 2 | — | — | — | 2/0 |
| [`mocap_pose_estimate`](extras/mocap_pose_estimate.md) | MocapPoseEstimate plugin | — | 2 | — | — | 0/1 |
| [`mount_control`](extras/mount_control.md) | Mount Control plugin | 2 | 1 | 1 | 1 | 2/1 |
| [`obstacle_distance`](extras/obstacle_distance.md) | Obstacle distance plugin | — | 1 | — | — | 0/1 |
| [`obstacle_distance_3d`](extras/obstacle_distance_3d.md) | Plugin to handle sending OBSTACLE_DISTANCE_3D MAVLink messages. | — | 1 | — | — | 0/1 |
| [`odometry`](extras/odom.md) | Odometry plugin | 1 | 1 | — | — | 1/1 |
| [`onboard_computer_status`](extras/onboard_computer_status.md) | Onboard Computer Status plugin | — | 1 | — | — | 0/1 |
| [`open_drone_id`](extras/open_drone_id.md) | Open Drone ID plugin | — | 5 | — | — | 0/5 |
| [`optical_flow`](extras/optical_flow.md) | Optical Flow custom plugin | 2 | 1 | — | — | 1/1 |
| [`play_tune`](extras/play_tune.md) | Play Tune service | — | 1 | — | — | 0/1 |
| [`px4flow`](extras/px4flow.md) | PX4 Optical Flow plugin | 3 | 1 | — | — | 1/1 |
| [`rangefinder`](extras/rangefinder.md) | Ardupilot Rangefinder plugin. | 1 | — | — | — | 1/0 |
| [`sim_state`](extras/sim_state.md) | SIM_STATE plugin. | 5 | — | — | — | 1/0 |
| [`tdr_radio`](extras/3dr_radio.md) | 3DR Radio plugin. | 1 | — | — | — | 2/0 |
| [`terrain`](extras/terrain.md) | Terrain plugin. | 2 | 1 | — | 1 | 3/2 |
| [`trajectory`](extras/trajectory.md) | Trajectory plugin to receive planned path from the FCU and | 1 | 2 | — | — | 1/2 |
| [`tunnel`](extras/tunnel.md) | Tunnel plugin | 1 | 1 | — | — | 1/1 |
| [`vfr_hud`](extras/vfr_hud.md) | VFR HUD plugin. | 1 | — | — | — | 1/0 |
| [`vibration`](extras/vibration.md) | Vibration plugin | 1 | — | — | — | 1/0 |
| [`vision_pose`](extras/vision_pose_estimate.md) | Vision pose estimate plugin | — | 2 | — | — | 0/1 |
| [`vision_speed`](extras/vision_speed_estimate.md) | Vision speed estimate plugin | — | 3 | — | — | 0/1 |
| [`wheel_odometry`](extras/wheel_odometry.md) | Wheel odometry plugin. | 4 | — | — | — | 2/0 |
