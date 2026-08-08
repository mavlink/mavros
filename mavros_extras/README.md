mavros extras
=============

Some extra plugins and nodes for [mavros][mr].

> The full ROS API of every plugin (topics, services, parameters) is documented
> in the [plugin reference](../docs/plugins/index.md) and the
> [generated plugin pages](../docs/plugins/extras/).

This package provides the optional plugins:

- `3dr_radio`, `adsb`, `camera`, `cam_imu_sync`, `cellular_status`,
  `companion_process_status`, `debug_value`, `distance_sensor`, `esc_status`,
  `esc_telemetry`, `fake_gps`, `gimbal_control`, `gps_input`, `gps_rtk`,
  `gps_status`, `guided_target`, `hil`, `landing_target`, `log_transfer`,
  `mag_calibration_status`, `mocap_pose_estimate`, `mount_control`,
  `obstacle_distance`, `obstacle_distance_3d`, `odom`,
  `onboard_computer_status`, `open_drone_id`, `optical_flow`, `play_tune`,
  `px4flow`, `rangefinder`, `sim_state`, `terrain`, `trajectory`, `tunnel`,
  `vfr_hud`, `vibration`, `vision_pose_estimate`, `vision_speed_estimate`,
  `wheel_odometry`.

See the [plugin reference](../docs/plugins/index.md) for how to load and
configure each plugin.


[mr]: https://github.com/mavlink/mavros