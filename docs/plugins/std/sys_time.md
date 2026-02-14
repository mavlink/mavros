# sys_time

- File: `mavros/src/plugins/sys_time.cpp`
- Class: `mavros::std_plugins::SystemTimePlugin`
- Namespace: `time`
- Brief: System time plugin


## Publishers
- `time_reference` (sensor_msgs::msg::TimeReference)
- `timesync_status` (mavros_msgs::msg::TimesyncStatus)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `time_ref_source` [default: `"fcu"`]
- `timesync_mode` [default: `"MAVLINK"`]
- `system_time_rate` [type: double, default: `0.0`]
- `timesync_rate` [type: double, default: `0.0`]
- `timesync_alpha_initial` [type: double, default: `0.05`] - Filter gains Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead to a smoother estimate, but track time drift more slowly, introducing a bias in the estimate. Larger values will cause low-amplitude oscillations. Beta : Used to smooth the clock skew estimate. Smaller values will lead to a tighter estimation of the skew (derivative), but will negatively affect how fast the filter reacts to clock skewing (e.g cause by temperature changes to the oscillator). Larger values will cause large-amplitude oscillations.
- `timesync_beta_initial` [type: double, default: `0.05`]
- `timesync_alpha_final` [type: double, default: `0.003`]
- `timesync_beta_final` [type: double, default: `0.003`]
- `convergence_window` [type: integer, default: `500`] - Filter gain scheduling The filter interpolates between the initial and final gains while the number of exhanged timesync packets is less than convergence_window. A lower value will allow the timesync to converge faster, but with potentially less accurate initial offset and skew estimates.
- `max_rtt_sample` [type: integer, default: `10`] - Outlier rejection and filter reset Samples with round-trip time higher than max_rtt_sample are not used to update the filter. More than max_consecutive_high_rtt number of such events in a row will throw a warning but not reset the filter. Samples whose calculated clock offset is more than max_deviation_sample off from the current estimate are not used to update the filter. More than max_consecutive_high_deviation number of such events in a row will reset the filter. This usually happens only due to a time jump on the remote system.
- `max_deviation_sample` [type: integer, default: `10`]
- `max_consecutive_high_rtt` [type: integer, default: `10`]
- `max_consecutive_high_deviation` [type: integer, default: `10`]


## MAVLink Subscriptions
- `SYSTEM_TIME` [handler: handle_system_time, dialect: common, msg_id: 2, id: `mavlink::common::msg::SYSTEM_TIME::MSG_ID`]
- `TIMESYNC` [handler: handle_timesync, dialect: common, msg_id: 111, id: `mavlink::common::msg::TIMESYNC::MSG_ID`]


## MAVLink Publications
- `SYSTEM_TIME` [arg: `mtime`, dialect: common, msg_id: 2, id: `mavlink::common::msg::SYSTEM_TIME::MSG_ID`]
- `TIMESYNC` [arg: `tsync`, dialect: common, msg_id: 111, id: `mavlink::common::msg::TIMESYNC::MSG_ID`]
