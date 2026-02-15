# wind_estimation

- File: `mavros/src/plugins/wind_estimation.cpp`
- Class: `mavros::std_plugins::WindEstimationPlugin`
- Namespace: `wind`
- Brief: Wind estimation plugin.


## Publishers
- `wind_estimation` ([geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html))


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`WIND`](https://mavlink.io/en/messages/ardupilotmega.html#WIND) [handler: handle_apm_wind, dialect: ardupilotmega, msg_id: 168, id: `mavlink::ardupilotmega::msg::WIND::MSG_ID`]
- [`WIND_COV`](https://mavlink.io/en/messages/common.html#WIND_COV) [handler: handle_px4_wind, dialect: common, msg_id: 231, id: `mavlink::common::msg::WIND_COV::MSG_ID`]


## MAVLink Publications
- None
