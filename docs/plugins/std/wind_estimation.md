# wind_estimation

- File: `mavros/src/plugins/wind_estimation.cpp`
- Class: `mavros::std_plugins::WindEstimationPlugin`
- Namespace: `wind`
- Brief: Wind estimation plugin.


## Publishers
- `wind_estimation` (geometry_msgs::msg::TwistWithCovarianceStamped)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `WIND` [handler: handle_apm_wind, dialect: ardupilotmega, msg_id: 168, id: `mavlink::ardupilotmega::msg::WIND::MSG_ID`]
- `WIND_COV` [handler: handle_px4_wind, dialect: common, msg_id: 231, id: `mavlink::common::msg::WIND_COV::MSG_ID`]


## MAVLink Publications
- None
