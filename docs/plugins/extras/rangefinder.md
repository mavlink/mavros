# rangefinder

- File: `mavros_extras/src/plugins/rangefinder.cpp`
- Class: `mavros::extra_plugins::RangefinderPlugin`
- Namespace: `rangefinder`
- Brief: Ardupilot Rangefinder plugin.


This plugin allows publishing rangefinder sensor data from Ardupilot FCU to ROS.

## Publishers
- `~/rangefinder` (sensor_msgs::msg::Range)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`RANGEFINDER`](https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER) [handler: handle_rangefinder, dialect: ardupilotmega, msg_id: 173, id: `mavlink::ardupilotmega::msg::RANGEFINDER::MSG_ID`]


## MAVLink Publications
- None
