# rangefinder

- File: `mavros_extras/src/plugins/rangefinder.cpp`
- Class: `mavros::extra_plugins::RangefinderPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Ardupilot Rangefinder plugin.


This plugin allows publishing rangefinder sensor data from Ardupilot FCU to ROS.

## Publishers
- `~/rangefinder` [type: [sensor_msgs::msg::Range](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Range.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish rangefinder data from MAVLink RANGEFINDER.

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
