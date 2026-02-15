# distance_sensor

- File: `mavros_extras/src/plugins/distance_sensor.cpp`
- Class: `mavros::extra_plugins::DistanceSensorPlugin`
- Namespace: `distance_sensor`
- Brief: Distance sensor plugin


This plugin allows publishing distance sensor data, which is connected to an offboard/companion computer through USB/Serial, to the FCU or vice-versa.

## Publishers
- `topic_name` (Range)


## Subscribers
- `topic_name` (Range)


## Services
- None


## Clients
- None


## Parameters
- `base_frame_id` [default: `"base_link"`]
- `config` [type: string, default: `""`]


## MAVLink Subscriptions
- `DISTANCE_SENSOR` [handler: handle_distance_sensor, dialect: common, msg_id: 132, id: `mavlink::common::msg::DISTANCE_SENSOR::MSG_ID`]


## MAVLink Publications
- `DISTANCE_SENSOR` [arg: `ds`, dialect: common, msg_id: 132, id: `mavlink::common::msg::DISTANCE_SENSOR::MSG_ID`]
