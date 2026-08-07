# distance_sensor

- File: `mavros_extras/src/plugins/distance_sensor.cpp`
- Class: `mavros::extra_plugins::DistanceSensorPlugin`
- Namespace: `distance_sensor`
- Brief: Distance sensor plugin


This plugin allows publishing distance sensor data, which is connected to an offboard/companion computer through USB/Serial, to the FCU or vice-versa.

## Publishers
- `topic_name` (Range) - Publish sensor_msgs/Range from MAVLink DISTANCE_SENSOR.


## Subscribers
- `topic_name` (Range) - Subscribe sensor_msgs/Range to send as DISTANCE_SENSOR to the FCU.


## Services
- None


## Clients
- None


## Parameters
- `base_frame_id` [default: `"base_link"`] - Base frame id used for distance sensor transforms.
- `config` [type: string, default: `""`] - Sensor mapping configuration (YAML: id, topic, orientation, etc).


## MAVLink Subscriptions
- [`DISTANCE_SENSOR`](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR) [handler: handle_distance_sensor, dialect: common, msg_id: 132, id: `mavlink::common::msg::DISTANCE_SENSOR::MSG_ID`]


## MAVLink Publications
- [`DISTANCE_SENSOR`](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR) [arg: `ds`, dialect: common, msg_id: 132, id: `mavlink::common::msg::DISTANCE_SENSOR::MSG_ID`]
