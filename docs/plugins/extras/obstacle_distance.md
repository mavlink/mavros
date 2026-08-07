# obstacle_distance

- File: `mavros_extras/src/plugins/obstacle_distance.cpp`
- Class: `mavros::extra_plugins::ObstacleDistancePlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Obstacle distance plugin


Publishes obstacle distance array to the FCU, in order to assist in an obstacle
avoidance flight.

## Publishers
- None

## Subscribers
- `~/send` [type: [sensor_msgs::msg::LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to LaserScan to send as OBSTACLE_DISTANCE to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `mav_frame` [type: string, default: `"GLOBAL"`] - MAVLink MAV_FRAME used when sending OBSTACLE_DISTANCE.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`OBSTACLE_DISTANCE`](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE) [arg: `msg`, dialect: common, msg_id: 330, id: `mavlink::common::msg::OBSTACLE_DISTANCE::MSG_ID`]
