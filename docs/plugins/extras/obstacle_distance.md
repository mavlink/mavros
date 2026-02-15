# obstacle_distance

- File: `mavros_extras/src/plugins/obstacle_distance.cpp`
- Class: `mavros::extra_plugins::ObstacleDistancePlugin`
- Namespace: `obstacle`
- Brief: Obstacle distance plugin


Publishes obstacle distance array to the FCU, in order to assist in an obstacle avoidance flight. @see obstacle_cb()

## Publishers
- None


## Subscribers
- `~/send` ([sensor_msgs::msg::LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html))


## Services
- None


## Clients
- None


## Parameters
- `mav_frame` [default: `"GLOBAL"`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`OBSTACLE_DISTANCE`](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE) [arg: `obstacle`, dialect: common, msg_id: 330, id: `mavlink::common::msg::OBSTACLE_DISTANCE::MSG_ID`]
