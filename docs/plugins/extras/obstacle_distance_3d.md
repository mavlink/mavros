# obstacle_distance_3d

- File: `mavros_extras/src/plugins/obstacle_distance_3d.cpp`
- Class: `mavros::extra_plugins::ObstacleDistance3DPlugin`
- Namespace: `obstacle_distance_3d`
- Brief: Plugin to handle sending OBSTACLE_DISTANCE_3D MAVLink messages.


Subscribes to a mavros_msgs/ObstacleDistance3D message and sends the data to the flight controller to report the position of a single obstacle.

## Publishers
- None


## Subscribers
- `~/send` ([mavros_msgs::msg::ObstacleDistance3D](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ObstacleDistance3D.html))


## Services
- None


## Clients
- None


## Parameters
- `mav_frame` [default: `"LOCAL_NED"`] - Add a configurable parameter for the MAVLink frame, just like the reference.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`OBSTACLE_DISTANCE_3D`](https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D) [arg: `obstacle`, dialect: ardupilotmega, msg_id: 11037, id: `mavlink::ardupilotmega::msg::OBSTACLE_DISTANCE_3D::MSG_ID`]
