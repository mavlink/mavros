# obstacle_distance_3d

- File: `mavros_extras/src/plugins/obstacle_distance_3d.cpp`
- Class: `mavros::extra_plugins::ObstacleDistance3DPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Plugin to handle sending OBSTACLE_DISTANCE_3D MAVLink messages.


Subscribes to a mavros_msgs/ObstacleDistance3D message and sends the
data to the flight controller to report the position of a single obstacle.

## Publishers
- None

## Subscribers
- `~/send` [type: [mavros_msgs::msg::ObstacleDistance3D](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ObstacleDistance3D.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to ObstacleDistance3D to send as OBSTACLE_DISTANCE_3D to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `mav_frame` [type: string, default: `"LOCAL_NED"`] - Add a configurable parameter for the MAVLink frame, just like the reference. MAVLink MAV_FRAME used when sending OBSTACLE_DISTANCE_3D.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`OBSTACLE_DISTANCE_3D`](https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D) [arg: `msg`, dialect: ardupilotmega, msg_id: 11037, id: `mavlink::ardupilotmega::msg::OBSTACLE_DISTANCE_3D::MSG_ID`]
