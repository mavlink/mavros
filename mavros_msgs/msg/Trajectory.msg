# MAVLink message: TRAJECTORY
# https://mavlink.io/en/messages/common.html#TRAJECTORY

std_msgs/Header header

uint8 type # See enum MAV_TRAJECTORY_REPRESENTATION.
uint8 MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0
uint8 MAV_TRAJECTORY_REPRESENTATION_BEZIER = 1

mavros_msgs/PositionTarget point_1
mavros_msgs/PositionTarget point_2
mavros_msgs/PositionTarget point_3
mavros_msgs/PositionTarget point_4
mavros_msgs/PositionTarget point_5

uint8[5] point_valid # States if respective point is valid.
uint16[5] command # MAV_CMD associated with each point. UINT16_MAX if unused.

float32[5] time_horizon # if type MAV_TRAJECTORY_REPRESENTATION_BEZIER, it represents the time horizon for each point, otherwise set to NaN
