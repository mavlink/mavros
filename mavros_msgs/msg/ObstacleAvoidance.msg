# MAVLink message: OBSTACLE_AVOIDANCE
# http://mavlink.org/messages/common#OBSTACLE_AVOIDANCE

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
int16[4] field_of_view # Field of View of the sensor/s being used. The first two parameters are used to indicate the direction (Azimuth: 0 360 degrees and elevation: -90 - 90 degrees) and the last two indicate the range (Azimuth and elevation). The range is applied symmetrically in the direction. Set first element to -1 to mark as unknown.

