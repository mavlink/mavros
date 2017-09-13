# WaypointList.msg
#
#  :current_seq:   seq nr of currently active waypoint
#                  waypoints[current_seq].is_current == True
#
#  :waypoints:     list of waypoints

uint16 current_seq
mavros_msgs/Waypoint[] waypoints
