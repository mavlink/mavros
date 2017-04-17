# MAVLink message: HOME_POSITION
# http://mavlink.org/messages/common#HOME_POSITION

std_msgs/Header header

geographic_msgs/GeoPoint geo # lla WGS84
geometry_msgs/Pose pose	# local position
geometry_msgs/Vector3 approach	# position of the end of approach vector
