# MAVLink message: HOME_POSITION
# https://mavlink.io/en/messages/common.html#HOME_POSITION

std_msgs/Header header

geographic_msgs/GeoPoint geo # geodetic coordinates in WGS-84 datum

geometry_msgs/Point position	# local position
geometry_msgs/Quaternion orientation	# XXX: verify field name (q[4])
geometry_msgs/Vector3 approach	# position of the end of approach vector
