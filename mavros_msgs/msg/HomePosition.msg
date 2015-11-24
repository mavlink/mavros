# MAVLink message: HOME_POSITION
# http://mavlink.org/messages/common#HOME_POSITION

std_msgs/Header header

float64 latitude	# WGS84
float64 longitude
float32 altitude	# AMSL

geometry_msgs/Vector3 position	# local position
geometry_msgs/Quaternion orientation	# XXX: verify field name (q[4])
geometry_msgs/Vector3 approach	# position of the end of approach vector
