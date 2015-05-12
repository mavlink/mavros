# DISTANCE_SENSOR message data

Header header

sensor_msgs/Range range

uint8 type			# see MAV_DISTANCE_SENSOR enum
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1

uint8 id 			# XXX FIXME enum ??

uint8 orientation 	# rad
uint8 covariance	# 1E-2 meters; 0 for unknown/invalid readings