# HilStateQuaternion.msg
#
# ROS representation of MAVLink HIL_STATE_QUATERNION
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION

std_msgs/Header header
float32[4] quaternion_wxyz
float32[3] angularspeed # has rollspeed, pitchspeed, yawspeed (rad/s)
int32[3] GPS # has lat *1E7, lon *1E7, alt *1000(millimeters)
int16[3] groundspeed # has vx, vy, vz (cm/s)
uint16 ind_airspeed
uint16 true_airspeed
int16[3] acceleration # has xacc, yacc, zacc in milli g