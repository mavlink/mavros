# HilStateQuaternion.msg
#
# ROS representation of MAVLink HIL_STATE_QUATERNION
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION

std_msgs/Header header
float32[4] quaternion_wxyz
float32 rollspeed
float32 pitchspeed
float32 yawspeed
int32 lat
int32 lon
int32 alt
int16 vx
int16 vy
int16 vz
uint16 ind_airspeed
uint16 true_airspeed
int16 xacc
int16 yacc
int16 zacc