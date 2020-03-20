# HilActuatorControls.msg
#
# ROS representation of MAVLink SERVO_OUTPUT_RAW
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW

std_msgs/Header header
uint8 port	        #	Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
uint16 servo1_raw	#   us	Servo output 1 value
uint16 servo2_raw	#   us	Servo output 2 value
uint16 servo3_raw	#   us	Servo output 3 value
uint16 servo4_raw	#   us	Servo output 4 value
uint16 servo5_raw	#   us	Servo output 5 value
uint16 servo6_raw	#   us	Servo output 6 value
uint16 servo7_raw	#   us	Servo output 7 value
uint16 servo8_raw	#   us	Servo output 8 value
uint16 servo9_raw   #   us	Servo output 9 value
uint16 servo10_raw  #   us	Servo output 10 value
uint16 servo11_raw  #   us	Servo output 11 value
uint16 servo12_raw  #   us	Servo output 12 value
uint16 servo13_raw  #   us	Servo output 13 value
uint16 servo14_raw  #   us	Servo output 14 value
uint16 servo15_raw  #   us	Servo output 15 value
uint16 servo16_raw  #   us	Servo output 16 value
