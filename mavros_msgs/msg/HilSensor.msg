# HilSensor.msg
#
# ROS representation of MAVLink HIL_SENSOR
# See mavlink message documentation here:
# https://pixhawk.ethz.ch/mavlink/#HIL_SENSOR

std_msgs/Header header

geometry_msgs/Vector3 acc
geometry_msgs/Vector3 gyro
geometry_msgs/Vector3 mag
sensor_msgs/FluidPressure abs_pressure
sensor_msgs/FluidPressure diff_pressure
sensor_msgs/FluidPressure pressure_alt
sensor_msgs/Temperature temperature

uint32 fields_updated
