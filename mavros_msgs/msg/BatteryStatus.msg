# Represent battery status from SYSTEM_STATUS
#
# To be replaced when sensor_msgs/BatteryState PR will be merged
# https://github.com/ros/common_msgs/pull/74

std_msgs/Header header
float32 voltage # [V]
float32 current # [A]
float32 remaining # 0..1
