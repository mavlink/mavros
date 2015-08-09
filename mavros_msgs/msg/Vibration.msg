# VIBRATION message data
# @description: Vibration levels and accelerometer clipping

std_msgs/Header header

geometry_msgs/Vector3 vibration		# 3-axis vibration levels
float32[3] clipping				# Accelerometers clipping