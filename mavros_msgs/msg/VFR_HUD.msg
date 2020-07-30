# Metrics typically displayed on a HUD for fixed wing aircraft
#
# VFR_HUD message

std_msgs/Header header
float32 airspeed	# m/s
float32 groundspeed	# m/s
int16 heading		# degrees 0..360
float32 throttle		# normalized to 0.0..1.0
float32 altitude		# MSL
float32 climb		# current climb rate m/s
