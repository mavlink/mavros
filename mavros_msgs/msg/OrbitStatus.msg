uint8 ORBIT_EXECUTION_STATE_UNKNOWN = 0 # For back-compatibility.
uint8 ORBIT_EXECUTION_STATE_APPROACH_TO_CIRCLE = 1
uint8 ORBIT_EXECUTION_STATE_STARTED = 2
uint8 ORBIT_EXECUTION_STATE_FINISHED = 3  # Meaningless for infinite orbits.

std_msgs/Header header # involves time since system start (microseconds) and coordinate system of the fields: x, y, z
float32 radius   # Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise. [m]
float64 x        # X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
float64 y        # Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters * 1e4, global = latitude in degrees * 1e7.
float32 z        # Altitude of center point. Coordinate system depends on frame field.
uint8 state # See enum ORBIT_EXECUTION_STATE.