# MAVLink message: LANDING_TARGET
# https://mavlink.io/en/messages/common.html

std_msgs/Header header

## MAV_FRAME enum
uint8 GLOBAL = 0                   # Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
uint8 LOCAL_NED = 2                # Local coordinate frame, Z-up (x: north, y: east, z: down).
uint8 MISSION = 3                  # NOT a coordinate frame, indicates a mission command.
uint8 GLOBAL_RELATIVE_ALT = 4      # Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
uint8 LOCAL_ENU = 5                # Local coordinate frame, Z-down (x: east, y: north, z: up)
uint8 GLOBAL_INT = 6               # Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
uint8 GLOBAL_RELATIVE_ALT_INT = 7  # Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
uint8 LOCAL_OFFSET_NED = 8         # Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
uint8 BODY_NED = 9                 # Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
uint8 BODY_OFFSET_NED = 10         # Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
uint8 GLOBAL_TERRAIN_ALT = 11      # Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
uint8 GLOBAL_TERRAIN_ALT_INT = 12  # Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.

## LANDING_TARGET_TYPE enum
uint8 LIGHT_BEACON = 0             # Landing target signaled by light beacon (ex: IR-LOCK)
uint8 RADIO_BEACON = 1             # Landing target signaled by radio beacon (ex: ILS, NDB)
uint8 VISION_FIDUCIAL = 2          # Landing target represented by a fiducial marker (ex: ARTag)
uint8 VISION_OTHER = 3             # Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)

uint8 target_num
uint8 frame
float32[2] angle
float32 distance
float32[2] size
geometry_msgs/Pose pose
uint8 type
