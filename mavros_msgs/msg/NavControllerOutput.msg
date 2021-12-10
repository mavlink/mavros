# https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT

std_msgs/Header header

float32 nav_roll            # Current desired roll
float32 nav_pitch           # Current desired pitch
int16 nav_bearing           # Current desired heading
int16 target_bearing        # Bearing to current waypoint/target
uint16 wp_dist              # Distance to active waypoint
float32 alt_error           # Current altitude error
float32 aspd_error          # Current airspeed error
float32 xtrack_error        # Current crosstrack error on x-y plane