# Current autopilot estimator state
#
# https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS_FLAGS

std_msgs/Header header
bool attitude_status_flag

bool velocity_horiz_status_flag
bool velocity_vert_status_flag

bool pos_horiz_rel_status_flag
bool pos_horiz_abs_status_flag

bool pos_vert_abs_status_flag
bool pos_vert_agl_status_flag

bool const_pos_mode_status_flag

bool pred_pos_horiz_rel_status_flag
bool pred_pos_horiz_abs_status_flag

bool gps_glitch_status_flag
bool accel_error_status_flag