# RTKBaseline received from the FCU.
# Full description: https://mavlink.io/en/messages/common.html#GPS_RTK
# Mavlink Common, #127and #128

std_msgs/Header header

uint32 time_last_baseline_ms
uint8 rtk_receiver_id
uint16 wn
uint32 tow
uint8 rtk_health
uint8 rtk_rate
uint8 nsats

uint8 baseline_coords_type 
uint8 RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0   # Earth-centered, earth-fixed
uint8 RTK_BASELINE_COORDINATE_SYSTEM_NED = 1    # RTK basestation centered, north, east, down

int32 baseline_a_mm
int32 baseline_b_mm
int32 baseline_c_mm
uint32 accuracy
int32 iar_num_hypotheses
