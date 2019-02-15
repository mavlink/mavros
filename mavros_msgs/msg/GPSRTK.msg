# FCU GPS RTK message for the gps_status plugin
# A copy of <a href="https://mavlink.io/en/messages/common.html#GPS_RTK">mavlink GPS_RTK message</a>

std_msgs/Header header

uint8 rtk_receiver_id      # Identification of connected RTK receiver.
int16 wn                   # GPS Week Number of last baseline.
uint32 tow                 # [ms] GPS Time of Week of last baseline.
uint8 rtk_health           # GPS-specific health report for RTK data.
uint8 rtk_rate             # [Hz] Rate of baseline messages being received by GPS.
uint8 nsats                # Current number of sats used for RTK calculation.
int32 baseline_a           # [mm] Current baseline in ECEF x or NED north component, depends on header.frame_id.
int32 baseline_b           # [mm] Current baseline in ECEF y or NED east component, depends on header.frame_id.
int32 baseline_c           # [mm] Current baseline in ECEF z or NED down component, depends on header.frame_id.
uint32 accuracy            # Current estimate of baseline accuracy.
int32 iar_num_hypotheses   # Current number of integer ambiguity hypotheses.


