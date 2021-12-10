# FCU GPS INPUT message for the gps_input plugin
# <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">mavlink GPS_INPUT message</a>.

std_msgs/Header header
## GPS_FIX_TYPE enum
uint8 GPS_FIX_TYPE_NO_GPS     = 0    # No GPS connected
uint8 GPS_FIX_TYPE_NO_FIX     = 1    # No position information, GPS is connected
uint8 GPS_FIX_TYPE_2D_FIX     = 2    # 2D position
uint8 GPS_FIX_TYPE_3D_FIX     = 3    # 3D position
uint8 GPS_FIX_TYPE_DGPS       = 4    # DGPS/SBAS aided 3D position
uint8 GPS_FIX_TYPE_RTK_FLOATR = 5    # TK float, 3D position
uint8 GPS_FIX_TYPE_RTK_FIXEDR = 6    # TK Fixed, 3D position
uint8 GPS_FIX_TYPE_STATIC     = 7    # Static fixed, typically used for base stations
uint8 GPS_FIX_TYPE_PPP        = 8    # PPP, 3D position
uint8 fix_type      # [GPS_FIX_TYPE] GPS fix type

uint8 gps_id        # ID of the GPS for multiple GPS inputs
uint16 ignore_flags # Bitmap indicating which GPS input flags fields to ignore. All other fields must be provided.

uint32 time_week_ms # [ms] GPS time (from start of GPS week)
uint16 time_week    # GPS week number
int32 lat           # [degE7] Latitude (WGS84, EGM96 ellipsoid)
int32 lon           # [degE7] Longitude (WGS84, EGM96 ellipsoid)
float32 alt         # [m] Altitude (MSL). Positive for up.

float32 hdop        # [m] GPS HDOP horizontal dilution of position.
float32 vdop        # [m] GPS VDOP vertical dilution of position
float32 vn          # [m/s] GPS velocity in NORTH direction in earth-fixed NED frame
float32 ve          # [m/s] GPS velocity in EAST direction in earth-fixed NED frame
float32 vd          # [m/s] GPS velocity in DOWN direction in earth-fixed NED frame

float32 speed_accuracy # [m/s] GPS speed accuracy
float32 horiz_accuracy # [m] GPS horizontal accuracy
float32 vert_accuracy  # [m] GPS vertical accuracy

uint8 satellites_visible # Number of satellites visible. If unknown, set to 255
uint16 yaw          # [cdeg] Yaw in earth frame from north.
