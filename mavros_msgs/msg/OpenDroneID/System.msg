# Remote ID message - System

std_msgs/Header header

uint8 operator_location_type
uint8 MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF = 0
uint8 MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS = 1
uint8 MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED = 2

uint8 classification_type
uint8 MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED = 0
uint8 MAV_ODID_CLASSIFICATION_TYPE_EU = 1

int32 operator_latitude
int32 operator_longitude
uint16 area_count
uint16 area_radius
float32 area_ceiling
float32 area_floor
uint8 category_eu
uint8 class_eu
float32 operator_altitude_geo
uint32 timestamp
