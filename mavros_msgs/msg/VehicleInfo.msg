# Vehicle Info msg
std_msgs/Header header
uint8 type                      #MAV_TYPE
uint8 autopilot                 #MAV_AUTOPILOT
string mode                     #MAV_MODE string
uint32 mode_id                  #MAV_MODE number
uint8 system_status             #MAV_STATE
uint8 sysid                     #SYSTEM ID
uint8 compid                    #COMPONENT ID
uint64 capabilities             #MAV_PROTOCOL_CAPABILITY
uint32 flight_sw_version        #Firmware version number
uint32 middleware_sw_version    #Middleware version number
uint32 os_sw_version            #Operating system version number
uint32 board_version            #HW / board version (last 8 bytes should be silicon ID, if any)
uint16 vendor_id                #ID of the board vendor
uint16 product_id               #ID of the product
uint64 uid                      #UID if provided by hardware