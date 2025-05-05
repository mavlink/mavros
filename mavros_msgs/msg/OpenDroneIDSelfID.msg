# Remote ID message - Self ID
# https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_SELF_ID

# [[[cog:
# import mavros_cog
# mavros_cog.idl_decl_enum('MAV_ODID_DESC_TYPE', 'DESC_TYPE_')
# ]]]
# MAV_ODID_DESC_TYPE
uint8 DESC_TYPE_TEXT = 0                 # Free-form text description of the purpose of the flight.
uint8 DESC_TYPE_EMERGENCY = 1            # Optional additional clarification when status == MAV_ODID_STATUS_EMERGENCY.
uint8 DESC_TYPE_EXTENDED_STATUS = 2      # Optional additional clarification when status != MAV_ODID_STATUS_EMERGENCY.
# [[[end]]] (checksum: 08bf4b7e669d01d655893e5d543f86b1)

std_msgs/Header header
string id_or_mac
uint8 description_type
string description
