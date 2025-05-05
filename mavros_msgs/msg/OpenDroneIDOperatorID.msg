# Remote ID message - Operator ID

# [[[cog:
# import mavros_cog
# mavros_cog.idl_decl_enum('MAV_ODID_OPERATOR_ID_TYPE', 'ID_TYPE_')
# ]]]
# MAV_ODID_OPERATOR_ID_TYPE
uint8 ID_TYPE_CAA = 0                    # CAA (Civil Aviation Authority) registered operator ID.
# [[[end]]] (checksum: be1240b63d8b101630595dd9ae4a42d4)

std_msgs/Header header
uint8 operator_id_type
string operator_id
