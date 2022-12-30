# Msg for Debug MAVLink API
#
# Supported types:
# DEBUG			https://mavlink.io/en/messages/common.html#DEBUG
# DEBUG_VECTOR			https://mavlink.io/en/messages/common.html#DEBUG_VECT
# DEBUG_FLOAT_ARRAY      https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY
# NAMED_VALUE_FLOAT		https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
# NAMED_VALUE_INT		https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT

std_msgs/Header header

int32 index			# index value of DEBUG value (-1 if not indexed)
int32 array_id      # Unique ID used to discriminate between DEBUG_FLOAT_ARRAYS (-1 if not used)

string name			# value name/key

float32 value_float		# float value for NAMED_VALUE_FLOAT and DEBUG
int32 value_int		# int value for NAMED_VALUE_INT
float32[] data			# DEBUG vector or array

uint8 type
uint8 TYPE_DEBUG		= 0
uint8 TYPE_DEBUG_VECT		= 1
uint8 TYPE_DEBUG_FLOAT_ARRAY	= 2
uint8 TYPE_NAMED_VALUE_FLOAT	= 3
uint8 TYPE_NAMED_VALUE_INT	= 4
