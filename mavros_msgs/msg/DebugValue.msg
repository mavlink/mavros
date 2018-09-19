# Msg for Debug MAVLink API
#
# Supported types:
# DEBUG			http://mavlink.org/messages/common#DEBUG
# DEBUG_VECTOR			http://mavlink.org/messages/common#DEBUG_VECT
# NAMED_VALUE_FLOAT		http://mavlink.org/messages/common#NAMED_VALUE_FLOAT
# NAMED_VALUE_INT		http://mavlink.org/messages/common#NAMED_VALUE_INT
# @TODO: add support for DEBUG_ARRAY (https://github.com/mavlink/mavlink/pull/734)

std_msgs/Header header

int32 index			# index value of DEBUG value (-1 if not indexed)

string name			# value name/key

float32 value_float		# float value for NAMED_VALUE_FLOAT and DEBUG
int32 value_int		# int value for NAMED_VALUE_INT
float32[] data			# DEBUG vector or array

uint8 type
uint8 TYPE_DEBUG		= 0
uint8 TYPE_DEBUG_VECT		= 1
uint8 TYPE_DEBUG_ARRAY		= 2
uint8 TYPE_NAMED_VALUE_FLOAT	= 3
uint8 TYPE_NAMED_VALUE_INT	= 4
