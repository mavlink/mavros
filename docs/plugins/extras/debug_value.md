# debug_value

- File: `mavros_extras/src/plugins/debug_value.cpp`
- Class: `mavros::extra_plugins::DebugValuePlugin`
- Namespace: `debug_value`
- Brief: Plugin for Debug msgs from MAVLink API


## Publishers
- `~/debug` (DV) - publishers Publish DEBUG messages from MAVLink DEBUG.
- `~/debug_vector` (DV) - Publish DEBUG_VECT messages from MAVLink DEBUG_VECT.
- `~/debug_float_array` (DV) - Publish DEBUG_FLOAT_ARRAY messages from MAVLink DEBUG_FLOAT_ARRAY.
- `~/named_value_float` (DV) - Publish NAMED_VALUE_FLOAT messages from MAVLink NAMED_VALUE_FLOAT.
- `~/named_value_int` (DV) - Publish NAMED_VALUE_INT messages from MAVLink NAMED_VALUE_INT.


## Subscribers
- `~/send` (DV) - subscribers Subscribe to DebugValue to send MAVLink debug messages to the FCU.


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`DEBUG`](https://mavlink.io/en/messages/common.html#DEBUG) [handler: handle_debug, dialect: common, msg_id: 254, id: `mavlink::common::msg::DEBUG::MSG_ID`]
- [`DEBUG_VECT`](https://mavlink.io/en/messages/common.html#DEBUG_VECT) [handler: handle_debug_vector, dialect: common, msg_id: 250, id: `mavlink::common::msg::DEBUG_VECT::MSG_ID`]
- [`DEBUG_FLOAT_ARRAY`](https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY) [handler: handle_debug_float_array, dialect: common, msg_id: 350, id: `mavlink::common::msg::DEBUG_FLOAT_ARRAY::MSG_ID`]
- [`NAMED_VALUE_FLOAT`](https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT) [handler: handle_named_value_float, dialect: common, msg_id: 251, id: `mavlink::common::msg::NAMED_VALUE_FLOAT::MSG_ID`]
- [`NAMED_VALUE_INT`](https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT) [handler: handle_named_value_int, dialect: common, msg_id: 252, id: `mavlink::common::msg::NAMED_VALUE_INT::MSG_ID`]


## MAVLink Publications
- [`DEBUG`](https://mavlink.io/en/messages/common.html#DEBUG) [arg: `debug`, dialect: common, msg_id: 254, id: `mavlink::common::msg::DEBUG::MSG_ID`]
- [`DEBUG_VECT`](https://mavlink.io/en/messages/common.html#DEBUG_VECT) [arg: `debug`, dialect: common, msg_id: 250, id: `mavlink::common::msg::DEBUG_VECT::MSG_ID`]
- [`DEBUG_FLOAT_ARRAY`](https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY) [arg: `debug`, dialect: common, msg_id: 350, id: `mavlink::common::msg::DEBUG_FLOAT_ARRAY::MSG_ID`]
- [`NAMED_VALUE_FLOAT`](https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT) [arg: `value`, dialect: common, msg_id: 251, id: `mavlink::common::msg::NAMED_VALUE_FLOAT::MSG_ID`]
- [`NAMED_VALUE_INT`](https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT) [arg: `value`, dialect: common, msg_id: 252, id: `mavlink::common::msg::NAMED_VALUE_INT::MSG_ID`]
