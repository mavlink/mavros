# debug_value

- File: `mavros_extras/src/plugins/debug_value.cpp`
- Class: `mavros::extra_plugins::DebugValuePlugin`
- Namespace: `debug_value`
- Brief: Plugin for Debug msgs from MAVLink API


## Publishers
- `~/debug` (DV) - publishers
- `~/debug_vector` (DV)
- `~/debug_float_array` (DV)
- `~/named_value_float` (DV)
- `~/named_value_int` (DV)


## Subscribers
- `~/send` (DV)


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `DEBUG` [handler: handle_debug, dialect: common, msg_id: 254, id: `mavlink::common::msg::DEBUG::MSG_ID`]
- `DEBUG_VECT` [handler: handle_debug_vector, dialect: common, msg_id: 250, id: `mavlink::common::msg::DEBUG_VECT::MSG_ID`]
- `DEBUG_FLOAT_ARRAY` [handler: handle_debug_float_array, dialect: common, msg_id: 350, id: `mavlink::common::msg::DEBUG_FLOAT_ARRAY::MSG_ID`]
- `NAMED_VALUE_FLOAT` [handler: handle_named_value_float, dialect: common, msg_id: 251, id: `mavlink::common::msg::NAMED_VALUE_FLOAT::MSG_ID`]
- `NAMED_VALUE_INT` [handler: handle_named_value_int, dialect: common, msg_id: 252, id: `mavlink::common::msg::NAMED_VALUE_INT::MSG_ID`]


## MAVLink Publications
- `DEBUG` [arg: `debug`, dialect: common, msg_id: 254, id: `mavlink::common::msg::DEBUG::MSG_ID`]
- `DEBUG_VECT` [arg: `debug`, dialect: common, msg_id: 250, id: `mavlink::common::msg::DEBUG_VECT::MSG_ID`] - [[[end]]] (sum: 4zWbFMda3z)
- `DEBUG_FLOAT_ARRAY` [arg: `debug`, dialect: common, msg_id: 350, id: `mavlink::common::msg::DEBUG_FLOAT_ARRAY::MSG_ID`]
- `NAMED_VALUE_FLOAT` [arg: `value`, dialect: common, msg_id: 251, id: `mavlink::common::msg::NAMED_VALUE_FLOAT::MSG_ID`]
- `NAMED_VALUE_INT` [arg: `value`, dialect: common, msg_id: 252, id: `mavlink::common::msg::NAMED_VALUE_INT::MSG_ID`]
