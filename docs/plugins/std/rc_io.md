# rc_io

- File: `mavros/src/plugins/rc_io.cpp`
- Class: `mavros::std_plugins::RCIOPlugin`
- Namespace: `rc`
- Brief: RC IO plugin


## Publishers
- `~/in` (mavros_msgs::msg::RCIn)
- `~/out` (mavros_msgs::msg::RCOut)


## Subscribers
- `~/override` (mavros_msgs::msg::OverrideRCIn)


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `RC_CHANNELS_RAW` [handler: handle_rc_channels_raw, dialect: common, msg_id: 35, id: `mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID`]
- `RC_CHANNELS` [handler: handle_rc_channels, dialect: common, msg_id: 65, id: `mavlink::common::msg::RC_CHANNELS::MSG_ID`]
- `SERVO_OUTPUT_RAW` [handler: handle_servo_output_raw, dialect: common, msg_id: 36, id: `mavlink::common::msg::SERVO_OUTPUT_RAW::MSG_ID`]


## MAVLink Publications
- `RC_CHANNELS_OVERRIDE` [arg: `ovr`, dialect: common, msg_id: 70, id: `mavlink::common::msg::RC_CHANNELS_OVERRIDE::MSG_ID`] - [[[end]]] (sum: +HglKUW20c)
