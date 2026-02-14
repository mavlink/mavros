# dummy

- File: `mavros/src/plugins/dummy.cpp`
- Class: `mavros::std_plugins::DummyPlugin`
- Namespace: `dummy`
- Brief: Dummy plugin.


@example_plugin Example and "how to" for users.

## Publishers
- None


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `HEARTBEAT` [handler: handle_heartbeat, dialect: minimal, msg_id: 0, id: `mavlink::minimal::msg::HEARTBEAT::MSG_ID`] - automatic message deduction by second argument
- `SYS_STATUS` [handler: handle_sys_status, dialect: common, msg_id: 1, id: `mavlink::common::msg::SYS_STATUS::MSG_ID`]
- `STATUSTEXT` [handler: handle_statustext_raw, dialect: common, msg_id: 253, id: `mavlink::common::msg::STATUSTEXT::MSG_ID`] - handle raw message, check framing!
- `STATUSTEXT` [handler: handle_statustext, dialect: common, msg_id: 253, id: `mavlink::common::msg::STATUSTEXT::MSG_ID`]


## MAVLink Publications
- None
