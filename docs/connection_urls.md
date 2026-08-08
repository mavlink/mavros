Connection URLs
===============

Both MAVROS and the underlying [libmavconn](https://github.com/mavlink/mavros/blob/master/libmavconn/README.md) transport
describe a link to an autopilot or ground station with a single URL. You can use
any supported type for the FCU and GCS connections.


Supported schemas
-----------------

  - Serial: `/path/to/serial/device[:baudrate]`
  - Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - Serial with hardware flow control: `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - UDP: `udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]`
  - UDP broadcast until GCS discovery: `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - UDP broadcast (permanent): `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
  - TCP server: `tcp-l://[bind_port][:port][/?ids=sysid,compid]`


Examples
--------

**UDP to a local SITL autopilot** (bind anywhere, send to SITL on 14550):

```
udp://0.0.0.0:14555@127.0.0.1:14550
```

**UDP, only bind the loopback port** (remote is filled in from the first
incoming packet - the classic "GCS discovers FCU" setup):

```
udp://127.0.0.1:14555@
```

**UDP broadcast** - we do not know the peer yet, so we broadcast to a port:

```
udp-b://0.0.0.0:14555@:14550
```

**TCP client** - MAVROS dials out to the autopilot (the autopilot is the server):

```
tcp://192.168.60.192:5760
```

**TCP server** - MAVROS listens and waits for the autopilot to connect:

```
tcp-l://0.0.0.0:5760
```

**Serial autopilot** on `/dev/ttyUSB0` at 57600 baud:

```
/dev/ttyUSB0:57600
```
or, with explicit MAVLink ids
```
serial:///dev/ttyUSB0:57600?ids=1,190
```

**Serial with hardware flow control**:

```
serial-hwfc:///dev/ttyTHS1:921600
```


Notes
-----

  - Ids from URL override the value given by `system_id` & `component_id` parameters.
  - `bind_host` - default `0.0.0.0` - i.e. IPv4 ANY (bind to all local interfaces).
  - UDP default ports: 14555 @ 14550 (bind port 14555, remote 14550).
  - UDP remote address is updated every time with each incoming packet on the bind port.
  - TCP default port: 5760.


How to read a URL
-----------------

A connection URL describes **two sockets at once**: where *we* sit (`bind`) and
where *the other side* sits (`remote`). It looks like this:

```
  udp://  0.0.0.0:14555   @   127.0.0.1:14550
  ──┬───  ──────┬──────   │   ──────┬────────
  scheme      bind       sep      remote
 (protocol)  (our end)           (their end)
```

Break it down:

- **`scheme`** - the transport protocol (`udp`, `tcp`, `udp-b`, …). This decides
  which type of Berkeley socket gets created.
- **`bind`** - the address *our* socket is bound to. This is the end where we
  `bind()` and listen for / receive from the other side. If you leave it empty or
  use `0.0.0.0`, we bind to **any** local interface (so we accept traffic from
  everywhere).
- **`@`** - the separator between *our* end and *their* end. Everything before it
  is the local endpoint, everything after is the remote endpoint.
- **`remote`** - the address of the *other* socket we send to. This is the end the
  `sendto()`/`connect()` target.

So the line above means: *"UDP socket bound on port 14555 of every local
interface, talking to the peer at 127.0.0.1:14550."*

The `[brackets]` in the schema mean **optional** and `:port` means there is a
default if you omit it. See the defaults in the [notes](#notes) above.


The optional `?ids=` tail
-------------------------

Everything after the `?` is a set of query options, currently only the MAVLink
addresses:

```
udp://0.0.0.0:14555@127.0.0.1:14550?ids=1,190
                                    └───┬───┘
                               system_id, component_id
```

- `system_id` - MAVLink system id of this side (default 1).
- `component_id` - MAVLink component id of this side (default 190).
- Ids given in the URL override the `system_id` & `component_id` parameters.
