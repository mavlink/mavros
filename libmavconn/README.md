MAVCONN library
===============

It is mavlink connection and communication library used in [MAVROS][mr].
Since 2014-11-02 it adopted to use outside from ROS environment
by splitting to individual package and removing dependencies to rosconsole.


Connection URL
--------------

Connection defined by URL.
Just pass one of that URL to `MAVConnInterface::open_url()` and get connection object.

Supported schemas:

  - Serial: `/path/to/serial/device[:baudrate]`
  - Serial: `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - Serial with hardware flow control: `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
  - UDP: `udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]`
  - UDP broadcast until GCS discovery: `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - UDP broadcast (permanent): `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]`
  - TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
  - TCP server: `tcp-l://[bind_port][:port][/?ids=sysid,compid]`

Note: ids from URL overrides ids given by system\_id & component\_id parameters.


Dependencies
------------

Same as for mavros:

  - Linux host
  - Asio library ( https://think-async.com/Asio/ )
  - console-bridge library
  - compiler with C++20 support


Shared io_service (optional)
----------------------------

By default each connection owns and runs its own `asio::io_service` thread.
For multi-connection setups you can provide a shared `asio::io_service` and
run it from your own thread pool.

```cpp
#include <asio.hpp>
#include <mavconn/interface.hpp>

asio::io_service shared_io;
auto work = std::make_unique<asio::io_service::work>(shared_io);
std::jthread io_thread([&]() { shared_io.run(); });

auto conn = mavconn::MAVConnInterface::open_url(
  "udp://0.0.0.0:14555@127.0.0.1:14550",
  1, mavconn::MAV_COMP_ID_UDP_BRIDGE,
  [](const mavlink::mavlink_message_t *, mavconn::Framing) {},
  {},
  &shared_io);

conn->close();
work.reset();
shared_io.stop();
```


License
-------

Licensed under terms of [*LGPLv3*][lgpllic], [*BSD*][bsdlic], or [*GPLv3*][gpllic].


[mr]: https://github.com/mavlink/mavros
[lgpllic]: https://www.gnu.org/licenses/lgpl.html
[gpllic]: https://www.gnu.org/licenses/gpl.html
[bsdlic]: https://github.com/mavlink/mavros/blob/master/LICENSE-BSD.txt
