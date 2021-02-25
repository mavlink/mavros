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
  - compiller with C++14 support


License
-------

Licensed under terms of [*LGPLv3*][lgpllic], [*BSD*][bsdlic], or [*GPLv3*][gpllic].


[mr]: https://github.com/mavlink/mavros
[lgpllic]: https://www.gnu.org/licenses/lgpl.html
[gpllic]: https://www.gnu.org/licenses/gpl.html
[bsdlic]: https://github.com/mavlink/mavros/blob/master/LICENSE-BSD.txt
