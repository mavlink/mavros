mav CLI
=======

`mav` is the MAVROS command-line helper. It talks to a running `mavros` UAS node
over the ROS 2 graph and lets you query and control the autopilot from the
terminal without writing code.

It is installed together with the `mavros` Python package and is run via
`ros2 run`:

```shell
ros2 run mavros mav --help
```

## Global options

- `--node-name` - name of the CLI's own ROS 2 node (default: random).
- `--mavros-ns` - namespace of the `mavros::UAS` node (default `/mavros`).
- `--verbose` - verbose output.
- `--wait-fcu` - wait for the FCU connection before running.
- `--version` - print the MAVROS version and exit.

## Subcommands

### `mav checkid`

Verify the target address and list MAVLink messages arriving at the UAS node.

- `-f, --follow` - keep running instead of exiting after the first report.
- `--watch-time` - watch period in seconds (default `15.0`).

### `mav cmd`

Send MAVLink commands to the device.

- `long` - send a `COMMAND_LONG` (confirmation, broadcast, 7 float params).
- `int` - send a `COMMAND_INT` (`--current`, `--autocontinue`, `--frame`, broadcast).
- `set_home` - set the home position (`--current-gps`, lat/lon/alt).
- `takeoff` - take off to a given position (min pitch, yaw, lat/lon/alt).
- `land` - land at a given position (yaw, lat/lon/alt).
- `takeoff_cur` - take off at the current position (min pitch, yaw, altitude).
- `land_cur` - land at the current position (yaw, altitude).
- `trigger_control` - enable/disable/reset the camera trigger.
- `trigger_interval` - set the camera trigger interval.

### `mav param`

Manipulate parameters of the MAVLink device.

- `load FILE` - load parameters from a file (Mission Planner / QGroundControl / MAVProxy formats).
- `dump FILE` - dump parameters to a file (`--force` to re-pull from the FCU).
- `get PARAM_ID` - print a single parameter value.
- `set PARAM_ID VALUE` - set a single parameter value.

### `mav mission` (`mav wp`)

Work with mission waypoints, geofences and rally points.

- `pull` - pull mission/fence/rally points from the FCU.
- `show` - show mission, fence or rally points (`--follow` to watch).
- `dump FILE` - save points to a QGC WPL / Plan file.
- `clear` - clear mission/fence/rally points.
- `setcur SEQ` - set the active/current waypoint.

### `mav ftp`

Simple MAVLink FTP client.

- `cd PATH` - change directory.
- `ls [PATH]` - list directory.
- `cat PATH` - print a file.
- `rm PATH` - remove a file.
- `reset` - reset the FTP session.
- `mkdir PATH` - create a directory.
- `rmdir PATH` - remove a directory.
- `download SRC DEST` - download a file (`--verify`, progress bar).
- `upload SRC DEST` - upload a file (`--verify`, `--overwrite`, progress bar).
- `verify LOCAL REMOTE` - verify a file against the remote copy.

### `mav safety`

Send safety commands.

- `arm` - arm the vehicle.
- `disarm` - disarm the vehicle.
- `kill` - emergency kill.

### `mav sys`

System-level operations.

- `mode` - set the flight mode (`--base-mode`, `--custom-mode`).
- `rate` - set MAVLink stream rates.
- `message_interval MSGID RATE` - set the interval of a MAVLink message.

## Example

```shell
# wait for the FCU, then print the current mode and arm
mav --wait-fcu sys mode --custom-mode OFFBOARD
mav safety arm
```