# libevents_vendor

This ROS 2 package builds and installs the C++ library from
[mavlink/libevents](https://github.com/mavlink/libevents). MAVROS consumes it
through `find_package(libevents_vendor REQUIRED)` and links the parser target.
MAVROS's system-status plugin owns the event logging integration.

## Pinned source

- Revision: [`0ef305faa4cffdb09f01ca862ebfc108b7ff0cd5`](https://github.com/mavlink/libevents/commit/0ef305faa4cffdb09f01ca862ebfc108b7ff0cd5)
- Commit date: September 27, 2026.
- Archive: `https://codeload.github.com/mavlink/libevents/tar.gz/0ef305faa4cffdb09f01ca862ebfc108b7ff0cd5`
- Archive SHA-256: `a0f0e3b04967d13c0f3763eefcbf024fbd3de2afb6a39f5307f5fc1b4aca8990`

`ExternalProject_Add` downloads the archive, checks its hash, and builds the
upstream `libs/cpp` project without source patches. CMake 3.19 or newer is
required. Initial source builds require access to the archive host; this is
build-time traffic, not traffic to the vehicle.

Upstream libraries are built statically with position-independent code for use
inside MAVROS shared libraries. Upstream tests are disabled in this dependency
build. The upstream build also produces its health/arming library; linking the
parser target does not link that library or the MAVLink receive interface.

## CMake interface

```cmake
find_package(libevents_vendor REQUIRED)
target_link_libraries(your_target PRIVATE libevents_vendor::parser)
```

The target links upstream's exported `libevents_parser` target and provides the
installed header search path. Consumers can include
`<libevents/parse/parser.h>`. The upstream install includes `parser.h`, its
relative common header, and the bundled JSON forward-declaration header.
The wrapper also installs the bundled `nlohmann/json.hpp` header alongside
`json_fwd.hpp`, so MAVROS metadata validation uses the same JSON revision as
the parser. This does not modify the upstream source.

The library is installed under `opt/libevents_vendor` within the ROS install
prefix. The vendor package's CMake configuration resolves that location relative
to itself; consumers do not need an absolute path to the build workspace.

## Licensing and release

The ROS wrapper follows the MAVROS repository's BSD/GPLv3/LGPLv3 licensing.
Upstream libevents is BSD-3-Clause; its bundled nlohmann JSON 3.11.3 is MIT.
The notices in `LICENSE.libevents` and `LICENSE.nlohmann-json` are installed
with this README under `share/libevents_vendor`.

The vendor package is included in this repository so `colcon` can build it
before MAVROS and `rosdep --ignore-src` can recognize the local dependency.
Binary distribution requires the new package to be included in a future MAVROS
ROS release. No released `libevents_vendor` package is assumed or provided by
this source change. Release registration and build-farm validation remain part
of the upstream handoff.

Update both the commit and archive hash when updating libevents. Recheck the
installed CMake targets, header layout, licenses, and supported build environments
before accepting a new revision.
