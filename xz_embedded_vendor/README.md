# XZ Embedded vendor package

Packages the upstream userspace decoder from
<https://github.com/tukaani-project/xz-embedded> at commit
`ae63ae3a36ed01724674e8f3d750dc47bf125410`, with a SHA-256-verified source archive.
Upstream sources are built unchanged using the small CMake wrapper in `upstream/`.
The upstream 0BSD license is reproduced in `LICENSE.xz-embedded`.

Consumers use `find_package(xz_embedded_vendor REQUIRED)` and link
`xz_embedded_vendor::decoder`. The header is `<xz_embedded/xz.h>`.

Only single-call mode is compiled: output storage doubles as the dictionary,
so the dictionary size advertised by an input does not cause a separate large
allocation. CRC32, CRC64, and SHA-256 checks are enabled. Unsupported filters or
integrity checks are rejected, not ignored. BCJ and Delta filters are not enabled;
the supported metadata format uses LZMA2 compression of JSON.

The wrapper forwards the C compiler, build type, and optional toolchain file to
the nested build, including sanitizer flags supplied through a toolchain file.
Upstream does not provide a CMake project or ROS package; this package supplies
only that integration. ROS registration and release remain upstream PR items.
