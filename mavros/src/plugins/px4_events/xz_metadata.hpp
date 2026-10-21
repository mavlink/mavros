// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Bounded XZ metadata decompression
 * @file xz_metadata.hpp
 * @author Sean Hickey
 */

#ifndef PLUGINS__PX4_EVENTS__XZ_METADATA_HPP_
#define PLUGINS__PX4_EVENTS__XZ_METADATA_HPP_

#include <cstddef>
#include <string>
#include <string_view>

namespace mavros
{
namespace px4_events
{

// Validate the single-stream index before allocating exact-sized output.
// Throws on malformed input, unsupported compression, or an exceeded limit.
std::string unpack_xz_metadata(std::string_view bytes, size_t output_limit);

}  // namespace px4_events
}  // namespace mavros

#endif  // PLUGINS__PX4_EVENTS__XZ_METADATA_HPP_
