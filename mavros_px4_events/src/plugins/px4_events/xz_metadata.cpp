// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Bounded XZ metadata decompression
 * @file xz_metadata.cpp
 * @author Sean Hickey
 */

#include "xz_metadata.hpp"

#include <lzma.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace mavros
{
namespace px4_events
{
namespace
{

// Independent of the JSON output limit: allow a 64 MiB dictionary plus overhead.
constexpr uint64_t DECODER_MEMORY_LIMIT = 128ULL * 1024 * 1024;

}  // namespace

std::string unpack_xz_metadata(std::string_view bytes, size_t output_limit)
{
  lzma_stream stream = LZMA_STREAM_INIT;
  std::unique_ptr<lzma_stream, decltype(& lzma_end)> cleanup(&stream, lzma_end);

  const auto check_result = [](lzma_ret result) {
      if (result == LZMA_MEMLIMIT_ERROR) {
        throw std::runtime_error("XZ metadata decoder exceeds 128 MiB memory limit");
      }
      if (result == LZMA_MEM_ERROR) {
        throw std::runtime_error("cannot allocate XZ metadata decoder");
      }
      if (result != LZMA_OK && result != LZMA_STREAM_END) {
        throw std::runtime_error("invalid, truncated, or unsupported XZ metadata");
      }
    };

  check_result(lzma_stream_decoder(
      &stream, DECODER_MEMORY_LIMIT,
      LZMA_CONCATENATED | LZMA_TELL_UNSUPPORTED_CHECK));

  stream.next_in = reinterpret_cast<const uint8_t *>(bytes.data());
  stream.avail_in = bytes.size();

  std::array<uint8_t, 4096> buffer{};
  std::string output;

  while (true) {
    const auto remaining = output_limit - output.size();
    // At the exact limit, allow footer processing but reject any extra byte.
    const auto available = remaining == 0 ? size_t{1} :
    std::min(buffer.size(), remaining);
    stream.next_out = buffer.data();
    stream.avail_out = available;

    const auto result = lzma_code(&stream, LZMA_FINISH);
    check_result(result);

    const auto produced = available - stream.avail_out;
    if (produced > remaining) {
      throw std::runtime_error("XZ metadata exceeds JSON size limit");
    }
    output.append(reinterpret_cast<const char *>(buffer.data()), produced);

    if (result == LZMA_STREAM_END) {
      if (stream.avail_in != 0) {
        throw std::runtime_error("XZ metadata contains trailing input");
      }
      return output;
    }
  }
}

}  // namespace px4_events
}  // namespace mavros
