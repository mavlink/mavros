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

#include <xz_embedded/xz.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>

namespace mavros
{
namespace px4_events
{
namespace
{

uint8_t byte_at(std::string_view bytes, size_t pos)
{
  return static_cast<uint8_t>(bytes[pos]);
}

// All callers first check that the complete four-byte field is present.
uint32_t little_endian32(std::string_view bytes, size_t pos)
{
  return static_cast<uint32_t>(byte_at(bytes, pos)) |
         (static_cast<uint32_t>(byte_at(bytes, pos + 1)) << 8) |
         (static_cast<uint32_t>(byte_at(bytes, pos + 2)) << 16) |
         (static_cast<uint32_t>(byte_at(bytes, pos + 3)) << 24);
}

void check_crc(std::string_view bytes, size_t start, size_t size)
{
  const auto crc = xz_crc32(
    reinterpret_cast<const uint8_t *>(bytes.data() + start), size, 0);
  if (crc != little_endian32(bytes, start + size)) {
    throw std::runtime_error("invalid XZ metadata header or index checksum");
  }
}

uint64_t index_integer(std::string_view bytes, size_t & pos, size_t end)
{
  uint64_t value = 0;
  for (unsigned int i = 0; i < 9; ++i) {
    if (pos == end) {
      throw std::runtime_error("truncated XZ metadata index");
    }
    const auto next = byte_at(bytes, pos++);
    value |= static_cast<uint64_t>(next & 0x7f) << (7 * i);
    if ((next & 0x80) == 0) {
      if (i != 0 && next == 0) {
        throw std::runtime_error("noncanonical XZ metadata index integer");
      }
      return value;
    }
  }
  throw std::runtime_error("invalid XZ metadata index integer");
}

size_t declared_output_size(std::string_view bytes, size_t limit)
{
  // Stream Header (12), minimum Index (8), Stream Footer (12).
  if (bytes.size() < 32 || bytes.size() % 4 != 0 ||
    bytes.substr(0, 6) != std::string_view("\xfd" "7zXZ\0", 6) ||
    bytes.substr(bytes.size() - 2) != "YZ")
  {
    throw std::runtime_error("invalid or truncated XZ metadata stream");
  }
  const size_t footer = bytes.size() - 12;
  check_crc(bytes, 6, 2);
  // Unlike the other CRC fields, the footer's CRC precedes its data.
  if (xz_crc32(reinterpret_cast<const uint8_t *>(bytes.data() + footer + 4), 6, 0) !=
    little_endian32(bytes, footer))
  {
    throw std::runtime_error("invalid XZ metadata footer checksum");
  }
  if (byte_at(bytes, 6) != 0 || (byte_at(bytes, 7) & 0xf0) != 0 ||
    bytes.substr(6, 2) != bytes.substr(footer + 8, 2))
  {
    throw std::runtime_error("invalid XZ metadata stream flags");
  }

  const uint64_t index_size = (static_cast<uint64_t>(little_endian32(bytes, footer + 4)) + 1) * 4;
  if (index_size < 8 || index_size > footer - 12) {
    throw std::runtime_error("invalid XZ metadata index size");
  }
  const size_t index_start = footer - static_cast<size_t>(index_size);
  const size_t index_end = footer - 4;
  check_crc(bytes, index_start, static_cast<size_t>(index_size) - 4);
  size_t pos = index_start;
  if (byte_at(bytes, pos++) != 0) {
    throw std::runtime_error("invalid XZ metadata index indicator");
  }
  const auto records = index_integer(bytes, pos, index_end);
  // Each record needs at least two one-byte integers. Bound the loop by input.
  if (records > (index_end - pos) / 2) {
    throw std::runtime_error("invalid XZ metadata index record count");
  }

  size_t total = 0;
  size_t block_end = 12;
  for (uint64_t i = 0; i < records; ++i) {
    const auto unpadded = index_integer(bytes, pos, index_end);
    const auto uncompressed = index_integer(bytes, pos, index_end);
    if (unpadded < 8 || unpadded > index_start - block_end) {
      throw std::runtime_error("invalid XZ metadata block size");
    }
    const uint64_t padded = (unpadded + 3) & ~uint64_t{3};
    if (padded > index_start - block_end) {
      throw std::runtime_error("invalid XZ metadata block padding");
    }
    block_end += static_cast<size_t>(padded);
    if (uncompressed > limit - total) {
      throw std::runtime_error("XZ metadata exceeds JSON size limit");
    }
    total += static_cast<size_t>(uncompressed);
  }
  if (block_end != index_start || index_end - pos > 3) {
    throw std::runtime_error("invalid XZ metadata index layout");
  }
  while (pos < index_end) {
    if (byte_at(bytes, pos++) != 0) {
      throw std::runtime_error("invalid XZ metadata index padding");
    }
  }
  return total;
}

}  // namespace

std::string unpack_xz_metadata(std::string_view bytes, size_t output_limit)
{
  static std::once_flag crc_initialized;
  std::call_once(crc_initialized, []() {
      xz_crc32_init();
      xz_crc64_init();
    });

  const auto size = declared_output_size(bytes, output_limit);
  std::string output(size, '\0');
  std::unique_ptr<xz_dec, decltype(& xz_dec_end)> decoder(xz_dec_init(XZ_SINGLE, 0), xz_dec_end);
  if (!decoder) {
    throw std::runtime_error("cannot allocate XZ metadata decoder");
  }
  xz_buf buffer{};
  buffer.in = reinterpret_cast<const uint8_t *>(bytes.data());
  buffer.in_size = bytes.size();
  buffer.out = reinterpret_cast<uint8_t *>(output.data());
  buffer.out_size = output.size();
  const auto result = xz_dec_run(decoder.get(), &buffer);
  if (result != XZ_STREAM_END) {
    throw std::runtime_error("invalid or unsupported XZ metadata, or output exceeds declared size");
  }
  if (buffer.in_pos != bytes.size() || buffer.out_pos != size) {
    throw std::runtime_error("XZ metadata input or output size mismatch");
  }
  return output;
}

}  // namespace px4_events
}  // namespace mavros
