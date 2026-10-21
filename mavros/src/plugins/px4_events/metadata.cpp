/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief PX4 event metadata parsing and checksum calculation
 * @file metadata.cpp
 * @author Sean Hickey
 */

#include "metadata.hpp"

#include <limits>
#include <optional>
#include <stdexcept>

#include <libevents/parse/nlohmann/json.hpp>

#include "decoder.hpp"

namespace mavros
{
namespace px4_events
{
namespace
{
uint32_t unsigned_field(const nlohmann::json & value)
{
  if (!value.is_number_integer() ||
    (!value.is_number_unsigned() && value.get<int64_t>() < 0))
  {
    throw std::runtime_error("metadata field must be an unsigned 32-bit integer");
  }
  const auto number = value.get<uint64_t>();
  if (number > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("metadata field exceeds unsigned 32-bit range");
  }
  return static_cast<uint32_t>(number);
}
}  // namespace

MetadataFile metadata_file(const std::string & uri, uint32_t crc)
{
  constexpr std::string_view prefix = "mftp://";
  if (uri.compare(0, prefix.size(), prefix) != 0) {
    throw std::runtime_error("only MAVLink FTP metadata URIs are supported");
  }
  auto path = uri.substr(prefix.size());
  if (!path.empty() && path.front() == '/') {
    path.erase(0, 1);
  }
  if (path.empty() || path.find_first_of("?#%\\") != std::string::npos) {
    throw std::runtime_error("invalid MAVLink FTP metadata path");
  }
  for (const auto ch : path) {
    if (static_cast<unsigned char>(ch) <= 32 || static_cast<unsigned char>(ch) == 127) {
      throw std::runtime_error("invalid MAVLink FTP metadata path");
    }
  }
  size_t start = 0;
  do {
    const auto end = path.find('/', start);
    const auto part = path.substr(start, end == std::string::npos ? end : end - start);
    if (part.empty() || part == "." || part == "..") {
      throw std::runtime_error("invalid MAVLink FTP metadata path");
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  } while (true);
  return {uri, path, crc};
}

MetadataFile event_metadata_file(const std::string & general_json)
{
  if (general_json.size() > Decoder::MAX_JSON_BYTES) {
    throw std::runtime_error("general metadata exceeds JSON size limit");
  }
  const auto general = nlohmann::json::parse(general_json);
  if (unsigned_field(general.at("version")) != 1) {
    throw std::runtime_error("unsupported general metadata version");
  }
  const auto & types = general.at("metadataTypes");
  if (!types.is_array()) {
    throw std::runtime_error("metadataTypes must be an array");
  }
  std::optional<MetadataFile> events;
  for (const auto & entry : types) {
    // COMP_METADATA_TYPE_EVENTS = 4. Other metadata types are outside this feature.
    if (unsigned_field(entry.at("type")) != 4) {
      continue;
    }
    if (events) {
      throw std::runtime_error("duplicate event metadata entries");
    }
    events = metadata_file(
      entry.at("uri").get<std::string>(), unsigned_field(entry.at("fileCrc")));
  }
  if (!events) {
    throw std::runtime_error("general metadata contains no event definitions");
  }
  return *events;
}

uint32_t metadata_crc32(std::string_view bytes) noexcept
{
  uint32_t crc = 0;
  for (const auto ch : bytes) {
    crc ^= static_cast<uint8_t>(ch);
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320U : 0U);
    }
  }
  return crc;
}

}  // namespace px4_events
}  // namespace mavros
