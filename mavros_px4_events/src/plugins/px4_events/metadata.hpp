/*
 * Copyright 2026 Sean Hickey.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief PX4 event metadata locations and checksum helpers
 * @file metadata.hpp
 * @author Sean Hickey
 */

#ifndef PLUGINS__PX4_EVENTS__METADATA_HPP_
#define PLUGINS__PX4_EVENTS__METADATA_HPP_

#include <cstdint>
#include <string>
#include <string_view>

namespace mavros
{
namespace px4_events
{

struct MetadataFile
{
  std::string uri;
  std::string ftp_path;
  uint32_t crc;
};

/**
 * Accept PX4's mftp://path and hostless mftp:///path forms.
 * Throw on invalid input.
 */
MetadataFile metadata_file(const std::string & uri, uint32_t crc);

/** Read the single EVENTS entry from general JSON; throw if unusable or ambiguous. */
MetadataFile event_metadata_file(const std::string & general_json);

/** MAVLink CRC32: reflected polynomial 0xEDB88320, initial zero, no final XOR.
 * PX4's general advertisement covers uncompressed JSON, while an event entry
 * covers the referenced file bytes (compressed bytes for a .xz file).
 * Callers must supply the appropriate bytes, without JSON reserialization.
 */
uint32_t metadata_crc32(std::string_view bytes) noexcept;

}  // namespace px4_events
}  // namespace mavros

#endif  // PLUGINS__PX4_EVENTS__METADATA_HPP_
