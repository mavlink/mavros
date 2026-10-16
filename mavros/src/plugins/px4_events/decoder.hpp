// Copyright 2026 Sean Hickey, All rights reserved.
// This file is part of MAVROS and subject to the top-level LICENSE.md.

#ifndef PLUGINS__PX4_EVENTS__DECODER_HPP_
#define PLUGINS__PX4_EVENTS__DECODER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace mavros
{
namespace px4_events
{

struct DecodedEvent
{
  uint8_t severity;
  std::string text;
};

/** Local metadata adapter; libevents owns argument decoding and formatting. */
class Decoder
{
public:
  static constexpr size_t MAX_COMPRESSED_BYTES = 1024 * 1024;
  static constexpr size_t MAX_JSON_BYTES = 8 * 1024 * 1024;

  using ErrorCallback = std::function<void (uint32_t, const char *)>;

  explicit Decoder(ErrorCallback report_error = {});
  ~Decoder();

  // Called at startup. An error leaves decoding inactive; disabled mode does no I/O.
  std::string configure(const std::string & mode, const std::string & path);
  // Shared bounded JSON/XZ loading for local files and downloaded metadata.
  static std::string unpack_metadata(std::string_view bytes, bool compressed);
  // A failed load leaves decoding inactive; no partially parsed definitions are exposed.
  std::string load_json(const std::string & json);

  std::optional<DecodedEvent> decode(
    uint32_t full_id, uint8_t log_levels, const std::array<uint8_t, 40> & arguments) const;

private:
  friend class DecoderTestPeer;

  struct Impl;
  std::unique_ptr<Impl> impl_;
  ErrorCallback report_error_;

  void report_failure(uint32_t full_id, const char * reason) const noexcept;
};

}  // namespace px4_events
}  // namespace mavros

#endif  // PLUGINS__PX4_EVENTS__DECODER_HPP_
