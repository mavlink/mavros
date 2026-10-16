//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

#include <gtest/gtest.h>

#include <lzma.h>

#include <array>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <libevents/parse/nlohmann/json.hpp>

#include "../src/plugins/px4_events/decoder.cpp"  // NOLINT(build/include)
#include "px4_events_test_helpers.hpp"

namespace mavros
{
namespace px4_events
{

// Like the plugin tests, include the implementation to exercise private state
// without adding a runtime fault-injection API.
class DecoderTestPeer
{
public:
  static void throw_on_formatting(Decoder & decoder)
  {
    decoder.impl_->config.formatters.escape = [](const std::string &) -> std::string {
        throw std::runtime_error("injected formatting failure");
      };
  }
};

namespace
{
std::string metadata()
{
  std::ifstream file(TEST_EVENT_METADATA);
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

std::string compress(const std::string & input)
{
  std::string output(lzma_stream_buffer_bound(input.size()), '\0');
  size_t pos = 0;
  const auto result = lzma_easy_buffer_encode(
    1, LZMA_CHECK_CRC64, nullptr, reinterpret_cast<const uint8_t *>(input.data()), input.size(),
    reinterpret_cast<uint8_t *>(output.data()), &pos, output.size());
  if (result != LZMA_OK) {
    throw std::runtime_error("cannot compress test metadata");
  }
  output.resize(pos);
  return output;
}

nlohmann::json health_metadata()
{
  auto json = nlohmann::json::parse(metadata());
  auto & px4 = json["components"]["1"];
  px4["enums"]["navigation_mode_group_t"] = {
    {"type", "uint32_t"}, {"is_bitfield", true},
    {"entries", {
        {"4", {{"name", "position"}, {"description", "Position mode"}}},
        {"8", {{"name", "mission"}, {"description", "Mission mode"}}}
      }}
  };
  px4["event_groups"]["default"]["events"]["8"] = {
    {"name", "health"}, {"message", "Health check"},
    {"arguments", {
        {{"name", "modes"}, {"type", "navigation_mode_group_t"}},
        {{"name", "flags"}, {"type", "flags_t"}}
      }}
  };
  return json;
}

class Px4EventsTest : public ::testing::Test
{
protected:
  Decoder decoder;
  std::array<uint8_t, 40> arguments{};

  void SetUp() override
  {
    ASSERT_TRUE(decoder.configure("local", TEST_EVENT_METADATA).empty());
  }

  template<typename T>
  void argument(size_t offset, T value)
  {
    // The fixture encodes the MAVLink little-endian payload explicitly.
    std::array<uint8_t, sizeof(T)> bytes;
    std::memcpy(bytes.data(), &value, sizeof(T));
    const uint16_t endian = 1;
    if (*reinterpret_cast<const uint8_t *>(&endian) != 1) {
      std::reverse(bytes.begin(), bytes.end());
    }
    std::copy(bytes.begin(), bytes.end(), arguments.begin() + offset);
  }
};

TEST_F(Px4EventsTest, in_memory_load_reuses_limits_and_failure_clears_definitions)
{
  const auto json = metadata();
  EXPECT_EQ(Decoder::unpack_metadata(json, false), json);
  EXPECT_EQ(Decoder::unpack_metadata(compress(json), true), json);
  EXPECT_THROW(Decoder::unpack_metadata("invalid XZ", true), std::runtime_error);
  EXPECT_THROW(Decoder::unpack_metadata(
      std::string(Decoder::MAX_COMPRESSED_BYTES + 1, 'x'), true), std::runtime_error);
  EXPECT_THROW(Decoder::unpack_metadata(
      std::string(Decoder::MAX_JSON_BYTES + 1, 'x'), false), std::runtime_error);
  ASSERT_TRUE(decoder.load_json(json).empty());
  EXPECT_TRUE(decoder.decode(1, 6, arguments));
  EXPECT_FALSE(decoder.load_json("invalid JSON").empty());
  EXPECT_FALSE(decoder.decode(1, 6, arguments));
}

TEST_F(Px4EventsTest, full_ids_distinguish_namespaces)
{
  auto common = decoder.decode(1, 6, arguments);
  auto px4 = decoder.decode(0x01000001, 6, arguments);
  ASSERT_TRUE(common);
  ASSERT_TRUE(px4);
  EXPECT_EQ(common->text, "Common namespace");
  EXPECT_EQ(px4->text, "Count 0; height 0.00 m");
  EXPECT_FALSE(decoder.decode(0x02000001, 6, arguments));
  EXPECT_FALSE(decoder.decode(0x010000ff, 6, arguments));
}

TEST_F(Px4EventsTest, numeric_arguments_use_libevents_formatting)
{
  argument<int16_t>(0, -12);
  argument<float>(2, 3.5f);
  const auto result = decoder.decode(0x01000001, 3, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Count -12; height 3.50 m");
  EXPECT_EQ(result->severity, 3);
}

TEST_F(Px4EventsTest, enums_bitfields_and_unknown_values_retain_information)
{
  arguments[0] = 1;
  arguments[1] = 3;
  auto result = decoder.decode(0x01000002, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Mode Ready; flags GPS|IMU");
  arguments[0] = 9;
  arguments[1] = 5;
  result = decoder.decode(0x01000002, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Mode (unknown: 9); flags GPS|unknown_bits=0x04");
  arguments[1] = 0xfd;
  result = decoder.decode(0x01000002, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Mode (unknown: 9); flags GPS|unknown_bits=0xfc");
  arguments[1] = 0xff;
  result = decoder.decode(0x01000002, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Mode (unknown: 9); flags GPS|IMU|unknown_bits=0xfc");
  arguments[1] = 0;
  result = decoder.decode(0x01000002, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Mode (unknown: 9); flags 0");
}

TEST_F(Px4EventsTest, px4_all_modes_is_compact_and_other_bitfields_keep_details)
{
  ASSERT_TRUE(decoder.load_json(health_metadata().dump()).empty());
  argument<uint32_t>(0, 0xffffffff);
  arguments[4] = 0xff;
  auto result = decoder.decode(0x01000008, 4, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Health check [modes=all, flags=GPS|IMU|unknown_bits=0xfc]");

  argument<uint32_t>(0, 12);
  arguments[4] = 0;
  result = decoder.decode(0x01000008, 4, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Health check [modes=Position mode|Mission mode, flags=0]");

  argument<uint32_t>(0, 0x1004);
  result = decoder.decode(0x01000008, 4, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Health check [modes=Position mode|unknown_bits=0x00001000, flags=0]");
}

TEST_F(Px4EventsTest, all_modes_shorthand_requires_px4_namespace_and_modes_argument)
{
  auto json = health_metadata();
  json["components"]["1"]["namespace"] = "other";
  ASSERT_TRUE(decoder.load_json(json.dump()).empty());
  argument<uint32_t>(0, 0xffffffff);
  auto result = decoder.decode(0x01000008, 4, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text,
    "Health check [modes=Position mode|Mission mode|unknown_bits=0xfffffff3, flags=0]");

  json = health_metadata();
  auto & health = json["components"]["1"]["event_groups"]["default"]["events"]["8"];
  health["arguments"][0]["name"] = "other_flags";
  ASSERT_TRUE(decoder.load_json(json.dump()).empty());
  result = decoder.decode(0x01000008, 4, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text,
    "Health check [other_flags=Position mode|Mission mode|unknown_bits=0xfffffff3, flags=0]");
}

TEST_F(Px4EventsTest, wide_bitfield_combines_unknown_high_bits_and_preserves_separator)
{
  auto json = nlohmann::json::parse(metadata());
  auto & flags = json["components"]["1"]["enums"]["flags_t"];
  flags["type"] = "uint64_t";
  flags["separator"] = ", ";
  ASSERT_TRUE(decoder.load_json(json.dump()).empty());
  arguments[0] = 1;
  argument<uint64_t>(1, (uint64_t{1} << 63) | (uint64_t{1} << 32) | 1);
  const auto result = decoder.decode(0x01000002, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Mode Ready; flags GPS, unknown_bits=0x8000000100000000");
}

TEST_F(Px4EventsTest, disabled_external_severity_is_debug_with_named_protocol_arguments)
{
  arguments[0] = 1;
  argument<uint32_t>(1, 256);
  const auto result = decoder.decode(0x01000003, 0x38, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->severity, 7);
  EXPECT_EQ(result->text, "protocol_summary [version=1, checks=256]");
}

TEST_F(Px4EventsTest, developer_profile_plain_text_and_unprinted_arguments)
{
  arguments[0] = 4;
  arguments[1] = 9;
  const auto result = decoder.decode(0x01000004, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Developer 4 TEST_PARAM Guide [hidden=9]");
}

TEST_F(Px4EventsTest, escaped_placeholders_do_not_hide_argument_values)
{
  arguments[0] = 4;
  argument<int8_t>(1, -2);
  const auto result = decoder.decode(0x01000005, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Literal {1}; value -2 [first=4]");
}

TEST_F(Px4EventsTest, wide_named_arguments_are_not_narrowed)
{
  argument<int64_t>(0, -1234567890123);
  argument<uint64_t>(8, 18446744073709551615ULL);
  const auto result = decoder.decode(0x01000006, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Summary [signed=-1234567890123, unsigned=18446744073709551615]");
}

TEST_F(Px4EventsTest, severity_mapping_and_short_single_line_output)
{
  for (uint8_t severity = 0; severity < 8; ++severity) {
    const auto result = decoder.decode(0x01000007, 0x70 | severity, arguments);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->severity, severity);
    EXPECT_EQ(result->text, "Warning next line");
  }
  EXPECT_FALSE(decoder.decode(0x01000007, 9, arguments));
}

TEST_F(Px4EventsTest, disabled_and_invalid_configuration_leave_decoder_inactive)
{
  EXPECT_TRUE(decoder.configure("disabled", "").empty());
  EXPECT_FALSE(decoder.decode(1, 6, arguments));
  for (const auto & mode : {"disabled", "auto", "typo"}) {
    EXPECT_FALSE(decoder.configure(mode, "/file/that/does/not/exist").empty());
    EXPECT_FALSE(decoder.decode(1, 6, arguments));
  }
  EXPECT_TRUE(decoder.configure("auto", "").empty());
  EXPECT_FALSE(decoder.decode(1, 6, arguments));
  EXPECT_FALSE(decoder.configure("local", "").empty());
  EXPECT_FALSE(decoder.configure("local", "/file/that/does/not/exist").empty());
}

TEST_F(Px4EventsTest, ordinary_results_do_not_report_decoding_exceptions)
{
  int reports = 0;
  Decoder reporting_decoder([&reports](uint32_t, const char *) {++reports;});
  EXPECT_FALSE(reporting_decoder.decode(1, 6, arguments));
  ASSERT_TRUE(reporting_decoder.configure("local", TEST_EVENT_METADATA).empty());
  EXPECT_TRUE(reporting_decoder.decode(1, 6, arguments));
  EXPECT_FALSE(reporting_decoder.decode(0x02000001, 6, arguments));
  EXPECT_FALSE(reporting_decoder.decode(1, 9, arguments));
  EXPECT_TRUE(reporting_decoder.configure("disabled", "").empty());
  EXPECT_FALSE(reporting_decoder.decode(1, 6, arguments));
  EXPECT_EQ(reports, 0);
}

TEST_F(Px4EventsTest, decoding_exception_reports_details_and_preserves_fallback)
{
  int reports = 0;
  uint32_t reported_id = 0;
  std::string reported_reason;
  bool diagnostic_throws = false;
  Decoder reporting_decoder(
    [&](uint32_t id, const char * reason) {
      ++reports;
      reported_id = id;
      reported_reason = reason;
      if (diagnostic_throws) {
        throw std::runtime_error("injected diagnostic failure");
      }
    });
  ASSERT_TRUE(reporting_decoder.configure("local", TEST_EVENT_METADATA).empty());
  DecoderTestPeer::throw_on_formatting(reporting_decoder);

  EXPECT_FALSE(reporting_decoder.decode(0x01000007, 4, arguments));
  EXPECT_EQ(reports, 1);
  EXPECT_EQ(reported_id, 0x01000007u);
  EXPECT_EQ(reported_reason, "injected formatting failure");

  // Even an exception from diagnostics must still produce raw fallback.
  diagnostic_throws = true;
  EXPECT_FALSE(reporting_decoder.decode(0x01000007, 4, arguments));
  EXPECT_EQ(reports, 2);
}

TEST_F(Px4EventsTest, json_and_xz_load_identically_and_corrupt_xz_is_rejected)
{
  TemporaryMetadata xz(compress(metadata()), ".json.xz");
  ASSERT_TRUE(decoder.configure("local", xz.path).empty());
  const auto result = decoder.decode(1, 6, arguments);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->text, "Common namespace");
  TemporaryMetadata corrupt("not an XZ file", ".json.xz");
  EXPECT_FALSE(decoder.configure("local", corrupt.path).empty());
  EXPECT_FALSE(decoder.decode(1, 6, arguments));
  auto truncated = compress(metadata());
  truncated.resize(truncated.size() - 8);
  TemporaryMetadata partial(truncated, ".json.xz");
  EXPECT_FALSE(decoder.configure("local", partial.path).empty());
}

// These helpers only mutate known-valid encoder output, preserving outer CRCs
// when testing whether the decoder rejects a dishonest Index.
size_t xz_index_start(const std::string & bytes)
{
  const auto footer = bytes.size() - 12;
  uint32_t backward_size = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    backward_size |= static_cast<uint32_t>(static_cast<uint8_t>(bytes[footer + 4 + i])) << (8 * i);
  }
  return footer - (static_cast<size_t>(backward_size) + 1) * 4;
}

void rewrite_xz_index_crc(std::string & bytes, size_t index)
{
  const auto crc_pos = bytes.size() - 16;
  const auto crc = lzma_crc32(
    reinterpret_cast<const uint8_t *>(bytes.data() + index), crc_pos - index, 0);
  for (unsigned int i = 0; i < 4; ++i) {
    bytes[crc_pos + i] = static_cast<char>((crc >> (8 * i)) & 0xff);
  }
}

TEST(Px4XzTest, exact_output_limit_succeeds_and_smaller_limit_rejects)
{
  const std::string plain(32, 'z');
  const auto bytes = compress(plain);
  EXPECT_EQ(unpack_xz_metadata(bytes, plain.size()), plain);
  EXPECT_THROW(unpack_xz_metadata(bytes, plain.size() - 1), std::runtime_error);
}

TEST(Px4XzTest, truncated_corrupt_and_extra_input_are_rejected)
{
  const std::string plain(32, 'z');
  const auto bytes = compress(plain);
  for (size_t size = 0; size < bytes.size(); ++size) {
    SCOPED_TRACE(size);
    EXPECT_THROW(unpack_xz_metadata(bytes.substr(0, size), plain.size()), std::runtime_error);
  }

  const auto index = xz_index_start(bytes);
  // Header CRC, block/check data, Index CRC, and Footer CRC respectively.
  for (const auto pos : {size_t{8}, index - 8, bytes.size() - 16, bytes.size() - 12}) {
    SCOPED_TRACE(pos);
    auto corrupt = bytes;
    corrupt[pos] ^= 1;
    EXPECT_THROW(unpack_xz_metadata(corrupt, plain.size()), std::runtime_error);
  }
  EXPECT_THROW(unpack_xz_metadata(bytes + std::string(4, '\0'), plain.size()), std::runtime_error);
  EXPECT_THROW(unpack_xz_metadata(bytes + bytes, plain.size() * 2), std::runtime_error);
}

TEST(Px4XzTest, dishonest_output_sizes_with_valid_index_crc_are_rejected)
{
  const std::string plain(32, 'z');
  const auto bytes = compress(plain);
  const auto index = xz_index_start(bytes);
  // This small fixture has one record and one-byte size fields.
  ASSERT_EQ(static_cast<uint8_t>(bytes[index + 1]), 1U);
  ASSERT_LT(static_cast<uint8_t>(bytes[index + 2]), 128U);
  ASSERT_EQ(static_cast<uint8_t>(bytes[index + 3]), plain.size());
  for (const auto declared : {31, 33}) {
    SCOPED_TRACE(declared);
    auto corrupt = bytes;
    corrupt[index + 3] = static_cast<char>(declared);
    rewrite_xz_index_crc(corrupt, index);
    // Both claims fit the limit: rejection must validate the actual stream,
    // rather than merely noticing an excessive declared allocation.
    EXPECT_THROW(unpack_xz_metadata(corrupt, 64), std::runtime_error);
  }
  EXPECT_EQ(unpack_xz_metadata(bytes, 64), plain);
}

TEST(Px4XzTest, invalid_record_count_and_noncanonical_integer_are_rejected)
{
  const auto bytes = compress(std::string(32, 'z'));
  const auto index = xz_index_start(bytes);
  ASSERT_EQ(static_cast<uint8_t>(bytes[index + 1]), 1U);

  auto corrupt = bytes;
  corrupt[index + 1] = 127;
  rewrite_xz_index_crc(corrupt, index);
  EXPECT_THROW(unpack_xz_metadata(corrupt, 64), std::runtime_error);

  corrupt = bytes;
  corrupt[index + 1] = static_cast<char>(0x81);
  corrupt[index + 2] = 0;
  rewrite_xz_index_crc(corrupt, index);
  EXPECT_THROW(unpack_xz_metadata(corrupt, 64), std::runtime_error);
}

TEST_F(Px4EventsTest, malformed_metadata_cannot_activate_partial_definitions)
{
  for (const auto & text : {"not json", "{\"version\":2}", "{\"version\":2,\"components\":{}}"}) {
    TemporaryMetadata file(text);
    EXPECT_FALSE(decoder.configure("local", file.path).empty());
    EXPECT_FALSE(decoder.decode(1, 6, arguments));
  }
  auto json = nlohmann::json::parse(metadata());
  json["version"] = 3;
  TemporaryMetadata future(json.dump());
  EXPECT_FALSE(decoder.configure("local", future.path).empty());
  json["version"] = 2;
  json["components"]["1"]["event_groups"]["default"]["events"]["1"]["arguments"][0]["type"] =
    "invalid";
  TemporaryMetadata invalid(json.dump());
  EXPECT_FALSE(decoder.configure("local", invalid.path).empty());
  EXPECT_FALSE(decoder.decode(1, 6, arguments));
}

TEST_F(Px4EventsTest, oversized_argument_payload_and_duplicate_full_id_are_rejected)
{
  auto json = nlohmann::json::parse(metadata());
  auto & events = json["components"]["1"]["event_groups"]["default"]["events"];
  events["6"]["arguments"] = nlohmann::json::array();
  for (int i = 0; i < 6; ++i) {
    events["6"]["arguments"].push_back({{"name", "wide"}, {"type", "uint64_t"}});
  }
  TemporaryMetadata oversized(json.dump());
  EXPECT_FALSE(decoder.configure("local", oversized.path).empty());
  json = nlohmann::json::parse(metadata());
  json["components"]["1"]["event_groups"]["duplicate"]["events"]["7"] =
  {{"name", "duplicate"}, {"message", "Conflicting definition"}};
  TemporaryMetadata duplicate(json.dump());
  EXPECT_FALSE(decoder.configure("local", duplicate.path).empty());
}

TEST_F(Px4EventsTest, file_and_decompression_limits_are_enforced)
{
  const std::string oversized(Decoder::MAX_JSON_BYTES + 1, ' ');
  TemporaryMetadata json(oversized);
  EXPECT_FALSE(decoder.configure("local", json.path).empty());
  TemporaryMetadata xz(compress(oversized), ".json.xz");
  EXPECT_FALSE(decoder.configure("local", xz.path).empty());
  TemporaryMetadata large_xz(std::string(Decoder::MAX_COMPRESSED_BYTES + 1, 'x'), ".json.xz");
  EXPECT_FALSE(decoder.configure("local", large_xz.path).empty());
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros
