//
// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * @brief Test PX4 event metadata parsing and checksums
 * @file test_px4_event_metadata.cpp
 * @author Sean Hickey
 */

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include <libevents/parse/nlohmann/json.hpp>

#include "../src/plugins/px4_events/metadata.hpp"

namespace mavros
{
namespace px4_events
{
namespace
{
nlohmann::json general_metadata()
{
  return {
    {"version", 1},
    {"metadataTypes", {
        {{"type", 1}, {"uri", "https://example.com/parameters.json.xz"}},
        {{"type", 4}, {"uri", "mftp://etc/extras/all_events.json.xz"},
          {"fileCrc", 123456789U}, {"uriFallback", "https://example.com/events.json.xz"}}
      }}
  };
}

TEST(Px4EventMetadata, ParsesPx4EventEntry)
{
  const auto file = event_metadata_file(general_metadata().dump());
  EXPECT_EQ(file.uri, "mftp://etc/extras/all_events.json.xz");
  EXPECT_EQ(file.ftp_path, "etc/extras/all_events.json.xz");
  EXPECT_EQ(file.crc, 123456789U);
}

TEST(Px4EventMetadata, AcceptsHostlessFtpAndZeroChecksum)
{
  const auto file = metadata_file("mftp:///etc/extras/component_general.json.xz", 0);
  EXPECT_EQ(file.uri, "mftp:///etc/extras/component_general.json.xz");
  EXPECT_EQ(file.ftp_path, "etc/extras/component_general.json.xz");
  EXPECT_EQ(file.crc, 0U);
}

TEST(Px4EventMetadata, RejectsUnsupportedTransportWithoutUsingFallback)
{
  auto general = general_metadata();
  auto & entry = general["metadataTypes"][1];
  entry["uri"] = "https://example.com/events.json.xz";
  entry["uriFallback"] = "mftp://etc/extras/all_events.json.xz";
  EXPECT_THROW(event_metadata_file(general.dump()), std::runtime_error);
  EXPECT_THROW(metadata_file("file:///tmp/events.json", 1), std::runtime_error);
}

TEST(Px4EventMetadata, RejectsInvalidFtpPaths)
{
  for (const std::string uri : {"mftp://", "mftp:///", "mftp:////etc/events.json",
      "mftp://etc/../events.json", "mftp://etc/./events.json", "mftp://etc//events.json",
      "mftp://etc/", "mftp://etc/events.json?query", "mftp://etc/events.json#fragment",
      "mftp://etc/events%20file.json", "mftp://etc/events file.json"})
  {
    EXPECT_THROW(metadata_file(uri, 1), std::runtime_error) << uri;
  }
  EXPECT_THROW(metadata_file(std::string("mftp://etc/a\0b", 14), 1), std::runtime_error);
}

TEST(Px4EventMetadata, RejectsMissingOrDuplicateEventEntries)
{
  auto general = general_metadata();
  general["metadataTypes"].erase(1);
  EXPECT_THROW(event_metadata_file(general.dump()), std::runtime_error);
  general = general_metadata();
  const auto duplicate = general["metadataTypes"][1];
  general["metadataTypes"].push_back(duplicate);
  EXPECT_THROW(event_metadata_file(general.dump()), std::runtime_error);
}

TEST(Px4EventMetadata, RejectsMalformedStructureAndUnsupportedVersion)
{
  EXPECT_THROW(event_metadata_file("{"), nlohmann::json::exception);
  auto general = general_metadata();
  general["version"] = 2;
  EXPECT_THROW(event_metadata_file(general.dump()), std::runtime_error);
  general = general_metadata();
  general["metadataTypes"] = nlohmann::json::object();
  EXPECT_THROW(event_metadata_file(general.dump()), std::runtime_error);
  general = general_metadata();
  general["metadataTypes"][1]["uri"] = 123;
  EXPECT_THROW(event_metadata_file(general.dump()), nlohmann::json::exception);
  general = general_metadata();
  general["metadataTypes"][1].erase("fileCrc");
  EXPECT_THROW(event_metadata_file(general.dump()), nlohmann::json::exception);
}

TEST(Px4EventMetadata, RejectsInvalidChecksumTypesAndRanges)
{
  for (const nlohmann::json & value : {
          nlohmann::json(-1), nlohmann::json(4294967296ULL), nlohmann::json(1.5),
          nlohmann::json("123"), nlohmann::json(true)})
  {
    auto general = general_metadata();
    general["metadataTypes"][1]["fileCrc"] = value;
    EXPECT_THROW(event_metadata_file(general.dump()), std::runtime_error);
  }
  auto general = general_metadata();
  general["metadataTypes"][1]["fileCrc"] = 4294967295ULL;
  EXPECT_EQ(event_metadata_file(general.dump()).crc, 0xffffffffU);
}

TEST(Px4EventMetadata, MatchesMavlinkCrcKnownVectors)
{
  EXPECT_EQ(metadata_crc32(""), 0U);
  EXPECT_EQ(metadata_crc32("123456789"), 0x2dfd2d88U);
  // Binary bytes, including NUL and high bits, must not be treated as a C string.
  EXPECT_EQ(metadata_crc32(std::string("\0\x80\xff", 3)), 0x168177c6U);
  EXPECT_NE(metadata_crc32("123456789"), metadata_crc32("123456788"));
}

TEST(Px4EventMetadata, ComparesEventFileBytesAgainstAdvertisedChecksum)
{
  auto general = general_metadata();
  general["metadataTypes"][1]["fileCrc"] = 0x2dfd2d88U;
  const auto file = event_metadata_file(general.dump());
  EXPECT_EQ(metadata_crc32("123456789"), file.crc);
  EXPECT_NE(metadata_crc32("corrupt"), file.crc);
}

TEST(Px4EventMetadata, RejectsOversizedGeneralJsonBeforeParsing)
{
  EXPECT_THROW(event_metadata_file(std::string(8 * 1024 * 1024 + 1, ' ')), std::runtime_error);
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros
