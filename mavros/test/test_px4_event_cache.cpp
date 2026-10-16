// mavros
// Copyright 2026 Sean Hickey, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md

/**
 * @brief Test persistent metadata identity, atomic publication, and eviction
 * @file test_px4_event_cache.cpp
 * @author Sean Hickey
 */

#include <gtest/gtest.h>
#include <lzma.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <libevents/parse/nlohmann/json.hpp>

#include "../src/plugins/px4_events/cache.hpp"

namespace mavros
{
namespace px4_events
{
namespace
{
namespace fs = std::filesystem;

class MetadataCacheTest : public ::testing::Test
{
protected:
  fs::path root;
  fs::path directory;
  const std::string events =
    R"({"version":2,"components":{"0":{
    "namespace":"common","event_groups":{"default":{"events":{
    "1":{"name":"test","message":"Decoded event"}}}}}}})";
  const std::string general =
    R"({"version":1,"metadataTypes":[{"type":4,"uri":"mftp://events.json","fileCrc":123}]})";

  void SetUp() override
  {
    char pattern[] = "/tmp/mavros-event-cache-XXXXXX";
    const auto result = mkdtemp(pattern);
    ASSERT_NE(result, nullptr);
    root = result;
    directory = root / "cache";
  }

  void TearDown() override
  {
    std::error_code error;
    fs::remove_all(root, error);
  }

  MetadataFile event_file(char suffix = 'a') const
  {
    return metadata_file(std::string("mftp://events-") + suffix + ".json", metadata_crc32(events));
  }

  std::vector<fs::path> entries() const
  {
    std::vector<fs::path> paths;
    for (const auto & entry : fs::directory_iterator(directory)) {
      if (entry.path().extension() == ".cache") {
        paths.push_back(entry.path());
      }
    }
    return paths;
  }

  static std::string read(const fs::path & path)
  {
    std::ifstream stream(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }

  static void write(const fs::path & path, const std::string & bytes)
  {
    std::ofstream stream(path, std::ios::binary);
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(stream.good());
  }

  static std::string compress(const std::string & input)
  {
    std::string output(lzma_stream_buffer_bound(input.size()), '\0');
    size_t end = 0;
    if (lzma_easy_buffer_encode(
        1, LZMA_CHECK_CRC64, nullptr, reinterpret_cast<const uint8_t *>(input.data()), input.size(),
        reinterpret_cast<uint8_t *>(output.data()), &end, output.size()) != LZMA_OK)
    {
      throw std::runtime_error("cannot compress cache fixture");
    }
    output.resize(end);
    return output;
  }
};

TEST_F(MetadataCacheTest, missing_cache_does_not_create_directory_and_limits_must_be_positive)
{
  MetadataCache cache(directory);
  EXPECT_FALSE(cache.load(event_file(), MetadataRole::Events));
  EXPECT_FALSE(fs::exists(directory));
  EXPECT_THROW(MetadataCache(directory, 0), std::invalid_argument);
  EXPECT_THROW(MetadataCache(directory, -1), std::invalid_argument);
}

TEST_F(MetadataCacheTest, general_and_event_files_round_trip_with_role_specific_xz_checksums)
{
  MetadataCache cache(directory);
  for (const bool compressed : {false, true}) {
    const auto suffix = compressed ? ".json.xz" : ".json";
    const auto event_bytes = compressed ? compress(events) : events;
    const auto general_bytes = compressed ? compress(general) : general;
    const auto event = metadata_file(std::string("mftp://events") + suffix,
            metadata_crc32(event_bytes));
    const auto descriptor = metadata_file(std::string("mftp://general") + suffix,
            metadata_crc32(general));
    ASSERT_TRUE(cache.save(event, MetadataRole::Events, event_bytes).empty());
    ASSERT_TRUE(cache.save(descriptor, MetadataRole::General, general_bytes).empty());
    EXPECT_EQ(cache.load(event, MetadataRole::Events), event_bytes);
    EXPECT_EQ(cache.load(descriptor, MetadataRole::General), general_bytes);
    EXPECT_FALSE(cache.load(event, MetadataRole::General));
    auto changed = event;
    ++changed.crc;
    EXPECT_FALSE(cache.load(changed, MetadataRole::Events));
    changed = event;
    changed.uri += "-different";
    EXPECT_FALSE(cache.load(changed, MetadataRole::Events));
  }
}

TEST_F(MetadataCacheTest, corrupt_truncated_and_forged_identity_entries_are_misses)
{
  MetadataCache cache(directory);
  const auto file = event_file();
  ASSERT_TRUE(cache.save(file, MetadataRole::Events, events).empty());
  ASSERT_EQ(entries().size(), 1U);
  const auto path = entries().front();
  EXPECT_EQ(
    fs::status(path).permissions() & (fs::perms::group_all | fs::perms::others_all),
    fs::perms::none);
  const auto original = read(path);
  auto corrupt = original;
  corrupt.back() ^= 1;
  write(path, corrupt);
  EXPECT_FALSE(cache.load(file, MetadataRole::Events));
  write(path, original.substr(0, original.size() - 1));
  EXPECT_FALSE(cache.load(file, MetadataRole::Events));

  const auto split = original.find('\n');
  auto header = nlohmann::json::parse(original.substr(0, split));
  header["uri"] = "mftp://another-vehicle.json";
  write(path, header.dump() + original.substr(split));
  EXPECT_FALSE(cache.load(file, MetadataRole::Events));
  write(path, std::string(4097, 'x') + '\n' + events);
  EXPECT_FALSE(cache.load(file, MetadataRole::Events));
  // Reject an oversized disk entry before allocating or parsing its contents.
  fs::resize_file(path, 8 * 1024 * 1024 + 4098);
  EXPECT_FALSE(cache.load(file, MetadataRole::Events));
}

TEST_F(MetadataCacheTest, invalid_saves_preserve_good_entry_and_oversized_entries_are_not_written)
{
  MetadataCache cache(directory);
  const auto file = event_file();
  ASSERT_TRUE(cache.save(file, MetadataRole::Events, events).empty());
  EXPECT_FALSE(cache.save(file, MetadataRole::Events, "incomplete").empty());
  EXPECT_EQ(cache.load(file, MetadataRole::Events), events);
  const std::string unsupported = R"({"version":99,"components":{}})";
  const auto bad = metadata_file("mftp://bad.json", metadata_crc32(unsupported));
  EXPECT_FALSE(cache.save(bad, MetadataRole::Events, unsupported).empty());
  EXPECT_EQ(entries().size(), 1U);
  MetadataCache small(directory, 1);
  EXPECT_FALSE(small.save(file, MetadataRole::Events, events).empty());
  EXPECT_FALSE(small.load(file, MetadataRole::Events));
  EXPECT_EQ(cache.load(file, MetadataRole::Events), events);
}

TEST_F(MetadataCacheTest, partial_temporary_files_are_ignored_and_failed_publication_cleans_up)
{
  MetadataCache cache(directory);
  fs::create_directories(directory);
  const auto partial = directory / ".px4-events-active";
  write(partial, "partial download");
  EXPECT_FALSE(cache.load(event_file(), MetadataRole::Events));
  ASSERT_TRUE(cache.save(event_file(), MetadataRole::Events, events).empty());
  ASSERT_EQ(entries().size(), 1U);
  const auto final = entries().front();
  fs::remove(final);
  fs::create_directory(final);  // Force rename to fail after writing the temporary file.
  EXPECT_FALSE(cache.save(event_file(), MetadataRole::Events, events).empty());
  EXPECT_TRUE(fs::exists(partial));
  EXPECT_EQ(std::distance(fs::directory_iterator(directory), fs::directory_iterator()), 2);
  const auto blocked = root / "not-a-directory";
  write(blocked, "existing file");
  MetadataCache unavailable(blocked);
  EXPECT_FALSE(unavailable.save(event_file(), MetadataRole::Events, events).empty());
}

TEST_F(MetadataCacheTest, eviction_removes_oldest_saved_entry_and_preserves_unrelated_files)
{
  MetadataCache cache(directory);
  ASSERT_TRUE(cache.save(event_file('a'), MetadataRole::Events, events).empty());
  const auto oldest = entries().front();
  fs::last_write_time(oldest, fs::file_time_type::clock::now() - std::chrono::hours(2));
  ASSERT_TRUE(cache.save(event_file('b'), MetadataRole::Events, events).empty());
  const auto one_entry_size = fs::file_size(oldest);
  write(directory / "unrelated.txt", "do not remove");
  write(directory / ".px4-events-active", "another writer");
  MetadataCache limited(directory, static_cast<int64_t>(one_entry_size * 2));
  ASSERT_TRUE(limited.save(event_file('c'), MetadataRole::Events, events).empty());
  EXPECT_FALSE(limited.load(event_file('a'), MetadataRole::Events));
  EXPECT_EQ(limited.load(event_file('b'), MetadataRole::Events), events);
  EXPECT_EQ(limited.load(event_file('c'), MetadataRole::Events), events);
  EXPECT_EQ(entries().size(), 2U);
  EXPECT_EQ(read(directory / "unrelated.txt"), "do not remove");
  EXPECT_EQ(read(directory / ".px4-events-active"), "another writer");
}

TEST_F(MetadataCacheTest, separate_cache_instances_publish_without_exposing_partial_files)
{
  MetadataCache writer(directory);
  MetadataCache reader(directory);
  const auto file = event_file();
  ASSERT_TRUE(writer.save(file, MetadataRole::Events, events).empty());
  std::atomic<bool> done{false};
  std::atomic<bool> failed{false};
  std::thread reading([&]() {
      do {
        if (reader.load(file, MetadataRole::Events) != events) {
          failed = true;
        }
      } while (!done.load());
    });
  for (int i = 0; i < 20; ++i) {
    if (!writer.save(file, MetadataRole::Events, events).empty()) {
      failed = true;
    }
  }
  done = true;
  reading.join();
  EXPECT_FALSE(failed.load());
  EXPECT_EQ(entries().size(), 1U);
}

}  // namespace
}  // namespace px4_events
}  // namespace mavros
