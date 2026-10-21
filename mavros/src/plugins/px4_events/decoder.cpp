// Copyright 2026 Sean Hickey, All rights reserved.
// This file is part of MAVROS and subject to the top-level LICENSE.md.

#include "decoder.hpp"

#include <libevents/parse/parser.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <libevents/parse/nlohmann/json.hpp>

#include "xz_metadata.hpp"

namespace mavros
{
namespace px4_events
{
namespace
{
using events::parser::BaseType;
using events::parser::Config;
using events::parser::EnumDefinition;
using events::parser::EventDefinition;
using events::parser::ParsedEvent;
using events::parser::Parser;

std::string single_line(std::string text)
{
  for (auto & ch : text) {
    if (static_cast<unsigned char>(ch) < 32 || ch == 127) {
      ch = ' ';
    }
  }
  return text;
}

std::string read_metadata(const std::string & path)
{
  const bool compressed = path.size() >= 3 && path.substr(path.size() - 3) == ".xz";
  const size_t limit = compressed ? Decoder::MAX_COMPRESSED_BYTES : Decoder::MAX_JSON_BYTES;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("cannot open metadata file: " + path);
  }

  std::vector<uint8_t> input(limit + 1);
  file.read(reinterpret_cast<char *>(input.data()), static_cast<std::streamsize>(input.size()));
  const auto size = static_cast<size_t>(file.gcount());
  if (file.bad()) {
    throw std::runtime_error("cannot read metadata file");
  }
  if (size > limit) {
    throw std::runtime_error("metadata file exceeds size limit");
  }
  return Decoder::unpack_metadata(
    std::string_view(reinterpret_cast<const char *>(input.data()), size), compressed);
}

uint32_t metadata_id(const std::string & text, uint32_t maximum)
{
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::runtime_error("invalid metadata ID");
  }
  size_t end = 0;
  const auto value = std::stoull(text, &end);
  if (end != text.size() || value > maximum) {
    throw std::runtime_error("metadata ID is out of range");
  }
  return static_cast<uint32_t>(value);
}

std::string escape_literal(const std::string & text)
{
  std::string result;
  for (const auto ch : single_line(text)) {
    if (ch == '\\' || ch == '<' || ch == '{') {
      result += '\\';
    }
    result += ch;
  }
  return result;
}

void validate_message_precision(const std::string & message)
{
  constexpr int MAX_PRECISION = 16;
  for (size_t pos = 0; pos < message.size(); ++pos) {
    if (message[pos] == '\\') {
      ++pos;
    } else if (message[pos] == '{') {
      if (message.find('}', pos) == std::string::npos) {
        break;
      }
      // Match libevents' base-zero numeric syntax without narrowing the index
      // or rendering a float. Escaped braces remain literal text.
      char * argument_end = nullptr;
      std::strtol(message.c_str() + pos + 1, &argument_end, 0);
      if (argument_end[0] != ':' || argument_end[1] != '.') {
        continue;
      }
      const auto precision_start = argument_end + 2;
      char * precision_end = nullptr;
      errno = 0;
      const auto precision = std::strtol(precision_start, &precision_end, 0);
      if (precision_end == precision_start || errno == ERANGE ||
        precision < 0 || precision > MAX_PRECISION)
      {
        throw std::runtime_error("event message precision must be an integer between 0 and 16");
      }
    }
  }
}

// Ask libevents which arguments appear in the dev-profile message. Using marker
// enums avoids duplicating its placeholder, escaping, and profile-tag parser.
std::string with_unprinted_arguments(EventDefinition definition, const Config & config)
{
  auto probe = definition;
  std::vector<EnumDefinition> markers(probe.arguments.size());
  events::EventType event{};
  for (size_t i = 0; i < probe.arguments.size(); ++i) {
    auto & marker = markers[i];
    marker.type = BaseType::uint8_t;
    marker.is_bitfield = false;
    marker.entries[0].description = "@@MAVROS_ARGUMENT_" + std::to_string(i) + "@@";
    probe.arguments[i].type = BaseType::uint8_t;
    probe.arguments[i].enum_def = &marker;
  }
  const auto rendered = ParsedEvent(event, config, probe).message();
  std::string extra;
  for (size_t i = 0; i < definition.arguments.size(); ++i) {
    if (rendered.find(markers[i].entries[0].description) == std::string::npos) {
      if (!extra.empty()) {
        extra += ", ";
      }
      extra += escape_literal(definition.arguments[i].name) + "={" + std::to_string(i + 1) + "}";
    }
  }
  if (!extra.empty()) {
    definition.message += " [" + extra + "]";
  }
  return definition.message;
}

std::optional<std::string> compact_bitfield(
  const events::parser::EventArgumentDefinition & argument, uint64_t raw_value)
{
  const auto & enumeration = *argument.enum_def;
  const int bits = events::parser::baseTypeSize(argument.type) * 8;
  if (bits <= 0 || bits > 64) {
    throw std::runtime_error("unsupported bitfield argument width");
  }
  const uint64_t mask = ~uint64_t{0} >> (64 - bits);
  const uint64_t value = raw_value & mask;
  if (value == 0) {
    return "0";
  }
  // PX4 health checks use an all-bits mask for broadly applicable failures.
  // Other bitfields must retain their individual flag meanings.
  if (enumeration.event_namespace == "px4" &&
    enumeration.name == "navigation_mode_group_t" && argument.name == "modes" && value == mask)
  {
    return "all";
  }

  uint64_t known = 0;
  for (const auto & entry : enumeration.entries) {
    const uint64_t bit = entry.first;
    if (bit != 0 && (bit & (bit - 1)) == 0) {
      known |= bit;
    }
  }
  const uint64_t unknown = value & ~known;
  if (unknown == 0) {
    return std::nullopt;
  }

  std::ostringstream text;
  bool had_entry = false;
  for (int i = 0; i < bits; ++i) {
    const uint64_t bit = uint64_t{1} << i;
    const auto entry = enumeration.entries.find(bit);
    if ((value & bit) != 0 && entry != enumeration.entries.end()) {
      if (had_entry) {
        text << enumeration.entry_separator;
      }
      text << entry->second.description;
      had_entry = true;
    }
  }
  if (had_entry) {
    text << enumeration.entry_separator;
  }
  text << "unknown_bits=0x" << std::hex << std::setfill('0') << std::setw(bits / 4) << unknown;
  return text.str();
}
}  // namespace

struct Decoder::Impl
{
  Parser parser;
  Config config;
  std::map<uint32_t, EventDefinition> definitions;

  explicit Impl(const std::string & metadata)
  {
    const auto json = nlohmann::json::parse(metadata);
    const auto version = json.at("version").get<int>();
    if (version != 1 && version != 2) {
      throw std::runtime_error("unsupported event metadata version");
    }
    if (!json.at("components").is_object() || !parser.loadDefinitions(metadata)) {
      throw std::runtime_error("invalid event metadata");
    }
    parser.setProfile("dev");
    config.profile = "dev";
    for (const auto & component : json.at("components").items()) {
      const auto namespace_id = metadata_id(component.key(), 255);
      const auto & body = component.value();
      if (!body.contains("event_groups")) {
        continue;
      }
      for (const auto & group : body.at("event_groups").items()) {
        const auto & group_events = group.value().at("events");
        if (!group_events.is_object()) {
          throw std::runtime_error("invalid event group");
        }
        for (const auto & entry : group_events.items()) {
          const auto id = (namespace_id << 24) | metadata_id(entry.key(), 0x00ffffff);
          events::EventType event{};
          event.id = id;
          const auto parsed = parser.parse(event);
          if (!parsed) {
            throw std::runtime_error("event contains unsupported argument types");
          }
          EventDefinition definition{};
          definition.id = id;
          definition.name = parsed->name();
          definition.message = entry.value().at("message").get<std::string>();
          size_t argument_bytes = 0;
          for (int i = 0; i < parsed->numArguments(); ++i) {
            const auto & argument = parsed->argument(static_cast<size_t>(i));
            if (argument.isEnum() && argument.type == BaseType::float_t) {
              throw std::runtime_error("event enums require an integer argument type");
            }
            argument_bytes += static_cast<size_t>(events::parser::baseTypeSize(argument.type));
            definition.arguments.push_back(argument);
          }
          if (argument_bytes > sizeof(event.arguments)) {
            throw std::runtime_error("event arguments exceed the MAVLink payload");
          }
          if (definition.message.empty()) {
            definition.message = escape_literal(definition.name);
          }
          validate_message_precision(definition.message);
          definition.message = with_unprinted_arguments(definition, config);
          if (!definitions.emplace(id, std::move(definition)).second) {
            throw std::runtime_error("duplicate full event ID");
          }
        }
      }
    }
    if (definitions.empty()) {
      throw std::runtime_error("metadata contains no event definitions");
    }
  }
};

Decoder::Decoder(ErrorCallback report_error)
: report_error_(std::move(report_error))
{}
Decoder::~Decoder() = default;

std::string Decoder::unpack_metadata(std::string_view bytes, bool compressed)
{
  if (bytes.size() > (compressed ? MAX_COMPRESSED_BYTES : MAX_JSON_BYTES)) {
    throw std::runtime_error("metadata file exceeds size limit");
  }
  if (!compressed) {
    return std::string(bytes);
  }
  return unpack_xz_metadata(bytes, MAX_JSON_BYTES);
}

std::string Decoder::load_json(const std::string & json)
{
  impl_.reset();
  try {
    if (json.size() > MAX_JSON_BYTES) {
      throw std::runtime_error("metadata exceeds JSON size limit");
    }
    impl_ = std::make_unique<Impl>(json);
  } catch (const std::exception & ex) {
    return ex.what();
  } catch (...) {
    return "unknown event metadata loading exception";
  }
  return {};
}

void Decoder::report_failure(uint32_t full_id, const char * reason) const noexcept
{
  try {
    if (report_error_) {
      report_error_(full_id, reason);
    }
  } catch (...) {
    // Diagnostics must not prevent raw event fallback.
  }
}

std::string Decoder::configure(const std::string & mode, const std::string & path)
{
  impl_.reset();
  if (mode != "local" && !path.empty()) {
    return "event_metadata_file is only valid in local mode";
  }
  if (mode == "disabled" || mode == "auto") {
    return {};
  }
  if (mode != "local") {
    return "event_decoding_mode must be disabled, local, or auto";
  }
  if (path.empty()) {
    return "local mode requires event_metadata_file";
  }
  try {
    return load_json(read_metadata(path));
  } catch (const std::exception & ex) {
    return ex.what();
  }
  return {};
}

std::optional<DecodedEvent> Decoder::decode(
  uint32_t full_id, uint8_t log_levels, const std::array<uint8_t, 40> & arguments) const
{
  try {
    if (!impl_) {
      return std::nullopt;
    }
    const auto found = impl_->definitions.find(full_id);
    const uint8_t severity = log_levels & 0x0f;
    if (found == impl_->definitions.end() || severity > 8) {
      return std::nullopt;
    }
    events::EventType event{};
    event.id = full_id;
    event.log_levels = log_levels;
    std::copy(arguments.begin(), arguments.end(), event.arguments);
    const ParsedEvent parsed(event, impl_->config, found->second);
    std::optional<EventDefinition> definition;
    std::vector<EnumDefinition> formatted_bitfields;
    for (int i = 0; i < parsed.numArguments(); ++i) {
      const auto & argument = parsed.argument(static_cast<size_t>(i));
      if (argument.isEnum() && argument.enum_def->is_bitfield) {
        const uint64_t value = parsed.argumentValueInt(i);
        const auto formatted = compact_bitfield(argument, value);
        if (!formatted) {
          continue;
        }
        // Override this value for one render; leave shared definitions and
        // argument bytes untouched. Reserve once to keep enum pointers stable.
        if (!definition) {
          definition = found->second;
          formatted_bitfields.reserve(found->second.arguments.size());
        }
        formatted_bitfields.push_back(*argument.enum_def);
        auto & override_enum = formatted_bitfields.back();
        override_enum.is_bitfield = false;
        override_enum.entries.clear();
        override_enum.entries[value].description = *formatted;
        definition->arguments[static_cast<size_t>(i)].enum_def = &override_enum;
      }
    }
    const auto text = single_line(
      definition ? ParsedEvent(event, impl_->config, *definition).message() : parsed.message());
    if (text.empty()) {
      return std::nullopt;
    }
    // Disabled external severity identifies non-user-facing events. Keep them
    // available to developers at DEBUG, without changing the raw ROS severity.
    return DecodedEvent{static_cast<uint8_t>(severity == 8 ? 7 : severity), text};
  } catch (const std::exception & ex) {
    report_failure(full_id, ex.what());
    return std::nullopt;
  } catch (...) {
    report_failure(full_id, "unknown exception");
    return std::nullopt;
  }
}

}  // namespace px4_events
}  // namespace mavros
