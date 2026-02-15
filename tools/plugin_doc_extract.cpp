#include <algorithm>
#include <atomic>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <functional>
#include <cstdlib>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>
#include <iostream>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

struct ApiEntry {
  std::string name;
  std::string type_name;
  int line = 0;
  std::string default_value;
  std::string description;
};

struct MavlinkSubEntry {
  std::string handler;
  std::string message_type;
  std::string message_name;
  std::string msg_id_expr;
  std::string dialect;
  int msg_id = -1;
  int line = 0;
  std::string description;
};

struct MavlinkPubEntry {
  std::string argument;
  std::string message_type;
  std::string message_name;
  std::string msg_id_expr;
  std::string dialect;
  int msg_id = -1;
  int line = 0;
  std::string description;
};

struct PluginApi {
  std::string plugin;
  fs::path path;
  std::string class_name;
  std::string ns;
  std::string brief;
  std::string description;
  std::vector<ApiEntry> publishers;
  std::vector<ApiEntry> subscribers;
  std::vector<ApiEntry> services;
  std::vector<ApiEntry> clients;
  std::vector<ApiEntry> parameters;
  std::vector<MavlinkSubEntry> mavlink_subscriptions;
  std::vector<MavlinkPubEntry> mavlink_publications;
};

struct Config {
  std::vector<fs::path> plugin_dirs;
  std::set<std::string> plugin_filter;
  fs::path output;
  int jobs = 4;
};

static const std::regex kRegisterPluginRe(R"(MAVROS_PLUGIN_REGISTER\(([^)]+)\))");
static const std::regex kPluginNameRe(R"(@plugin\s+([a-z0-9_]+))");
static const std::regex kPluginBriefRe(R"(@brief\s+([^\n\r]+))");
static const std::regex kPluginNsRe(
  R"re(:\s*(?:[A-Za-z_][A-Za-z0-9_:<>]*::)?(?:Plugin|MissionBase)\s*\(\s*[^,]+,\s*"([^"]+)")re");
static const std::regex kDoxygenBlockRe(R"(/\*\*([\s\S]*?)\*/)");

static const std::regex kPublisherRe(
  R"(create_publisher<([^>]+)>\s*\(\s*([^,]+)\s*,)");
static const std::regex kSubscriptionRe(
  R"(create_subscription<([^>]+)>\s*\(\s*([^,]+)\s*,)");
static const std::regex kServiceRe(
  R"(create_service<([^>]+)>\s*\(\s*([^,]+)\s*,)");
static const std::regex kClientRe(
  R"(create_client<([^>]+)>\s*\(\s*([^,\)]+)\s*[\),])");
static const std::regex kParamWatchRe(
  R"re(node_declare_and_watch_parameter\s*\(\s*"([^"]+)")re");
static const std::regex kParamDeclareRe(
  R"re(node->declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)")re");
static const std::regex kMfDeclRe(
  R"(message_filters::Subscriber<([^>]+)>\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*;)");
static const std::regex kMfSubscribeRe(
  R"(([a-zA-Z_][a-zA-Z0-9_]*)\s*\.\s*subscribe\s*\(\s*[^,]+,\s*([^,\)]+))");
static const std::regex kConstCharPtrRe(
  R"re((?:static\s+)?constexpr\s+const\s+char\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*"([^"]*)"\s*;)re");
static const std::regex kConstCharAutoRe(
  R"re((?:static\s+)?constexpr\s+auto\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*"([^"]*)"\s*;)re");
static const std::regex kParamWatchDefaultRe(
  R"re(node_declare_and_watch_parameter\s*\(\s*"([^"]+)"\s*,\s*([^,]+)\s*,)re");
static const std::regex kParamDeclareDefaultRe(
  R"re(node->declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"\s*,\s*([^,\)]+))re");
static const std::regex kMakeHandlerTypedRe(
  R"(make_handler\s*\(\s*&[a-zA-Z_][a-zA-Z0-9_:]*::([a-zA-Z_][a-zA-Z0-9_]*)\s*\))");
static const std::regex kMakeHandlerRawRe(
  R"(make_handler\s*\(\s*([^,]+)\s*,\s*&[a-zA-Z_][a-zA-Z0-9_:]*::([a-zA-Z_][a-zA-Z0-9_]*)\s*\))");
static const std::regex kFunctionDefRe(
  R"(void\s+(?:[a-zA-Z_][a-zA-Z0-9_]*::)?([a-zA-Z_][a-zA-Z0-9_]*)\s*\(([^;{}]*)\)\s*(?:const\s*)?\{)");
static const std::regex kMavlinkParamTypeRe(
  R"((?:const\s+)?(mavlink::[a-zA-Z0-9_]+::msg::[a-zA-Z0-9_]+)\s*&)");
static const std::regex kAnyRefParamTypeRe(
  R"((?:const\s+)?([A-Za-z_][A-Za-z0-9_:]*)\s*&)");
static const std::regex kUsingAliasMavlinkTypeRe(
  R"(using\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(mavlink::[a-zA-Z0-9_]+::msg::[a-zA-Z0-9_]+)\s*;)");
static const std::regex kUsingImportedMavlinkTypeRe(
  R"(using\s+(mavlink::[a-zA-Z0-9_]+::msg::([A-Za-z0-9_]+))\s*;)");
static const std::regex kClassInheritRe(
  R"(class\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*public\s+([A-Za-z_][A-Za-z0-9_:]*))");
static const std::regex kMsgIdExprTypeRe(
  R"((mavlink::[a-zA-Z0-9_]+::msg::[a-zA-Z0-9_]+)::MSG_ID)");
static const std::regex kMavlinkTypeRe(
  R"(mavlink::([a-zA-Z0-9_]+)::msg::([A-Za-z0-9_]+))");
static const std::regex kHeaderMsgIdRe(
  R"(MSG_ID\s*=\s*([0-9]+))");
static const std::regex kHeaderMsgIdDefineRe(
  R"(MAVLINK_MSG_ID_[A-Z0-9_]+\s+([0-9]+))");
static const std::regex kMissionBaseTypeRe(
  R"re(MissionBase\s*\(\s*[^,]+,\s*"[^"]+"\s*,\s*plugin::MTYPE::([A-Z_]+))re");
static const std::regex kSendMessageRe(
  R"(uas_?->send_message(?:_ignore_drop)?\s*\(\s*([^,\)]+))");
static const std::regex kMavlinkVarDeclRe(
  R"((mavlink::[a-zA-Z0-9_]+::msg::[A-Za-z0-9_]+)\s+([A-Za-z_][A-Za-z0-9_]*)\s*(?:[=({;]))");
static const std::regex kAutoLambdaMavlinkDeclRe(
  R"(auto\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*\([\s\S]{0,256}?->\s*(mavlink::[a-zA-Z0-9_]+::msg::[A-Za-z0-9_]+))");
static const std::regex kAutoMavlinkDirectDeclRe(
  R"((?:const\s+)?auto\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(mavlink::[a-zA-Z0-9_]+::msg::[A-Za-z0-9_]+)\s*[({])");
static const std::regex kAutoTemplateMavlinkDeclRe(
  R"((?:const\s+)?auto\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*[A-Za-z_][A-Za-z0-9_]*\s*<[\s\S]{0,200}?,\s*(mavlink::[a-zA-Z0-9_]+::msg::[A-Za-z0-9_]+)\s*>\s*\()");
static const std::regex kIdentifierRe(
  R"([A-Za-z_][A-Za-z0-9_]*)");

std::string trim(std::string s)
{
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

std::string to_upper(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
  return s;
}

std::string strip_quotes(std::string s)
{
  s = trim(std::move(s));
  std::smatch m;
  static const std::regex str_re(R"re("([^"]+)")re");
  if (std::regex_search(s, m, str_re)) {
    return m[1].str();
  }
  return s;
}

std::string resolve_symbol(std::string raw, const std::map<std::string, std::string> & symbols)
{
  std::string token = trim(std::move(raw));
  if (auto it = symbols.find(token); it != symbols.end()) {
    return it->second;
  }
  if (auto pos = token.rfind("::"); pos != std::string::npos && pos + 2 < token.size()) {
    const std::string suffix = token.substr(pos + 2);
    if (auto it = symbols.find(suffix); it != symbols.end()) {
      return it->second;
    }
  }
  return strip_quotes(token);
}

std::string infer_param_type(const std::string & default_expr)
{
  const std::string expr = trim(default_expr);
  if (expr.empty()) return "";
  if (std::regex_match(expr, std::regex(R"(true|false)"))) return "bool";
  if (expr.find('"') != std::string::npos || expr.find("std::string") != std::string::npos) {
    return "string";
  }
  if (std::regex_match(expr, std::regex(R"([-+]?\d+)"))) return "integer";
  if (std::regex_match(expr, std::regex(R"([-+]?\d*\.\d+(?:[eE][-+]?\d+)?[fFlL]?)")) ||
    std::regex_match(expr, std::regex(R"([-+]?\d+[eE][-+]?\d+[fFlL]?)")))
  {
    return "double";
  }
  if (expr.find(".seconds()") != std::string::npos || expr.find("Duration") != std::string::npos) {
    return "double";
  }
  return "";
}

int line_for_offset(const std::string & text, size_t offset)
{
  return static_cast<int>(1 + std::count(text.begin(), text.begin() + std::min(offset, text.size()), '\n'));
}

std::string read_file(const fs::path & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string read_file_if_exists(const fs::path & path)
{
  if (!fs::exists(path) || !fs::is_regular_file(path)) return "";
  return read_file(path);
}

std::vector<std::string> split_lines(const std::string & text)
{
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    out.push_back(line);
  }
  if (!text.empty() && text.back() == '\n') {
    out.push_back("");
  }
  return out;
}

std::string strip_comment_markers(std::string line)
{
  line = trim(std::move(line));
  if (line.rfind("///", 0) == 0) return trim(line.substr(3));
  if (line.rfind("//!", 0) == 0) return trim(line.substr(3));
  if (line.rfind("//", 0) == 0) return trim(line.substr(2));
  if (line.rfind("/**", 0) == 0) line = line.substr(3);
  else if (line.rfind("/*", 0) == 0) line = line.substr(2);
  if (!line.empty() && line.back() == '/') line.pop_back();
  line = trim(line);
  if (!line.empty() && line[0] == '*') line = trim(line.substr(1));
  if (line.size() >= 2 && line.substr(line.size() - 2) == "*/") {
    line = trim(line.substr(0, line.size() - 2));
  }
  while (!line.empty() && line.back() == '*') {
    line.pop_back();
    line = trim(line);
  }
  return line;
}

std::string extract_entry_comment(const std::vector<std::string> & lines, int line_1based)
{
  if (line_1based <= 0 || static_cast<size_t>(line_1based) > lines.size()) return "";
  const std::string line = lines[static_cast<size_t>(line_1based - 1)];

  if (const size_t cpos = line.find("//"); cpos != std::string::npos) {
    const std::string inline_comment = strip_comment_markers(line.substr(cpos));
    if (!inline_comment.empty()) return inline_comment;
  }

  std::vector<std::string> parts;
  int cur = line_1based - 2;
  int blank_lines = 0;
  for (int i = 0; i < 12 && cur >= 0; ++i, --cur) {
    const std::string t = trim(lines[static_cast<size_t>(cur)]);

    if (t.empty()) {
      if (parts.empty()) {
        if (++blank_lines > 1) break;
        continue;
      }
      break;
    }

    if (t.rfind("//", 0) == 0) {
      parts.push_back(strip_comment_markers(t));
      continue;
    }

    // Keep extraction strict: only contiguous // comments above declarations.
    break;
  }

  if (parts.empty() && line_1based >= 2) {
    int end = line_1based - 2;
    while (end >= 0 && trim(lines[static_cast<size_t>(end)]).empty()) {
      --end;
    }
    if (end >= 0) {
      const std::string end_line = trim(lines[static_cast<size_t>(end)]);
      if (end_line.find("*/") != std::string::npos) {
        std::vector<std::string> block_parts;
        int start = end;
        for (int i = 0; i < 20 && start >= 0; ++i, --start) {
          const std::string bt = trim(lines[static_cast<size_t>(start)]);
          block_parts.push_back(strip_comment_markers(bt));
          if (bt.find("/*") != std::string::npos) break;
        }
        std::reverse(block_parts.begin(), block_parts.end());

        std::string block;
        int block_lines = 0;
        bool has_bad = false;
        bool has_brief = false;
        for (const auto & p : block_parts) {
          const std::string t = trim(p);
          if (t.empty()) continue;
          if (t.rfind("@brief", 0) == 0) {
            const std::string brief = trim(t.substr(6));
            if (!brief.empty()) {
              if (!block.empty()) block += " ";
              block += brief;
              ++block_lines;
              has_brief = true;
            }
            continue;
          }
          if (!t.empty() && t[0] == '@') {
            continue;
          }
          if (t.find("get_subscriptions") != std::string::npos ||
            t.find("make_handler") != std::string::npos ||
            t.find('{') != std::string::npos || t.find('}') != std::string::npos)
          {
            has_bad = true;
          }
          ++block_lines;
          if (!block.empty()) block += " ";
          block += t;
        }

        if (!has_bad && !block.empty() && (has_brief || block_lines <= 6)) {
          return trim(block);
        }
      }
    }
  }

  std::reverse(parts.begin(), parts.end());
  std::string out;
  for (const auto & p : parts) {
    const std::string t = trim(p);
    if (t.empty()) continue;
    if (t.find("[[[end]]]") != std::string::npos) continue;
    if (!out.empty()) out += " ";
    out += t;
  }
  return trim(out);
}

void dedup_entries(std::vector<ApiEntry> & entries)
{
  std::sort(entries.begin(), entries.end(), [](const auto & a, const auto & b) {
    if (a.line != b.line) return a.line < b.line;
    if (a.name != b.name) return a.name < b.name;
    return a.type_name < b.type_name;
  });
  entries.erase(
    std::unique(entries.begin(), entries.end(), [](const auto & a, const auto & b) {
      return a.name == b.name && a.type_name == b.type_name;
    }),
    entries.end());
}

void dedup_mavlink_entries(std::vector<MavlinkSubEntry> & entries)
{
  std::sort(entries.begin(), entries.end(), [](const auto & a, const auto & b) {
    if (a.line != b.line) return a.line < b.line;
    if (a.handler != b.handler) return a.handler < b.handler;
    return a.message_type < b.message_type;
  });
  entries.erase(
    std::unique(entries.begin(), entries.end(), [](const auto & a, const auto & b) {
      return a.handler == b.handler && a.message_type == b.message_type && a.msg_id_expr == b.msg_id_expr;
    }),
    entries.end());
}

void dedup_mavlink_publications(std::vector<MavlinkPubEntry> & entries)
{
  std::sort(entries.begin(), entries.end(), [](const auto & a, const auto & b) {
    if (a.line != b.line) return a.line < b.line;
    if (a.message_type != b.message_type) return a.message_type < b.message_type;
    return a.argument < b.argument;
  });
  entries.erase(
    std::unique(entries.begin(), entries.end(), [](const auto & a, const auto & b) {
      return a.line == b.line && a.message_type == b.message_type && a.argument == b.argument;
    }),
    entries.end());
}

std::string parse_plugin_block(const std::string & text)
{
  std::smatch m;
  auto begin = text.cbegin();
  while (std::regex_search(begin, text.cend(), m, kDoxygenBlockRe)) {
    std::string block = m[1].str();
    if (block.find("@plugin") != std::string::npos) {
      return block;
    }
    begin = m.suffix().first;
  }
  return "";
}

std::string clean_description(const std::string & block)
{
  std::stringstream in(block);
  std::stringstream out;
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    line = trim(line);
    if (!line.empty() && line[0] == '*') {
      line = trim(line.substr(1));
    }
    if (line.empty() || line.rfind("@plugin", 0) == 0 || line.rfind("@brief", 0) == 0) {
      continue;
    }
    if (!first) out << " ";
    out << line;
    first = false;
  }
  return out.str();
}

std::map<std::string, std::string> extract_string_symbols(const std::string & text)
{
  std::map<std::string, std::string> out;

  for (std::sregex_iterator it(text.begin(), text.end(), kConstCharPtrRe), end; it != end; ++it) {
    out[(*it)[1].str()] = (*it)[2].str();
  }
  for (std::sregex_iterator it(text.begin(), text.end(), kConstCharAutoRe), end; it != end; ++it) {
    out[(*it)[1].str()] = (*it)[2].str();
  }

  return out;
}

void extract_entries(
  const std::string & text, const std::regex & re, bool has_type,
  const std::map<std::string, std::string> & symbols, const std::vector<std::string> & lines,
  std::vector<ApiEntry> & out)
{
  for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it) {
    const auto & m = *it;
    ApiEntry e;
    e.type_name = has_type ? trim(m[1].str()) : "";
    e.name = resolve_symbol(has_type ? m[2].str() : m[1].str(), symbols);
    const size_t pos = static_cast<size_t>(m.position());
    e.line = line_for_offset(text, pos);
    e.description = extract_entry_comment(lines, e.line);
    if (!e.name.empty()) out.push_back(std::move(e));
  }
}

void extract_parameters(
  const std::string & text, const std::map<std::string, std::string> & symbols,
  const std::vector<std::string> & lines,
  std::vector<ApiEntry> & out)
{
  for (std::sregex_iterator it(text.begin(), text.end(), kParamWatchDefaultRe), end; it != end; ++it) {
    ApiEntry e;
    e.name = (*it)[1].str();
    e.default_value = trim((*it)[2].str());
    e.type_name = infer_param_type(resolve_symbol(e.default_value, symbols));
    const size_t pos = static_cast<size_t>((*it).position());
    e.line = line_for_offset(text, pos);
    e.description = extract_entry_comment(lines, e.line);
    out.push_back(std::move(e));
  }
  for (std::sregex_iterator it(text.begin(), text.end(), kParamDeclareDefaultRe), end; it != end; ++it) {
    ApiEntry e;
    e.name = (*it)[1].str();
    e.default_value = trim((*it)[2].str());
    e.type_name = infer_param_type(resolve_symbol(e.default_value, symbols));
    const size_t pos = static_cast<size_t>((*it).position());
    e.line = line_for_offset(text, pos);
    e.description = extract_entry_comment(lines, e.line);
    out.push_back(std::move(e));
  }
}

std::map<std::string, std::string> extract_handler_types(const std::string & text)
{
  std::map<std::string, std::string> local_mavlink_bases;
  for (std::sregex_iterator it(text.begin(), text.end(), kClassInheritRe), end; it != end; ++it) {
    const std::string derived = (*it)[1].str();
    const std::string base = trim((*it)[2].str());
    if (base.rfind("mavlink::", 0) == 0) {
      local_mavlink_bases[derived] = base;
    }
  }

  std::map<std::string, std::string> alias_types;
  for (std::sregex_iterator it(text.begin(), text.end(), kUsingAliasMavlinkTypeRe), end; it != end; ++it) {
    alias_types[(*it)[1].str()] = trim((*it)[2].str());
  }
  for (std::sregex_iterator it(text.begin(), text.end(), kUsingImportedMavlinkTypeRe), end; it != end; ++it) {
    alias_types[(*it)[2].str()] = trim((*it)[1].str());
  }

  std::map<std::string, std::string> out;
  for (std::sregex_iterator it(text.begin(), text.end(), kFunctionDefRe), end; it != end; ++it) {
    const std::string handler = (*it)[1].str();
    const std::string params = (*it)[2].str();
    std::smatch type_m;
    if (std::regex_search(params, type_m, kMavlinkParamTypeRe)) {
      out[handler] = trim(type_m[1].str());
    } else if (std::regex_search(params, type_m, kAnyRefParamTypeRe)) {
      const std::string any_type = trim(type_m[1].str());
      if (auto a = alias_types.find(any_type); a != alias_types.end()) {
        out[handler] = a->second;
      } else if (auto b = local_mavlink_bases.find(any_type); b != local_mavlink_bases.end()) {
        out[handler] = b->second;
      }
    }
  }
  return out;
}

std::string tail_name(const std::string & full)
{
  if (const auto p = full.rfind("::"); p != std::string::npos && p + 2 < full.size()) {
    return full.substr(p + 2);
  }
  return full;
}

std::pair<std::string, std::string> split_mavlink_type(const std::string & type_name)
{
  std::smatch m;
  if (std::regex_search(type_name, m, kMavlinkTypeRe)) {
    return {m[1].str(), m[2].str()};
  }
  return {"", ""};
}

std::vector<fs::path> candidate_mavlink_roots(const fs::path & repo_root)
{
  std::vector<fs::path> roots;
  if (const char * env = std::getenv("MAVLINK_INCLUDE_DIR")) {
    roots.emplace_back(env);
  }
  roots.emplace_back(repo_root / "build/mavlink/include/v2.0");
  roots.emplace_back("/ws/build/mavlink/include/v2.0");
  roots.emplace_back("/ws/install/mavlink/include/mavlink/v2.0");
  roots.emplace_back("/opt/ros/kilted/include/mavlink/v2.0");
  return roots;
}

std::map<std::string, int> build_mavlink_msgid_index(const fs::path & repo_root)
{
  std::map<std::string, int> out;
  for (const auto & root : candidate_mavlink_roots(repo_root)) {
    if (!fs::exists(root) || !fs::is_directory(root)) continue;
    for (const auto & dialect_ent : fs::directory_iterator(root)) {
      if (!dialect_ent.is_directory()) continue;
      const std::string dialect = dialect_ent.path().filename().string();
      for (const auto & ent : fs::directory_iterator(dialect_ent.path())) {
        if (!ent.is_regular_file()) continue;
        const auto ext = ent.path().extension().string();
        if (ext != ".hpp" && ext != ".h") continue;
        const std::string fname = ent.path().filename().string();
        const std::string prefix = "mavlink_msg_";
        if (fname.rfind(prefix, 0) != 0) continue;
        auto suffix_pos = fname.rfind(".hpp");
        if (suffix_pos == std::string::npos) {
          suffix_pos = fname.rfind(".h");
        }
        if (suffix_pos == std::string::npos || suffix_pos <= prefix.size()) continue;
        const std::string msg_snake = fname.substr(prefix.size(), suffix_pos - prefix.size());
        std::string msg_upper = msg_snake;
        std::transform(msg_upper.begin(), msg_upper.end(), msg_upper.begin(), [](unsigned char c) {
          return c == '-' ? '_' : std::toupper(c);
        });

        const std::string text = read_file(ent.path());
        std::smatch m;
        if (!std::regex_search(text, m, kHeaderMsgIdRe)) {
          const std::regex msg_define_re(
            "MAVLINK_MSG_ID_" + msg_upper + R"(\s+([0-9]+))");
          if (!std::regex_search(text, m, msg_define_re) &&
            !std::regex_search(text, m, kHeaderMsgIdDefineRe))
          {
            continue;
          }
        }
        const int id = std::stoi(m[1].str());
        out[dialect + "::" + msg_upper] = id;
      }
    }
    if (!out.empty()) break;
  }
  return out;
}

void resolve_mavlink_meta(MavlinkSubEntry & e, const std::map<std::string, int> & idx)
{
  if (!e.message_type.empty()) {
    const auto [dialect, msg_name] = split_mavlink_type(e.message_type);
    if (!dialect.empty()) e.dialect = dialect;
    if (!msg_name.empty() && e.message_name.empty()) e.message_name = msg_name;
  }

  if (e.msg_id < 0 && !e.msg_id_expr.empty()) {
    const std::string ex = trim(e.msg_id_expr);
    if (std::regex_match(ex, std::regex(R"([0-9]+)"))) {
      e.msg_id = std::stoi(ex);
    } else {
      std::smatch idm;
      if (std::regex_search(ex, idm, kMsgIdExprTypeRe)) {
        const auto [dialect, msg_name] = split_mavlink_type(idm[1].str());
        if (e.dialect.empty()) e.dialect = dialect;
        if (e.message_name.empty()) e.message_name = msg_name;
      }
    }
  }

  if (e.msg_id < 0 && !e.dialect.empty() && !e.message_name.empty()) {
    const std::string key = e.dialect + "::" + to_upper(e.message_name);
    if (auto it = idx.find(key); it != idx.end()) {
      e.msg_id = it->second;
    }
  }
}

void resolve_mavlink_meta(MavlinkPubEntry & e, const std::map<std::string, int> & idx)
{
  if (!e.message_type.empty()) {
    const auto [dialect, msg_name] = split_mavlink_type(e.message_type);
    if (!dialect.empty()) e.dialect = dialect;
    if (!msg_name.empty() && e.message_name.empty()) e.message_name = msg_name;
  }

  if (e.msg_id < 0 && !e.msg_id_expr.empty()) {
    const std::string ex = trim(e.msg_id_expr);
    if (std::regex_match(ex, std::regex(R"([0-9]+)"))) {
      e.msg_id = std::stoi(ex);
    } else {
      std::smatch idm;
      if (std::regex_search(ex, idm, kMsgIdExprTypeRe)) {
        const auto [dialect, msg_name] = split_mavlink_type(idm[1].str());
        if (e.dialect.empty()) e.dialect = dialect;
        if (e.message_name.empty()) e.message_name = msg_name;
      }
    }
  }

  if (e.msg_id < 0 && !e.dialect.empty() && !e.message_name.empty()) {
    const std::string key = e.dialect + "::" + to_upper(e.message_name);
    if (auto it = idx.find(key); it != idx.end()) {
      e.msg_id = it->second;
    }
  }
}

std::string normalize_send_arg(std::string arg)
{
  arg = trim(std::move(arg));
  while (!arg.empty() && (arg[0] == '&' || arg[0] == '*')) {
    arg.erase(arg.begin());
    arg = trim(arg);
  }
  if (arg.rfind("std::move(", 0) == 0 && arg.back() == ')') {
    arg = trim(arg.substr(10, arg.size() - 11));
  }
  return arg;
}

std::map<std::string, std::vector<std::pair<int, std::string>>> extract_mavlink_var_decls(
  const std::string & text)
{
  std::map<std::string, std::vector<std::pair<int, std::string>>> out;

  for (std::sregex_iterator it(text.begin(), text.end(), kMavlinkVarDeclRe), end; it != end; ++it) {
    const auto type_name = trim((*it)[1].str());
    const auto var_name = (*it)[2].str();
    const int line = line_for_offset(text, static_cast<size_t>((*it).position()));
    out[var_name].emplace_back(line, type_name);
  }

  for (std::sregex_iterator it(text.begin(), text.end(), kAutoLambdaMavlinkDeclRe), end; it != end; ++it) {
    const auto var_name = (*it)[1].str();
    const auto type_name = trim((*it)[2].str());
    const int line = line_for_offset(text, static_cast<size_t>((*it).position()));
    out[var_name].emplace_back(line, type_name);
  }
  for (std::sregex_iterator it(text.begin(), text.end(), kAutoMavlinkDirectDeclRe), end; it != end; ++it) {
    const auto var_name = (*it)[1].str();
    const auto type_name = trim((*it)[2].str());
    const int line = line_for_offset(text, static_cast<size_t>((*it).position()));
    out[var_name].emplace_back(line, type_name);
  }
  for (std::sregex_iterator it(text.begin(), text.end(), kAutoTemplateMavlinkDeclRe), end; it != end; ++it) {
    const auto var_name = (*it)[1].str();
    const auto type_name = trim((*it)[2].str());
    const int line = line_for_offset(text, static_cast<size_t>((*it).position()));
    out[var_name].emplace_back(line, type_name);
  }

  for (auto & kv : out) {
    auto & v = kv.second;
    std::sort(v.begin(), v.end(), [](const auto & a, const auto & b) { return a.first < b.first; });
  }

  return out;
}

std::vector<std::pair<int, std::string>> extract_mavlink_class_bases(const std::string & text)
{
  std::vector<std::pair<int, std::string>> out;
  for (std::sregex_iterator it(text.begin(), text.end(), kClassInheritRe), end; it != end; ++it) {
    const std::string base = trim((*it)[2].str());
    if (base.rfind("mavlink::", 0) != 0) continue;
    const int line = line_for_offset(text, static_cast<size_t>((*it).position()));
    out.emplace_back(line, base);
  }
  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) { return a.first < b.first; });
  return out;
}

std::string resolve_type_from_var_decls(
  const std::map<std::string, std::vector<std::pair<int, std::string>>> & decls,
  const std::string & var_name, int at_line)
{
  const auto it = decls.find(var_name);
  if (it == decls.end()) return "";
  const auto & vec = it->second;
  std::string best;
  int best_line = -1;
  for (const auto & [line, type_name] : vec) {
    if (line <= at_line && line >= best_line) {
      best_line = line;
      best = type_name;
    }
  }
  return best;
}

std::string resolve_type_from_mavlink_class_bases(
  const std::vector<std::pair<int, std::string>> & bases, int at_line)
{
  std::string best;
  int best_line = -1;
  for (const auto & [line, base] : bases) {
    if (line <= at_line && line >= best_line) {
      best_line = line;
      best = base;
    }
  }
  return best;
}

void extract_mavlink_publications(
  const std::string & text, const std::vector<std::string> & lines,
  const std::map<std::string, int> & msgid_index, std::vector<MavlinkPubEntry> & out)
{
  const auto decls = extract_mavlink_var_decls(text);
  const auto class_bases = extract_mavlink_class_bases(text);

  for (std::sregex_iterator it(text.begin(), text.end(), kSendMessageRe), end; it != end; ++it) {
    MavlinkPubEntry e;
    const size_t pos = static_cast<size_t>((*it).position());
    e.line = line_for_offset(text, pos);
    e.argument = normalize_send_arg((*it)[1].str());
    e.description = extract_entry_comment(lines, e.line);

    std::smatch tm;
    if (std::regex_search(e.argument, tm, kMavlinkTypeRe)) {
      e.message_type = tm[0].str();
    } else if (std::regex_match(e.argument, kIdentifierRe)) {
      e.message_type = resolve_type_from_var_decls(decls, e.argument, e.line);
      if (e.message_type.empty() && e.argument == "this") {
        e.message_type = resolve_type_from_mavlink_class_bases(class_bases, e.line);
      }
    }

    if (!e.message_type.empty()) {
      e.message_name = tail_name(e.message_type);
      e.msg_id_expr = e.message_type + "::MSG_ID";
    }
    resolve_mavlink_meta(e, msgid_index);
    out.push_back(std::move(e));
  }

  dedup_mavlink_publications(out);
}

void extract_mavlink_subscriptions(
  const std::string & text, const std::vector<std::string> & lines,
  const std::map<std::string, int> & msgid_index, std::vector<MavlinkSubEntry> & out)
{
  const auto handler_types = extract_handler_types(text);

  for (std::sregex_iterator it(text.begin(), text.end(), kMakeHandlerRawRe), end; it != end; ++it) {
    MavlinkSubEntry e;
    e.msg_id_expr = trim((*it)[1].str());
    e.handler = (*it)[2].str();
    if (auto p = handler_types.find(e.handler); p != handler_types.end()) {
      e.message_type = p->second;
      e.message_name = tail_name(e.message_type);
    }
    if (e.message_type.empty()) {
      std::smatch idm;
      if (std::regex_search(e.msg_id_expr, idm, kMsgIdExprTypeRe)) {
        e.message_type = trim(idm[1].str());
        e.message_name = tail_name(e.message_type);
      }
    }
    resolve_mavlink_meta(e, msgid_index);
    const size_t pos = static_cast<size_t>((*it).position());
    e.line = line_for_offset(text, pos);
    e.description = extract_entry_comment(lines, e.line);
    out.push_back(std::move(e));
  }

  for (std::sregex_iterator it(text.begin(), text.end(), kMakeHandlerTypedRe), end; it != end; ++it) {
    MavlinkSubEntry e;
    e.handler = (*it)[1].str();
    if (auto p = handler_types.find(e.handler); p != handler_types.end()) {
      e.message_type = p->second;
      e.message_name = tail_name(e.message_type);
      e.msg_id_expr = e.message_type + "::MSG_ID";
    }
    resolve_mavlink_meta(e, msgid_index);
    const size_t pos = static_cast<size_t>((*it).position());
    e.line = line_for_offset(text, pos);
    e.description = extract_entry_comment(lines, e.line);
    out.push_back(std::move(e));
  }

  dedup_mavlink_entries(out);
}

void extract_mavlink_from_context(
  const fs::path & path, const std::map<std::string, int> & msgid_index,
  std::vector<MavlinkSubEntry> * out_subs, std::vector<MavlinkPubEntry> * out_pubs)
{
  const std::string text = read_file_if_exists(path);
  if (text.empty()) return;
  const auto lines = split_lines(text);
  if (out_subs != nullptr) {
    extract_mavlink_subscriptions(text, lines, msgid_index, *out_subs);
  }
  if (out_pubs != nullptr) {
    extract_mavlink_publications(text, lines, msgid_index, *out_pubs);
  }
}

void apply_missionbase_subscription_fallback(
  std::vector<MavlinkSubEntry> & subs, const std::map<std::string, int> & msgid_index)
{
  static const std::map<std::string, std::string> kHandlerType = {
    {"handle_mission_item", "mavlink::common::msg::MISSION_ITEM"},
    {"handle_mission_item_int", "mavlink::common::msg::MISSION_ITEM_INT"},
    {"handle_mission_request", "mavlink::common::msg::MISSION_REQUEST"},
    {"handle_mission_request_int", "mavlink::common::msg::MISSION_REQUEST_INT"},
    {"handle_mission_count", "mavlink::common::msg::MISSION_COUNT"},
    {"handle_mission_ack", "mavlink::common::msg::MISSION_ACK"},
    {"handle_mission_current", "mavlink::common::msg::MISSION_CURRENT"},
    {"handle_mission_item_reached", "mavlink::common::msg::MISSION_ITEM_REACHED"},
  };

  for (auto & e : subs) {
    if (!e.message_type.empty()) continue;
    const auto it = kHandlerType.find(e.handler);
    if (it == kHandlerType.end()) continue;
    e.message_type = it->second;
    e.message_name = tail_name(e.message_type);
    e.msg_id_expr = e.message_type + "::MSG_ID";
    resolve_mavlink_meta(e, msgid_index);
  }
}

bool uses_missionbase_context(const fs::path & path)
{
  static const std::unordered_set<std::string> kFiles = {
    "waypoint.cpp",
    "geofence.cpp",
    "rallypoint.cpp",
  };
  return kFiles.find(path.filename().string()) != kFiles.end();
}

bool uses_setpoint_mixin_context(const fs::path & path)
{
  static const std::unordered_set<std::string> kFiles = {
    "setpoint_accel.cpp",
    "setpoint_attitude.cpp",
    "setpoint_position.cpp",
    "setpoint_raw.cpp",
    "setpoint_trajectory.cpp",
    "setpoint_velocity.cpp",
  };
  return kFiles.find(path.filename().string()) != kFiles.end();
}

PluginApi parse_plugin(
  const fs::path & path, const fs::path & repo_root, const std::map<std::string, int> & msgid_index)
{
  const std::string text = read_file(path);
  const auto lines = split_lines(text);
  const auto symbols = extract_string_symbols(text);
  PluginApi api;
  api.path = fs::relative(path, repo_root);

  const std::string plugin_block = parse_plugin_block(text);
  if (plugin_block.empty()) {
    return api;
  }
  std::smatch m;
  if (std::regex_search(plugin_block, m, kPluginNameRe)) api.plugin = m[1].str();
  if (api.plugin.empty()) return PluginApi{};
  if (std::regex_search(plugin_block, m, kPluginBriefRe)) api.brief = trim(m[1].str());
  api.description = clean_description(plugin_block);

  if (std::regex_search(text, m, kRegisterPluginRe)) api.class_name = trim(m[1].str());
  if (std::regex_search(text, m, kPluginNsRe)) api.ns = trim(m[1].str());

  extract_entries(text, kPublisherRe, true, symbols, lines, api.publishers);
  extract_entries(text, kSubscriptionRe, true, symbols, lines, api.subscribers);
  extract_entries(text, kServiceRe, true, symbols, lines, api.services);
  extract_entries(text, kClientRe, true, symbols, lines, api.clients);
  extract_parameters(text, symbols, lines, api.parameters);
  extract_mavlink_subscriptions(text, lines, msgid_index, api.mavlink_subscriptions);
  extract_mavlink_publications(text, lines, msgid_index, api.mavlink_publications);

  // Pull message metadata from inherited mixins/base classes where handlers/senders are defined.
  if (uses_missionbase_context(path)) {
    extract_mavlink_from_context(
      repo_root / "mavros/include/mavros/mission_protocol_base.hpp",
      msgid_index, &api.mavlink_subscriptions, &api.mavlink_publications);
    extract_mavlink_from_context(
      repo_root / "mavros/src/plugins/mission_protocol_base.cpp",
      msgid_index, nullptr, &api.mavlink_publications);
    apply_missionbase_subscription_fallback(api.mavlink_subscriptions, msgid_index);

    std::smatch mt;
    std::string mission_type = "MISSION";
    if (std::regex_search(text, mt, kMissionBaseTypeRe)) {
      mission_type = mt[1].str();
    }
    if (mission_type != "MISSION") {
      api.mavlink_subscriptions.erase(
        std::remove_if(
          api.mavlink_subscriptions.begin(), api.mavlink_subscriptions.end(),
          [](const MavlinkSubEntry & e) {
            return e.message_name == "MISSION_CURRENT" || e.message_name == "MISSION_ITEM_REACHED";
          }),
        api.mavlink_subscriptions.end());
    }
  }
  if (uses_setpoint_mixin_context(path)) {
    extract_mavlink_from_context(
      repo_root / "mavros/include/mavros/setpoint_mixin.hpp",
      msgid_index, nullptr, &api.mavlink_publications);
  }

  std::map<std::string, std::string> mf_types;
  for (std::sregex_iterator it(text.begin(), text.end(), kMfDeclRe), end; it != end; ++it) {
    mf_types[(*it)[2].str()] = trim((*it)[1].str());
  }
  for (std::sregex_iterator it(text.begin(), text.end(), kMfSubscribeRe), end; it != end; ++it) {
    ApiEntry e;
    const auto var = (*it)[1].str();
    e.name = resolve_symbol((*it)[2].str(), symbols);
    if (auto p = mf_types.find(var); p != mf_types.end()) e.type_name = p->second;
    const size_t pos = static_cast<size_t>((*it).position());
    e.line = line_for_offset(text, pos);
    e.description = extract_entry_comment(lines, e.line);
    if (!e.name.empty()) api.subscribers.push_back(std::move(e));
  }

  dedup_entries(api.publishers);
  dedup_entries(api.subscribers);
  dedup_entries(api.services);
  dedup_entries(api.clients);
  dedup_entries(api.parameters);
  dedup_mavlink_entries(api.mavlink_subscriptions);
  dedup_mavlink_publications(api.mavlink_publications);
  return api;
}

void emit_kv_str(YAML::Emitter & em, const char * key, const std::string & value)
{
  em << YAML::Key << YAML::DoubleQuoted << key;
  em << YAML::Value << YAML::DoubleQuoted << value;
}

void emit_entries(YAML::Emitter & em, const std::vector<ApiEntry> & entries)
{
  em << YAML::Value << YAML::BeginSeq;
  for (const auto & e : entries) {
    em << YAML::BeginMap;
    emit_kv_str(em, "name", e.name);
    emit_kv_str(em, "type_name", e.type_name);
    em << YAML::Key << YAML::DoubleQuoted << "line";
    em << YAML::Value << e.line;
    if (!e.default_value.empty()) {
      emit_kv_str(em, "default_value", e.default_value);
    }
    if (!e.description.empty()) {
      emit_kv_str(em, "description", e.description);
    }
    em << YAML::EndMap;
  }
  em << YAML::EndSeq;
}

void write_json(const std::vector<PluginApi> & items, const fs::path & output)
{
  YAML::Emitter em;
  em << YAML::Flow;
  em << YAML::BeginSeq;
  for (const auto & p : items) {
    em << YAML::BeginMap;
    emit_kv_str(em, "plugin", p.plugin);
    emit_kv_str(em, "path", p.path.generic_string());
    emit_kv_str(em, "class_name", p.class_name);
    emit_kv_str(em, "namespace", p.ns);
    emit_kv_str(em, "brief", p.brief);
    emit_kv_str(em, "description", p.description);

    em << YAML::Key << YAML::DoubleQuoted << "publishers";
    emit_entries(em, p.publishers);
    em << YAML::Key << YAML::DoubleQuoted << "subscribers";
    emit_entries(em, p.subscribers);
    em << YAML::Key << YAML::DoubleQuoted << "services";
    emit_entries(em, p.services);
    em << YAML::Key << YAML::DoubleQuoted << "clients";
    emit_entries(em, p.clients);
    em << YAML::Key << YAML::DoubleQuoted << "parameters";
    emit_entries(em, p.parameters);
    em << YAML::Key << YAML::DoubleQuoted << "mavlink_subscriptions";
    em << YAML::Value << YAML::BeginSeq;
    for (const auto & s : p.mavlink_subscriptions) {
      em << YAML::BeginMap;
      emit_kv_str(em, "handler", s.handler);
      emit_kv_str(em, "message_type", s.message_type);
      emit_kv_str(em, "message_name", s.message_name);
      emit_kv_str(em, "msg_id_expr", s.msg_id_expr);
      emit_kv_str(em, "dialect", s.dialect);
      if (s.msg_id >= 0) {
        em << YAML::Key << YAML::DoubleQuoted << "msg_id";
        em << YAML::Value << s.msg_id;
      }
      em << YAML::Key << YAML::DoubleQuoted << "line";
      em << YAML::Value << s.line;
      if (!s.description.empty()) {
        emit_kv_str(em, "description", s.description);
      }
      em << YAML::EndMap;
    }
    em << YAML::EndSeq;
    em << YAML::Key << YAML::DoubleQuoted << "mavlink_publications";
    em << YAML::Value << YAML::BeginSeq;
    for (const auto & s : p.mavlink_publications) {
      em << YAML::BeginMap;
      emit_kv_str(em, "argument", s.argument);
      emit_kv_str(em, "message_type", s.message_type);
      emit_kv_str(em, "message_name", s.message_name);
      emit_kv_str(em, "msg_id_expr", s.msg_id_expr);
      emit_kv_str(em, "dialect", s.dialect);
      if (s.msg_id >= 0) {
        em << YAML::Key << YAML::DoubleQuoted << "msg_id";
        em << YAML::Value << s.msg_id;
      }
      em << YAML::Key << YAML::DoubleQuoted << "line";
      em << YAML::Value << s.line;
      if (!s.description.empty()) {
        emit_kv_str(em, "description", s.description);
      }
      em << YAML::EndMap;
    }
    em << YAML::EndSeq;
    em << YAML::EndMap;
  }
  em << YAML::EndSeq;

  std::ofstream os(output);
  os << em.c_str() << "\n";
}

Config parse_args(int argc, char ** argv)
{
  Config cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--plugin-dir" && i + 1 < argc) cfg.plugin_dirs.emplace_back(argv[++i]);
    else if (a == "--plugin" && i + 1 < argc) cfg.plugin_filter.insert(argv[++i]);
    else if (a == "--output" && i + 1 < argc) cfg.output = argv[++i];
    else if (a == "--jobs" && i + 1 < argc) cfg.jobs = std::max(1, std::stoi(argv[++i]));
    else if (a == "--help") {
      std::cout
        << "Usage: plugin_doc_gen_cpp [--plugin-dir DIR] [--plugin NAME] [--jobs N] --output FILE\n";
      std::exit(0);
    }
  }
  if (cfg.plugin_dirs.empty()) {
    cfg.plugin_dirs = {fs::path("mavros/src/plugins"), fs::path("mavros_extras/src/plugins")};
  }
  if (cfg.output.empty()) cfg.output = "plugin_api.json";
  return cfg;
}

fs::path detect_repo_root()
{
  fs::path cwd = fs::current_path();
  for (fs::path p = cwd; !p.empty(); p = p.parent_path()) {
    if (fs::exists(p / "mavros/src/plugins") && fs::exists(p / "mavros_extras/src/plugins")) {
      return p;
    }
    if (p == p.root_path()) break;
  }
  return cwd;
}

int main(int argc, char ** argv)
{
  const Config cfg = parse_args(argc, argv);
  const fs::path repo_root = detect_repo_root();
  const auto msgid_index = build_mavlink_msgid_index(repo_root);

  std::vector<fs::path> files;
  for (const auto & dir : cfg.plugin_dirs) {
    const fs::path abs_dir = dir.is_absolute() ? dir : repo_root / dir;
    if (!fs::exists(abs_dir)) continue;
    for (const auto & ent : fs::directory_iterator(abs_dir)) {
      if (!ent.is_regular_file() || ent.path().extension() != ".cpp") continue;
      auto text = read_file(ent.path());
      std::smatch m;
      if (!std::regex_search(text, m, kPluginNameRe)) continue;
      const std::string plugin = m[1].str();
      if (!cfg.plugin_filter.empty() && !cfg.plugin_filter.count(plugin)) continue;
      files.push_back(ent.path());
    }
  }
  std::sort(files.begin(), files.end());

  std::vector<PluginApi> out;
  out.reserve(files.size());
  std::mutex out_mtx;
  std::atomic<size_t> index{0};

  const int jobs = std::max(1, std::min(cfg.jobs, static_cast<int>(files.size() ? files.size() : 1)));
  std::vector<std::thread> workers;
  workers.reserve(jobs);
  for (int i = 0; i < jobs; ++i) {
    workers.emplace_back([&]() {
      while (true) {
        const size_t idx = index.fetch_add(1);
        if (idx >= files.size()) break;
        PluginApi parsed = parse_plugin(files[idx], repo_root, msgid_index);
        if (parsed.plugin.empty()) continue;
        std::lock_guard<std::mutex> lk(out_mtx);
        out.push_back(std::move(parsed));
      }
    });
  }
  for (auto & t : workers) t.join();

  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) {
    return a.plugin < b.plugin;
  });

  fs::create_directories(cfg.output.parent_path());
  write_json(out, cfg.output);
  std::cerr << "Generated " << out.size() << " plugins into " << cfg.output << "\n";
  return 0;
}
