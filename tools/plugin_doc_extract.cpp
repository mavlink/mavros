/*
 * Copyright 2026 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS plugin documentation extractor (clang-tooling based).
 *
 * Parses each plugin source with the clang AST and extracts the exposed ROS
 * API (publishers, subscribers, services, clients, parameters) and MAVLink
 * message traffic, plus the plugin doxygen metadata and per-entity QoS. The
 * output is a JSON document consumed by plugin_doc_gen.py.
 *
 * Dependencies: clang-tooling and yaml-cpp.
 */

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <clang/AST/ASTContext.h>
#include <clang/AST/Decl.h>
#include <clang/AST/DeclCXX.h>
#include <clang/AST/Expr.h>
#include <clang/AST/ExprCXX.h>
#include <clang/AST/ASTTypeTraits.h>
#include <clang/AST/ParentMapContext.h>
#include <clang/AST/RecursiveASTVisitor.h>
#include <clang/Basic/SourceManager.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/Frontend/FrontendActions.h>
#include <clang/Lex/Lexer.h>
#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/CompilationDatabase.h>
#include <clang/Tooling/Tooling.h>
#include <llvm/Support/CommandLine.h>

namespace fs = std::filesystem;
using namespace clang;

// --------------------------------------------------------------------------
// Data structures
// --------------------------------------------------------------------------

struct ApiEntry
{
  std::string name;
  std::string type_name;
  int line = 0;
  std::string default_value;
  std::string description;
  std::string qos_kind;    // "named" | "inline" (empty = none)
  std::string qos_name;    // named helper simple name
  std::string qos_config;  // inline canonical config
  std::string qos_var;     // variable name hint (optional)
};

struct MavlinkEntry
{
  std::string name;         // handler or argument
  std::string message_type;
  std::string message_name;
  std::string msg_id_expr;
  std::string dialect;
  int msg_id = -1;
  int line = 0;
  std::string description;
};

struct PluginApi
{
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
  std::vector<MavlinkEntry> mavlink_subscriptions;
  std::vector<MavlinkEntry> mavlink_publications;
  std::vector<std::string> plugin_bases;   // direct base class simple names
  std::string file;         // absolute source path for mapping
};

// --------------------------------------------------------------------------
// Small helpers
// --------------------------------------------------------------------------

std::string trim(std::string s)
{
  auto not_space = [](unsigned char c) {return !std::isspace(c);};
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

std::string to_upper(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::toupper(c);});
  return s;
}

std::string tail_name(const std::string & full)
{
  if (const auto p = full.rfind("::"); p != std::string::npos && p + 2 < full.size()) {
    return full.substr(p + 2);
  }
  return full;
}

std::string strip_quotes(std::string s)
{
  s = trim(std::move(s));
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

std::pair<std::string, std::string> split_mavlink_type(const std::string & type_name)
{
  auto p1 = type_name.rfind("::msg::");
  if (p1 == std::string::npos) {
    return {"", ""};
  }
  const std::string msg = type_name.substr(p1 + 7);
  auto lt = msg.find('<');
  const std::string msg_clean = (lt == std::string::npos) ? msg : msg.substr(0, lt);
  const std::string prefix = type_name.substr(0, p1);
  auto p2 = prefix.rfind("::");
  const std::string dialect = (p2 == std::string::npos) ? prefix : prefix.substr(p2 + 2);
  return {dialect, msg_clean};
}

std::string read_file(const fs::path & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// --------------------------------------------------------------------------
// MAVLink msgid index (read from generated mavlink headers)
// --------------------------------------------------------------------------

static const std::regex kHeaderMsgIdRe(R"(MSG_ID\s*=\s*([0-9]+))");

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
    if (!fs::exists(root) || !fs::is_directory(root)) {continue;}
    for (const auto & dialect_ent : fs::directory_iterator(root)) {
      if (!dialect_ent.is_directory()) {continue;}
      const std::string dialect = dialect_ent.path().filename().string();
      for (const auto & ent : fs::directory_iterator(dialect_ent.path())) {
        if (!ent.is_regular_file()) {continue;}
        const std::string fname = ent.path().filename().string();
        if (fname.rfind("mavlink_msg_", 0) != 0) {continue;}
        auto suffix_pos = fname.rfind(".hpp");
        if (suffix_pos == std::string::npos) {suffix_pos = fname.rfind(".h");}
        if (suffix_pos == std::string::npos || suffix_pos <= 12) {continue;}
        std::string msg_snake = fname.substr(12, suffix_pos - 12);
        std::string msg_upper;
        for (unsigned char c : msg_snake) {
          msg_upper += (c == '-') ? '_' : std::toupper(c);
        }
        const std::string text = read_file(ent.path());
        std::smatch m;
        if (!std::regex_search(text, m, kHeaderMsgIdRe)) {continue;}
        out[dialect + "::" + msg_upper] = std::stoi(m[1].str());
      }
    }
    if (!out.empty()) {break;}
  }
  return out;
}

void resolve_mavlink_meta(MavlinkEntry & e, const std::map<std::string, int> & idx)
{
  if (!e.message_type.empty()) {
    const auto [dialect, msg_name] = split_mavlink_type(e.message_type);
    if (!dialect.empty()) {e.dialect = dialect;}
    if (!msg_name.empty() && e.message_name.empty()) {e.message_name = msg_name;}
  }
  if (e.msg_id < 0 && !e.dialect.empty() && !e.message_name.empty()) {
    const std::string key = e.dialect + "::" + to_upper(e.message_name);
    if (auto it = idx.find(key); it != idx.end()) {
      e.msg_id = it->second;
    }
  }
}

// --------------------------------------------------------------------------
// QoS
// --------------------------------------------------------------------------

static const std::unordered_set<std::string> kNamedQosHelpers = {
  "LatchedStateQoS", "SensorDataQoS", "ServicesQoS", "ParametersQoS",
  "ParameterEventsQoS", "SystemDefaultQoS", "RosoutQoS",
};

struct QosInfo
{
  bool present = false;
  std::string kind;      // "named" | "inline"
  std::string name;      // named helper simple name
  std::string config;    // canonical config string for inline
  std::string var;       // variable name hint
};

// --------------------------------------------------------------------------
// AST visitor
// --------------------------------------------------------------------------

class PluginVisitor : public RecursiveASTVisitor<PluginVisitor>
{
public:
  PluginVisitor(ASTContext & ctx, PluginApi & api, const std::map<std::string, int> & idx)
  : ctx_(ctx), SM_(ctx.getSourceManager()), api_(api), idx_(idx) {}

  bool VisitCXXRecordDecl(CXXRecordDecl * rec)
  {
    if (!rec->hasDefinition() || !rec->getIdentifier() ||
      !SM_.isInMainFile(rec->getBeginLoc()))
    {
      return true;
    }
    RawComment * raw = rec->getASTContext().getRawCommentForDeclNoCache(rec);
    if (!raw) {
      return true;
    }
    const std::string text = raw->getRawText(SM_).str();
    if (text.find("@plugin") == std::string::npos) {
      return true;
    }
    api_.class_name = rec->getQualifiedNameAsString();
    api_.ns = rec->getQualifiedNameAsString();
    if (const auto p = api_.ns.rfind("::"); p != std::string::npos) {
      api_.ns = api_.ns.substr(0, p);
    }
    for (const CXXBaseSpecifier & b : rec->bases()) {
      api_.plugin_bases.push_back(tail_name(b.getType().getAsString()));
    }
    parse_plugin_comment(text);
    return true;
  }

  bool VisitVarDecl(VarDecl * vd)
  {
    if (!SM_.isInMainFile(vd->getBeginLoc())) {
      return true;
    }
    const std::string t = vd->getType().getAsString();
    if (t.find("message_filters::Subscriber<") != std::string::npos) {
      auto lt = t.find('<');
      auto gt = t.rfind('>');
      if (lt != std::string::npos && gt != std::string::npos && gt > lt) {
        ApiEntry e;
        e.name = vd->getName().str();
        e.type_name = clean_type(t.substr(lt + 1, gt - lt - 1));
        e.line = static_cast<int>(SM_.getSpellingLineNumber(vd->getBeginLoc()));
        e.description = extract_comment_above(e.line);
        mf_sub_index_[e.name] = api_.subscribers.size();
        api_.subscribers.push_back(std::move(e));
      }
    }
    return true;
  }

  bool VisitFieldDecl(FieldDecl * fd)
  {
    if (!SM_.isInMainFile(fd->getBeginLoc())) {
      return true;
    }
    const std::string t = fd->getType().getAsString();
    if (t.find("message_filters::Subscriber<") != std::string::npos) {
      auto lt = t.find('<');
      auto gt = t.rfind('>');
      if (lt != std::string::npos && gt != std::string::npos && gt > lt) {
        ApiEntry e;
        e.name = fd->getName().str();
        e.type_name = clean_type(t.substr(lt + 1, gt - lt - 1));
        e.line = static_cast<int>(SM_.getSpellingLineNumber(fd->getBeginLoc()));
        e.description = extract_comment_above(e.line);
        mf_sub_index_[e.name] = api_.subscribers.size();
        api_.subscribers.push_back(std::move(e));
      }
    }
    return true;
  }

  bool VisitCXXMemberCallExpr(CXXMemberCallExpr * call)
  {
    if (!SM_.isInMainFile(call->getBeginLoc())) {
      return true;
    }
    const FunctionDecl * callee = call->getDirectCallee();
    if (!callee) {
      return true;
    }
    const std::string name = callee->getName().str();
    if (name == "subscribe" && call->getNumArgs() >= 2) {
      // message_filters::Subscriber<T>::subscribe(node, topic, qos)
      if (auto * obj = dyn_cast<DeclRefExpr>(call->getImplicitObjectArgument())) {
        const std::string var = obj->getDecl()->getName().str();
        auto it = mf_sub_index_.find(var);
        if (it != mf_sub_index_.end() && it->second < api_.subscribers.size()) {
          const std::string topic = resolve_string(call->getArg(1));
          api_.subscribers[it->second].name = topic.empty() ? var : topic;
          if (call->getNumArgs() >= 3) {
            set_qos(api_.subscribers[it->second], resolve_qos(call->getArg(2)));
          }
        }
      }
    }
    if (name == "create_publisher") {
      handle_create(call, api_.publishers);
    } else if (name == "create_subscription") {
      handle_create(call, api_.subscribers);
    } else if (name == "create_service") {
      handle_service(call, api_.services);
    } else if (name == "create_client") {
      handle_create(call, api_.clients);
    } else if (name == "node_declare_and_watch_parameter") {
      handle_param_watch(call);
    } else if (name == "make_handler") {
      handle_make_handler(call);
    } else if (name == "send_message" || name == "send_message_ignore_drop") {
      handle_send_message(call);
    }
    return true;
  }

private:
  ASTContext & ctx_;
  SourceManager & SM_;
  PluginApi & api_;
  const std::map<std::string, int> & idx_;
  std::vector<std::string> src_lines_;
  std::map<std::string, size_t> mf_sub_index_;

  std::string source_text(Expr * e)
  {
    if (!e) {return "";}
    return Lexer::getSourceText(
      CharSourceRange::getTokenRange(e->getSourceRange()), SM_, ctx_.getLangOpts()).str();
  }

  //! Whole plugin source lines (loaded lazily) for text-based comment extraction.
  std::vector<std::string> & src_lines()
  {
    if (src_lines_.empty() && !api_.file.empty()) {
      std::ifstream in(api_.file);
      std::string line;
      while (std::getline(in, line)) {
        src_lines_.push_back(line);
      }
    }
    return src_lines_;
  }

  //! Comment lines directly above a source line (skips assignment prefixes).
  std::string extract_comment_above(int decl_line)
  {
    auto & lines = src_lines();
    if (decl_line <= 1 || static_cast<size_t>(decl_line) > lines.size()) {return "";}
    std::vector<std::string> parts;
    int cur = decl_line - 2;
    int blanks = 0;
    for (int i = 0; i < 12 && cur >= 0; ++i, --cur) {
      const std::string t = trim(lines[static_cast<size_t>(cur)]);
      if (t.empty()) {
        if (parts.empty()) {
          if (++blanks > 1) {break;}
          continue;
        }
        break;
      }
      if (t.rfind("//", 0) == 0) {
        parts.push_back(strip_comment_marker(t));
        continue;
      }
      // assignment prefix line, e.g. "get_parameters_srv ="
      if (std::regex_match(t, std::regex(R"([A-Za-z_][A-Za-z0-9_:]*\s*=\s*)"))) {
        continue;
      }
      break;
    }
    std::reverse(parts.begin(), parts.end());
    std::string out;
    for (const auto & p : parts) {
      if (!out.empty()) {out += " ";}
      out += p;
    }
    return out;
  }

  static std::string strip_comment_marker(std::string line)
  {
    line = trim(std::move(line));
    if (line.rfind("//", 0) == 0) {line = trim(line.substr(2));}
    if (line.rfind("*", 0) == 0) {line = trim(line.substr(1));}
    if (line.rfind("!", 0) == 0) {line = trim(line.substr(1));}
    return line;
  }

  //! Normalize a C++ type string to the logical ROS/mavlink type name.
  static std::string clean_type(std::string t)
  {
    t = trim(std::move(t));
    if (t.rfind("struct ", 0) == 0) {
      t = trim(t.substr(7));
    } else if (t.rfind("class ", 0) == 0) {
      t = trim(t.substr(6));
    }
    if (t.rfind("const ", 0) == 0) {
      t = trim(t.substr(6));
    }
    while (!t.empty() && (t.back() == '&' || t.back() == '*')) {
      t.pop_back();
    }
    // strip the generated ROS2 allocator template suffix: Foo_<...>
    if (const auto lt = t.rfind("_<"); lt != std::string::npos) {
      t = t.substr(0, lt);
    }
    return trim(t);
  }

  //! Walk up parents to find the enclosing VarDecl of a statement.
  VarDecl * enclosing_var(Stmt * s)
  {
    DynTypedNode cur = DynTypedNode::create(*s);
    while (true) {
      DynTypedNodeList parents = ctx_.getParents(cur);
      if (parents.empty()) {return nullptr;}
      cur = parents[0];
      if (const auto vd = cur.get<VarDecl>()) {
        return const_cast<VarDecl *>(vd);
      }
      if (!cur.get<Stmt>() && !cur.get<Decl>()) {
        return nullptr;
      }
    }
  }

  std::string decl_comment(Stmt * s)
  {
    VarDecl * vd = enclosing_var(s);
    if (!vd) {return "";}
    RawComment * rc = ctx_.getRawCommentForDeclNoCache(vd);
    if (!rc) {return "";}
    return clean_comment(rc->getRawText(SM_).str());
  }

  static std::string clean_comment(std::string text)
  {
    std::stringstream in(text);
    std::string out;
    std::string line;
    while (std::getline(in, line)) {
      std::string t = trim(line);
      if (t.rfind("/*", 0) == 0 || t.rfind("*/", 0) == 0) {continue;}
      if (t.rfind("//", 0) == 0) {t = trim(t.substr(2));}
      if (t.rfind("*", 0) == 0) {t = trim(t.substr(1));}
      if (t.empty() || t.rfind("@", 0) == 0) {continue;}
      if (!out.empty()) {out += " ";}
      out += t;
    }
    return out;
  }

  std::string resolve_string(Expr * e)
  {
    if (!e) {return "";}
    e = e->IgnoreImplicit();
    if (auto * sl = dyn_cast<StringLiteral>(e)) {
      return sl->getString().str();
    }
    if (auto * ctor = dyn_cast<CXXConstructExpr>(e)) {
      if (ctor->getNumArgs() == 1) {
        return resolve_string(ctor->getArg(0));
      }
    }
    if (auto * dre = dyn_cast<DeclRefExpr>(e)) {
      if (auto * vd = dyn_cast<VarDecl>(dre->getDecl())) {
        if (vd->hasInit()) {
          return resolve_string(vd->getInit());
        }
      }
    }
    if (auto * me = dyn_cast<MemberExpr>(e)) {
      if (auto * vd = dyn_cast<VarDecl>(me->getMemberDecl())) {
        if (vd->hasInit()) {
          return resolve_string(vd->getInit());
        }
      }
    }
    return "";
  }

  //! Reconstruct a QoS expression as a canonical config string from the AST.
  std::string describe_qos(Expr * e)
  {
    if (!e) {return "";}
    e = e->IgnoreImplicit();
    if (auto * mc = dyn_cast<CXXMemberCallExpr>(e)) {
      const std::string obj = describe_qos(mc->getImplicitObjectArgument());
      const std::string m = mc->getMethodDecl()->getName().str();
      return obj.empty() ? m + "()" : obj + "." + m + "()";
    }
    if (auto * cons = dyn_cast<CXXConstructExpr>(e)) {
      // If we are constructing a QoS from another QoS expression, return the
      // inner expression as-is (avoid "QoS(rclcpp::QoS(10).transient_local())").
      if (cons->getNumArgs() == 1) {
        const std::string inner = describe_qos(cons->getArg(0));
        if (inner.find("QoS") != std::string::npos) {
          return inner;
        }
      }
      std::string base = tail_name(cons->getType().getAsString());
      std::string args;
      bool first = true;
      for (unsigned i = 0; i < cons->getNumArgs(); ++i) {
        const std::string a = describe_qos(cons->getArg(i));
        if (a.empty()) {continue;}
        if (!first) {args += ",";}
        args += a;
        first = false;
      }
      return base + "(" + args + ")";
    }
    if (auto * il = dyn_cast<IntegerLiteral>(e)) {
      return std::to_string(il->getValue().getZExtValue());
    }
    if (auto * sl = dyn_cast<StringLiteral>(e)) {
      return sl->getString().str();
    }
    if (auto * ce = dyn_cast<CallExpr>(e)) {
      if (const FunctionDecl * d = ce->getDirectCallee()) {
        return tail_name(d->getName().str()) + "()";
      }
    }
    // fall back to source text
    std::string t;
    for (const char c : source_text(e)) {
      if (!std::isspace(static_cast<unsigned char>(c))) {t += c;}
    }
    return t;
  }

  QosInfo resolve_qos(Expr * e)
  {
    QosInfo q;
    if (!e) {return q;}
    e = e->IgnoreImplicit();
    if (auto * dre = dyn_cast<DeclRefExpr>(e)) {
      if (auto * vd = dyn_cast<VarDecl>(dre->getDecl())) {
        if (vd->hasInit()) {
          q = resolve_qos(vd->getInit());
          if (q.present && q.var.empty()) {
            q.var = vd->getName().str();
          }
          return q;
        }
      }
    }
    // named helper detection (member call, free call, or construct)
    std::string callee;
    if (auto * mc = dyn_cast<CXXMemberCallExpr>(e)) {
      if (const FunctionDecl * d = mc->getDirectCallee()) {
        callee = d->getName().str();
      }
    } else if (auto * ce = dyn_cast<CallExpr>(e)) {
      if (const FunctionDecl * d = ce->getDirectCallee()) {
        callee = d->getName().str();
      }
    } else if (auto * cons = dyn_cast<CXXConstructExpr>(e)) {
      callee = tail_name(cons->getType().getAsString());
    }
    if (!callee.empty() && kNamedQosHelpers.count(callee) != 0) {
      q.present = true;
      q.kind = "named";
      q.name = callee;
      return q;
    }
    // Only treat as inline QoS if the expression type is a QoS class.
    if (e->getType().getAsString().find("QoS") == std::string::npos) {
      return q;
    }
    const std::string cfg = describe_qos(e);
    if (cfg.empty()) {return q;}
    q.present = true;
    q.kind = "inline";
    q.config = cfg;
    return q;
  }

  std::string template_arg_type(CXXMemberCallExpr * call, unsigned idx_arg)
  {
    const FunctionDecl * callee = call->getDirectCallee();
    if (!callee) {return "";}
    if (auto * tsi = callee->getTemplateSpecializationInfo()) {
      const TemplateArgumentList & args = *tsi->TemplateArguments;
      if (idx_arg < args.size() && args[idx_arg].getKind() == TemplateArgument::Type) {
        return args[idx_arg].getAsType().getAsString();
      }
    }
    return "";
  }

  void set_qos(ApiEntry & e, const QosInfo & q)
  {
    if (!q.present) {return;}
    e.qos_kind = q.kind;
    e.qos_name = q.name;
    e.qos_config = q.config;
    e.qos_var = q.var;
  }

  void handle_create(CXXMemberCallExpr * call, std::vector<ApiEntry> & out)
  {
    if (call->getNumArgs() < 1) {return;}
    ApiEntry e;
    e.type_name = clean_type(template_arg_type(call, 0));
    e.name = resolve_string(call->getArg(0));
    if (e.name.empty()) {
      Expr * a = call->getArg(0)->IgnoreImplicit();
      if (isa<DeclRefExpr>(a) || isa<MemberExpr>(a)) {
        // runtime-configured topic (e.g. per-sensor distance sensor)
        e.name = "<configurable per sensor>";
      } else {
        e.name = strip_quotes(trim(source_text(a)));
      }
    }
    e.line = static_cast<int>(SM_.getSpellingLineNumber(call->getBeginLoc()));
    if (call->getNumArgs() >= 2) {
      set_qos(e, resolve_qos(call->getArg(1)));
    }
    e.description = extract_comment_above(e.line);
    out.push_back(std::move(e));
  }

  void handle_service(CXXMemberCallExpr * call, std::vector<ApiEntry> & out)
  {
    if (call->getNumArgs() < 1) {return;}
    ApiEntry e;
    e.type_name = clean_type(template_arg_type(call, 0));
    e.name = resolve_string(call->getArg(0));
    if (e.name.empty()) {
      Expr * a = call->getArg(0)->IgnoreImplicit();
      if (isa<DeclRefExpr>(a) || isa<MemberExpr>(a)) {
        // runtime-configured topic (e.g. per-sensor distance sensor)
        e.name = "<configurable per sensor>";
      } else {
        e.name = strip_quotes(trim(source_text(a)));
      }
    }
    e.line = static_cast<int>(SM_.getSpellingLineNumber(call->getBeginLoc()));
    // create_service<SrvT>(name, cb, qos, cg)
    if (call->getNumArgs() >= 3) {
      set_qos(e, resolve_qos(call->getArg(2)));
    }
    e.description = extract_comment_above(e.line);
    out.push_back(std::move(e));
  }

  void handle_param_watch(CXXMemberCallExpr * call)
  {
    if (call->getNumArgs() < 1) {return;}
    ApiEntry e;
    e.name = resolve_string(call->getArg(0));
    if (e.name.empty()) {
      // fall back to the source text (e.g. a string literal wrapped in a temp)
      e.name = strip_quotes(trim(source_text(call->getArg(0))));
    }
    e.line = static_cast<int>(SM_.getSpellingLineNumber(call->getBeginLoc()));
    if (call->getNumArgs() >= 2) {
      e.default_value = trim(source_text(call->getArg(1)));
      e.type_name = infer_param_type(e.default_value);
    }
    e.description = extract_comment_above(e.line);
    api_.parameters.push_back(std::move(e));
  }

  void handle_make_handler(CXXMemberCallExpr * call)
  {
    if (call->getNumArgs() == 0) {return;}
    MavlinkEntry e;
    Expr * arg0 = call->getArg(0)->IgnoreImplicit();
    FunctionDecl * fd = nullptr;
    if (auto * uop = dyn_cast<UnaryOperator>(arg0)) {   // &Plugin::fn (typed)
      if (auto * dre = dyn_cast<DeclRefExpr>(uop->getSubExpr())) {
        fd = dyn_cast<FunctionDecl>(dre->getDecl());
        if (fd) {
          e.name = fd->getName().str();
          for (const ParmVarDecl * p : fd->parameters()) {
            std::string t = clean_type(p->getType().getAsString());
            if (t.find("mavlink::") != std::string::npos &&
              t.find("::msg::") != std::string::npos)
            {
              e.message_type = t;
              break;
            }
            // param may be a class derived from a mavlink message (e.g. FTPRequest
            // derives from FILE_TRANSFER_PROTOCOL).
            if (auto * rec = p->getType().getNonReferenceType()->getAsCXXRecordDecl()) {
              for (const CXXBaseSpecifier & b : rec->bases()) {
                const std::string bt = clean_type(b.getType().getAsString());
                if (bt.find("mavlink::") != std::string::npos) {
                  e.message_type = bt;
                  break;
                }
              }
              if (!e.message_type.empty()) {break;}
            }
          }
        }
      }
    } else {   // raw: make_handler(msgid_expr, &Plugin::fn)
      // The msgid expr is usually mavlink::...::msg::X::MSG_ID; resolve X from the
      // enclosing class of the MSG_ID static member.
      if (auto * dre = dyn_cast<DeclRefExpr>(arg0)) {
        if (auto * vd = dyn_cast<VarDecl>(dre->getDecl())) {
          if (auto * ctxt = dyn_cast<CXXRecordDecl>(vd->getDeclContext())) {
            e.message_type = ctxt->getQualifiedNameAsString();
          }
        }
      }
      if (e.message_type.empty()) {
        const std::string id_src = trim(source_text(arg0));
        if (id_src.size() > 7 && id_src.substr(id_src.size() - 7) == "::MSG_ID") {
          e.message_type = id_src.substr(0, id_src.size() - 7);
        }
      }
      if (call->getNumArgs() >= 2) {
        e.name = get_handler_name(call->getArg(1));
        if (auto * uop2 = dyn_cast<UnaryOperator>(call->getArg(1)->IgnoreImplicit())) {
          if (auto * dre = dyn_cast<DeclRefExpr>(uop2->getSubExpr())) {
            fd = dyn_cast<FunctionDecl>(dre->getDecl());
          }
        }
      }
    }
    if (!e.message_type.empty()) {
      e.msg_id_expr = e.message_type + "::MSG_ID";
    }
    if (fd) {
      e.line = static_cast<int>(SM_.getSpellingLineNumber(fd->getBeginLoc()));
      e.description = extract_comment_above(e.line);
    }
    resolve_mavlink_meta(e, idx_);
    api_.mavlink_subscriptions.push_back(std::move(e));
  }

  std::string get_handler_name(Expr * e)
  {
    e = e->IgnoreImplicit();
    if (auto * uop = dyn_cast<UnaryOperator>(e)) {
      if (auto * dre = dyn_cast<DeclRefExpr>(uop->getSubExpr())) {
        return dre->getDecl()->getName().str();
      }
    }
    return "";
  }

  void handle_send_message(CXXMemberCallExpr * call)
  {
    if (call->getNumArgs() == 0) {return;}
    Expr * arg = call->getArg(0)->IgnoreImplicit();
    MavlinkEntry e;
    e.line = static_cast<int>(SM_.getSpellingLineNumber(call->getBeginLoc()));
    e.message_type = clean_type(arg->getType().getAsString());
    // If the argument is a class derived from a mavlink message (e.g. FTPRequest
    // derives from FILE_TRANSFER_PROTOCOL), resolve to that mavlink base.
    if (e.message_type.find("mavlink::") == std::string::npos) {
      if (auto * rec = arg->getType()->getAsCXXRecordDecl()) {
        for (const CXXBaseSpecifier & b : rec->bases()) {
          const std::string bt = clean_type(b.getType().getAsString());
          if (bt.find("mavlink::") != std::string::npos) {
            e.message_type = bt;
            break;
          }
        }
      }
    }
    e.name = "msg";
    e.message_name = tail_name(e.message_type);
    e.msg_id_expr = e.message_type + "::MSG_ID";
    resolve_mavlink_meta(e, idx_);
    api_.mavlink_publications.push_back(std::move(e));
  }

  std::string infer_param_type(const std::string & default_expr)
  {
    const std::string expr = trim(default_expr);
    if (expr.empty()) {return "";}
    if (expr == "true" || expr == "false") {return "bool";}
    if (expr.find('"') != std::string::npos || expr.find("std::string") != std::string::npos) {
      return "string";
    }
    if (std::regex_match(expr, std::regex(R"([-+]?\d+)"))) {return "integer";}
    if (std::regex_match(expr, std::regex(R"([-+]?\d*\.\d+[fFlL]?)")) ||
      std::regex_match(expr, std::regex(R"([-+]?\d+[eE][-+]?\d+[fFlL]?)")))
    {
      return "double";
    }
    if (expr.find(".seconds()") != std::string::npos ||
      expr.find("Duration") != std::string::npos)
    {
      return "double";
    }
    return "";
  }

  void parse_plugin_comment(const std::string & text)
  {
    std::stringstream in(text);
    std::string line;
    std::vector<std::string> desc_lines;
    while (std::getline(in, line)) {
      std::string t = trim(line);
      if (t.rfind("/*", 0) == 0 || t.rfind("*/", 0) == 0) {continue;}
      if (t.rfind("//", 0) == 0) {t = trim(t.substr(2));}
      if (t.rfind("*", 0) == 0) {t = trim(t.substr(1));}
      if (t.rfind("@plugin", 0) == 0) {
        std::smatch m;
        if (std::regex_search(t, m, std::regex(R"(@plugin\s+([a-z0-9_]+))"))) {
          api_.plugin = m[1].str();
        }
        continue;
      }
      if (t.rfind("@brief", 0) == 0) {
        api_.brief = trim(t.substr(6));
        continue;
      }
      if (t.empty()) {
        if (!desc_lines.empty()) {desc_lines.push_back("");}
        continue;
      }
      if (t.rfind("@", 0) == 0) {continue;}
      desc_lines.push_back(t);
    }
    std::string out;
    for (size_t i = 0; i < desc_lines.size(); ++i) {
      if (i > 0) {
        out += (desc_lines[i].empty() || desc_lines[i - 1].empty()) ? "\n\n" : "\n";
      }
      out += desc_lines[i];
    }
    api_.description = out;
  }
};

// --------------------------------------------------------------------------
// Frontend action
// --------------------------------------------------------------------------

class PluginAction : public ASTFrontendAction
{
public:
  PluginAction(
    std::map<std::string, PluginApi *> & file_to_api,
    const std::map<std::string, int> & idx)
  : file_to_api_(file_to_api), idx_(idx) {}

protected:
  std::unique_ptr<ASTConsumer> CreateASTConsumer(
    CompilerInstance & CI, StringRef infile) override
  {
    PluginApi * api = nullptr;
    const std::string infile_std = infile.str();
    auto it = file_to_api_.find(infile_std);
    if (it != file_to_api_.end()) {
      api = it->second;
    } else {
      // Fall back to the main-file id if the path spelling differs.
      auto fid = CI.getSourceManager().getMainFileID();
      auto fe = CI.getSourceManager().getFileEntryRefForID(fid);
      if (fe) {
        it = file_to_api_.find(fe->getName().str());
        if (it != file_to_api_.end()) {
          api = it->second;
        }
      }
    }
    if (!api) {
      api = &fallback_;
    }
    return std::make_unique<Consumer>(CI.getASTContext(), *api, idx_);
  }

private:
  std::map<std::string, PluginApi *> & file_to_api_;
  const std::map<std::string, int> & idx_;
  PluginApi fallback_;

  class Consumer : public ASTConsumer
  {
public:
    Consumer(ASTContext & ctx, PluginApi & api, const std::map<std::string, int> & idx)
    : visitor_(ctx, api, idx) {}

    void HandleTranslationUnit(ASTContext & ctx) override
    {
      visitor_.TraverseDecl(ctx.getTranslationUnitDecl());
    }

private:
    PluginVisitor visitor_;
  };
};

// --------------------------------------------------------------------------
// Output (YAML-emitted JSON)
// --------------------------------------------------------------------------

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
    if (!e.qos_kind.empty()) {
      em << YAML::Key << YAML::DoubleQuoted << "qos" << YAML::BeginMap;
      emit_kv_str(em, "kind", e.qos_kind);
      if (e.qos_kind == "named") {
        emit_kv_str(em, "name", e.qos_name);
      } else {
        emit_kv_str(em, "config", e.qos_config);
      }
      if (!e.qos_var.empty()) {
        emit_kv_str(em, "var", e.qos_var);
      }
      em << YAML::EndMap;
    }
    if (!e.default_value.empty()) {emit_kv_str(em, "default_value", e.default_value);}
    if (!e.description.empty()) {emit_kv_str(em, "description", e.description);}
    em << YAML::EndMap;
  }
  em << YAML::EndSeq;
}

void emit_mavlink(YAML::Emitter & em, const std::vector<MavlinkEntry> & entries, const char * key)
{
  em << YAML::Value << YAML::BeginSeq;
  for (const auto & s : entries) {
    em << YAML::BeginMap;
    emit_kv_str(em, key, s.name);
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
    if (!s.description.empty()) {emit_kv_str(em, "description", s.description);}
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
    if (p.plugin.empty()) {continue;}
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
    emit_mavlink(em, p.mavlink_subscriptions, "handler");
    em << YAML::Key << YAML::DoubleQuoted << "mavlink_publications";
    emit_mavlink(em, p.mavlink_publications, "argument");
    em << YAML::EndMap;
  }
  em << YAML::EndSeq;
  std::ofstream os(output);
  os << em.c_str() << "\n";
}

// --------------------------------------------------------------------------
// main
// --------------------------------------------------------------------------

static llvm::cl::opt<std::string> ClOutput(
  "output", llvm::cl::desc("Output JSON file"), llvm::cl::init(""));
static llvm::cl::list<std::string> ClPluginDir("plugin-dir", llvm::cl::desc("Plugin source dir"));
static llvm::cl::list<std::string> ClPlugin("plugin", llvm::cl::desc("Only these plugins"));
static llvm::cl::opt<int> ClJobs("jobs", llvm::cl::desc("Worker threads"), llvm::cl::init(4));
static llvm::cl::opt<std::string> ClCompileCommandsDir(
  "compile-commands-dir", llvm::cl::desc("Directory containing compile_commands.json"),
  llvm::cl::init(""));

fs::path detect_repo_root()
{
  fs::path cwd = fs::current_path();
  for (fs::path p = cwd; !p.empty(); p = p.parent_path()) {
    if (fs::exists(p / "mavros/src/plugins") && fs::exists(p / "mavros_extras/src/plugins")) {
      return p;
    }
    if (p == p.root_path()) {break;}
  }
  return cwd;
}

int main(int argc, char ** argv)
{
  llvm::cl::ParseCommandLineOptions(argc, argv, "MAVROS plugin documentation extractor");
  llvm::cl::SetVersionPrinter([](llvm::raw_ostream & os) {
      os << "plugin_doc_extract (clang-tooling)\n";
  });

  const fs::path repo_root = detect_repo_root();
  const auto msgid_index = build_mavlink_msgid_index(repo_root);

  std::vector<fs::path> plugin_dirs = {
    repo_root / "mavros/src/plugins", repo_root / "mavros_extras/src/plugins"};
  if (!ClPluginDir.empty()) {
    plugin_dirs.clear();
    for (const auto & d : ClPluginDir) {
      plugin_dirs.emplace_back(d);
    }
  }
  std::set<std::string> plugin_filter(ClPlugin.begin(), ClPlugin.end());

  // Collect plugin source files (those with an @plugin comment).
  std::vector<std::string> files;
  for (const auto & dir : plugin_dirs) {
    if (!fs::exists(dir)) {continue;}
    for (const auto & ent : fs::directory_iterator(dir)) {
      if (!ent.is_regular_file() || ent.path().extension() != ".cpp") {continue;}
      const std::string raw = read_file(ent.path());
      std::smatch m;
      if (!std::regex_search(raw, m, std::regex(R"(@plugin\s+([a-z0-9_]+))"))) {continue;}
      if (!plugin_filter.empty() && !plugin_filter.count(m[1].str())) {continue;}
      files.push_back(ent.path().string());
    }
  }
  std::sort(files.begin(), files.end());

  // Load the compilation database (flags per source file).
  std::string error;
  std::unique_ptr<tooling::CompilationDatabase> db;
  if (!ClCompileCommandsDir.empty()) {
    db = tooling::CompilationDatabase::loadFromDirectory(ClCompileCommandsDir, error);
  }
  if (!db) {
    llvm::errs() << "No compilation database found"
                 << (error.empty() ? "." : (": " + error)) << "\n";
    return 1;
  }

  // Append the MissionBase implementation so MissionBase-derived plugins
  // (waypoint, geofence, rallypoint) inherit its MAVLink traffic.
  const fs::path mission_base_cpp = repo_root / "mavros/src/plugins/mission_protocol_base.cpp";
  if (fs::exists(mission_base_cpp)) {
    files.push_back(mission_base_cpp.string());
  }

  std::map<std::string, PluginApi *> file_to_api;
  std::vector<PluginApi> apis(files.size());
  for (size_t i = 0; i < files.size(); ++i) {
    apis[i].file = files[i];
    apis[i].path = fs::path(files[i]);
    file_to_api[files[i]] = &apis[i];
  }

  tooling::ClangTool tool(*db, files);

  class Factory : public tooling::FrontendActionFactory
  {
public:
    Factory(
      std::map<std::string, PluginApi *> & m, const std::map<std::string, int> & idx)
    : m_(m), idx_(idx) {}

    std::unique_ptr<FrontendAction> create() override
    {
      return std::make_unique<PluginAction>(m_, idx_);
    }

private:
    std::map<std::string, PluginApi *> & m_;
    const std::map<std::string, int> & idx_;
  };

  Factory factory(file_to_api, msgid_index);
  const int rc = tool.run(&factory);

  // Post-process: inherit MAVLink traffic from MissionBase and setpoint mixins.
  PluginApi * base_api = nullptr;
  if (fs::exists(mission_base_cpp)) {
    base_api = file_to_api[mission_base_cpp.string()];
  }
  for (auto & api : apis) {
    if (api.plugin.empty()) {continue;}
    const bool is_mission =
      std::find(api.plugin_bases.begin(), api.plugin_bases.end(), "MissionBase") !=
      api.plugin_bases.end();
    if (is_mission && base_api) {
      api.mavlink_subscriptions = base_api->mavlink_subscriptions;
      api.mavlink_publications = base_api->mavlink_publications;
    }
    if (is_mission) {
      // MissionBase handlers are defined inline in the header, so they are not
      // visible when parsing the .cpp; add the known set here.
      static const std::map<std::string, std::string> kMissionSubs = {
        {"handle_mission_item", "mavlink::common::msg::MISSION_ITEM"},
        {"handle_mission_item_int", "mavlink::common::msg::MISSION_ITEM_INT"},
        {"handle_mission_request", "mavlink::common::msg::MISSION_REQUEST"},
        {"handle_mission_request_int", "mavlink::common::msg::MISSION_REQUEST_INT"},
        {"handle_mission_count", "mavlink::common::msg::MISSION_COUNT"},
        {"handle_mission_ack", "mavlink::common::msg::MISSION_ACK"},
        {"handle_mission_current", "mavlink::common::msg::MISSION_CURRENT"},
        {"handle_mission_item_reached", "mavlink::common::msg::MISSION_ITEM_REACHED"},
      };
      const bool not_waypoint = api.plugin != "waypoint";
      // MISSION_ITEM(_INT) are sent via the inline send_waypoint template.
      for (const auto & wp_type : {"mavlink::common::msg::MISSION_ITEM",
          "mavlink::common::msg::MISSION_ITEM_INT"})
      {
        MavlinkEntry e;
        e.message_type = wp_type;
        e.message_name = tail_name(wp_type);
        e.msg_id_expr = std::string(wp_type) + "::MSG_ID";
        e.name = "wpi";
        resolve_mavlink_meta(e, msgid_index);
        api.mavlink_publications.push_back(e);
      }
      for (const auto & [handler, type] : kMissionSubs) {
        if (not_waypoint && (handler == "handle_mission_current" ||
          handler == "handle_mission_item_reached"))
        {
          continue;   // geofence/rallypoint mission types have no CURRENT/REACHED
        }
        bool dup = false;
        for (const auto & s : api.mavlink_subscriptions) {
          if (s.name == handler) {dup = true; break;}
        }
        if (dup) {continue;}
        MavlinkEntry e;
        e.name = handler;
        e.message_type = type;
        e.message_name = tail_name(type);
        e.msg_id_expr = type + "::MSG_ID";
        resolve_mavlink_meta(e, msgid_index);
        api.mavlink_subscriptions.push_back(std::move(e));
      }
    }
    // Setpoint mixin publications (a plugin may use several mixins).
    std::vector<std::string> mixin_msgs;
    for (const auto & b : api.plugin_bases) {
      if (b.find("SetPositionTargetLocalNEDMixin") != std::string::npos) {
        mixin_msgs.push_back("SET_POSITION_TARGET_LOCAL_NED");
      } else if (b.find("SetPositionTargetGlobalIntMixin") != std::string::npos) {
        mixin_msgs.push_back("SET_POSITION_TARGET_GLOBAL_INT");
      } else if (b.find("SetAttitudeTargetMixin") != std::string::npos) {
        mixin_msgs.push_back("SET_ATTITUDE_TARGET");
      }
    }
    for (const auto & mixin_msg : mixin_msgs) {
      MavlinkEntry e;
      e.message_type = "mavlink::common::msg::" + mixin_msg;
      e.message_name = mixin_msg;
      e.msg_id_expr = e.message_type + "::MSG_ID";
      e.name = "msg";
      resolve_mavlink_meta(e, msgid_index);
      api.mavlink_publications.push_back(e);
    }
    // De-duplicate publications by message name.
    std::sort(
      api.mavlink_publications.begin(), api.mavlink_publications.end(),
      [](const MavlinkEntry & a, const MavlinkEntry & b) {
        return a.message_name < b.message_name;
      });
    api.mavlink_publications.erase(
      std::unique(
        api.mavlink_publications.begin(), api.mavlink_publications.end(),
        [](const MavlinkEntry & a, const MavlinkEntry & b) {
          return a.message_name == b.message_name;
        }),
      api.mavlink_publications.end());
  }

  std::vector<PluginApi> out;
  for (auto & api : apis) {
    if (!api.plugin.empty()) {
      out.push_back(std::move(api));
    }
  }
  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) {
      return a.plugin < b.plugin;
  });

  fs::path output = ClOutput.empty() ? fs::path("plugin_api.json") : fs::path(ClOutput.getValue());
  fs::create_directories(output.parent_path());
  write_json(out, output);
  std::cerr << "Generated " << out.size() << " plugins into " << output << "\n";
  (void)rc;
  return 0;
}
