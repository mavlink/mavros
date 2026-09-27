//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Regression test for issue #2294: dynamically created plugin sub-nodes must
 * still apply the process's parameter sources (--params-file / -p) even though
 * they are created with use_global_arguments(false) to keep process-global
 * __node/__ns remap rules from renaming them (issue #2262).
 *
 * main() initializes rclcpp with both a global node-name remap and a params
 * file, so a single test covers both halves: the plugin keeps its own name and
 * yet reads the values from the file.
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "test_plugin_helpers.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace mavros
{
namespace uas
{

#ifndef TEST_PLUGIN_PARAMS_YAML
#error "TEST_PLUGIN_PARAMS_YAML must point at the test parameters file"
#endif

//! Minimal plugin with a few parameters; subnode name "sys" mirrors the
//  shipped apm_config.yaml / px4_config.yaml `/**/sys:` section.
class ParamsPlugin : public plugin::Plugin
{
public:
  explicit ParamsPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "sys")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "heartbeat_rate", 1.0, [this](const rclcpp::Parameter & p) {
        heartbeat_rate = p.as_double();
      });
    node_declare_and_watch_parameter(
      "conn_timeout", 10.0, [this](const rclcpp::Parameter & p) {
        conn_timeout = p.as_double();
      });
    //! Not present in the params file: must keep the compiled-in default.
    node_declare_and_watch_parameter(
      "control_rate", 42.0, [this](const rclcpp::Parameter & p) {
        control_rate = p.as_double();
      });
  }

  Subscriptions get_subscriptions() override
  {
    return {};
  }

  double heartbeat_rate{1.0};
  double conn_timeout{10.0};
  double control_rate{42.0};
};

class PluginParamsTest : public TestUAS
{
};

TEST_F(PluginParamsTest, subnode_applies_process_params_file)
{
  auto uas = create_uas();
  auto plugin = std::make_shared<ParamsPlugin>(uas.get());
  auto node = plugin->get_node();

  // #2262 guard: the global `-r __node:=renamed_global` must not rename the
  // plugin sub-node.
  EXPECT_STREQ("sys", node->get_name());

  // #2294: values from the process `--params-file` must be applied.
  EXPECT_DOUBLE_EQ(7.0, node->get_parameter("heartbeat_rate").as_double());
  EXPECT_DOUBLE_EQ(33.0, node->get_parameter("conn_timeout").as_double());

  // A parameter absent from the file keeps its compiled-in default.
  EXPECT_DOUBLE_EQ(42.0, node->get_parameter("control_rate").as_double());
}

}   // namespace uas
}   // namespace mavros

int main(int argc, char ** argv)
{
  // Initialize rclcpp with a global remap rule (as a component container or
  // launch file would) and the parameter file (as node.launch does).
  const char * ros_args[] = {
    "mavros-plugin-params-test",
    "--ros-args",
    "-r", "__node:=renamed_global",
    "--params-file", TEST_PLUGIN_PARAMS_YAML,
  };
  rclcpp::init(
    static_cast<int>(sizeof(ros_args) / sizeof(const char *)),
    const_cast<char **>(ros_args));

  ::testing::InitGoogleTest(&argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
