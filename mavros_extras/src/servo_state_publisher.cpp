/*
 * Copyright 2013,2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS SSP Node
 * @file
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <memory>

#include "mavros/servo_state_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * Servo State Publisher Node (component loader)
 * to work similar to mavros v1.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec();

  rclcpp::NodeOptions options;

  RCLCPP_INFO(node->get_logger(), "Starting mavros servo_state_publisher node");
  auto ssp_node = std::make_shared<mavros::extras::ServoStatePublisher>(options);
  exec.add_node(ssp_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
