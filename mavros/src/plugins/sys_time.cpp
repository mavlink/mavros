    node_declare_and_watch_parameter(
      "system_time_rate", 0.0, [&](const rclcpp::Parameter & p) {
        auto rate_d = p.as_double();

        if (rate_d <= 0.0) {
          if (sys_time_timer) {
            sys_time_timer->cancel();
            sys_time_timer.reset();
          }
        } else {
          auto period = std::chrono::duration<double>(1.0 / rate_d);

          sys_time_timer =
          node->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SystemTimePlugin::sys_time_cb, this));
        }
      });

    node_declare_and_watch_parameter(
      "timesync_rate", 0.0, [&](const rclcpp::Parameter & p) {
        auto rate_d = p.as_double();

        if (rate_d <= 0.0) {
          if (timesync_timer) {
            timesync_timer->cancel();
            timesync_timer.reset();
            uas->diagnostic_updater.removeByName(dt_diag.getName());
          }
        } else {
          auto period = std::chrono::duration<double>(1.0 / rate_d);

          timesync_timer =
          node->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SystemTimePlugin::timesync_cb, this));

          uas->diagnostic_updater.add(dt_diag);
        }
      });