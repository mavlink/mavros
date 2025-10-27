cmake_minimum_required(VERSION 3.8)
project(mavros_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Add Python node waypoint_mission_manager
install(PROGRAMS
  ${PROJECT_NAME}/waypoint_mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_mission_manager
)

# Add Python node task_control
install(PROGRAMS
  ${PROJECT_NAME}/flight_task_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME flight_task_control
)

if(BUILD_TESTING)
  # NOTE(vooon): Does not support our custom triple-license, tiered to make it to work.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# vim: ts=2 sw=2 et:
