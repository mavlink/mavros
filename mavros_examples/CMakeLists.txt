cmake_minimum_required(VERSION 3.8)
project(mavros_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

find_package(rclpy REQUIRED)

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

ament_package()
