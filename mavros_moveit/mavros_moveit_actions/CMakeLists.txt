cmake_minimum_required(VERSION 2.8.3)
project(mavros_moveit_actions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	moveit_core 
	roscpp 
	actionlib 
	actionlib_msgs 
	geometry_msgs 
	mavros_msgs 
	trajectory_msgs 
	message_generation
	control_msgs)
	
find_package(Boost REQUIRED COMPONENTS system)
add_action_files(
  DIRECTORY action
  FILES FollowMultiDofJointTrajectory.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp actionlib_msgs mavros_msgs trajectory_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(follow_multi_dof_joint_trajectory_action_server src/follow_multi_dof_joint_trajectory_action_server.cpp)
target_link_libraries(follow_multi_dof_joint_trajectory_action_server ${catkin_LIBRARIES})

add_dependencies(
  follow_multi_dof_joint_trajectory_action_server
  ${mavros_moveit_actions_EXPORTED_TARGETS} mavros_msgs_generate_messages 
)
