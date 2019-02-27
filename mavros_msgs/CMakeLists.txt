cmake_minimum_required(VERSION 3.5)
project(mavros_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# include_directories(include)

set(msg_files
  "msg/ADSBVehicle.msg"
  "msg/ActuatorControl.msg"
  "msg/Altitude.msg"
  "msg/AttitudeTarget.msg"
  "msg/BatteryStatus.msg"
  "msg/CamIMUStamp.msg"
  "msg/CommandCode.msg"
  "msg/CompanionProcessStatus.msg"
  "msg/DebugValue.msg"
  "msg/ExtendedState.msg"
  "msg/FileEntry.msg"
  "msg/GlobalPositionTarget.msg"
  "msg/HilActuatorControls.msg"
  "msg/HilControls.msg"
  "msg/HilGPS.msg"
  "msg/HilSensor.msg"
  "msg/HilStateQuaternion.msg"
  "msg/HomePosition.msg"
  "msg/LogData.msg"
  "msg/LogEntry.msg"
  "msg/ManualControl.msg"
  "msg/Mavlink.msg"
  "msg/OpticalFlowRad.msg"
  "msg/OverrideRCIn.msg"
  "msg/Param.msg"
  "msg/ParamValue.msg"
  "msg/PositionTarget.msg"
  "msg/RCIn.msg"
  "msg/RCOut.msg"
  "msg/RTCM.msg"
  "msg/RadioStatus.msg"
  "msg/State.msg"
  "msg/StatusText.msg"
  "msg/Thrust.msg"
  "msg/TimesyncStatus.msg"
  "msg/Trajectory.msg"
  "msg/VfrHud.msg"
  "msg/VehicleInfo.msg"
  "msg/Vibration.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/WaypointReached.msg"
  "msg/WheelOdomStamped.msg"
)

set(srv_files
  "srv/CommandBool.srv"
  "srv/CommandBool.srv"
  "srv/CommandHome.srv"
  "srv/CommandInt.srv"
  "srv/CommandLong.srv"
  "srv/CommandTOL.srv"
  "srv/CommandTriggerControl.srv"
  "srv/CommandTriggerInterval.srv"
  "srv/FileChecksum.srv"
  "srv/FileClose.srv"
  "srv/FileList.srv"
  "srv/FileMakeDir.srv"
  "srv/FileOpen.srv"
  "srv/FileRead.srv"
  "srv/FileRemove.srv"
  "srv/FileRemoveDir.srv"
  "srv/FileRename.srv"
  "srv/FileTruncate.srv"
  "srv/FileWrite.srv"
  "srv/LogRequestData.srv"
  "srv/LogRequestEnd.srv"
  "srv/LogRequestList.srv"
  "srv/MessageInterval.srv"
  "srv/ParamGet.srv"
  "srv/ParamPull.srv"
  "srv/ParamPush.srv"
  "srv/ParamSet.srv"
  "srv/SetMavFrame.srv"
  "srv/SetMode.srv"
  "srv/StreamRate.srv"
  "srv/VehicleInfoGet.srv"
  "srv/WaypointClear.srv"
  "srv/WaypointPull.srv"
  "srv/WaypointPush.srv"
  "srv/WaypointSetCurrent.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  builtin_interfaces
  geographic_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES mavros_msgs_mapping_rule.yaml
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
