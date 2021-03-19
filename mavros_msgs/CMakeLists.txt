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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# include_directories(include)

set(msg_files
  # [[[cog:
  # from pathlib import Path
  # for f in sorted(Path('msg').glob('*.msg')):
  #     cog.outl(f'"{f}"')
  # ]]]
  "msg/ADSBVehicle.msg"
  "msg/ActuatorControl.msg"
  "msg/Altitude.msg"
  "msg/AttitudeTarget.msg"
  "msg/BatteryStatus.msg"
  "msg/CamIMUStamp.msg"
  "msg/CommandCode.msg"
  "msg/CompanionProcessStatus.msg"
  "msg/DebugValue.msg"
  "msg/ESCInfo.msg"
  "msg/ESCInfoItem.msg"
  "msg/ESCStatus.msg"
  "msg/ESCStatusItem.msg"
  "msg/EstimatorStatus.msg"
  "msg/ExtendedState.msg"
  "msg/FileEntry.msg"
  "msg/GPSRAW.msg"
  "msg/GPSRTK.msg"
  "msg/GlobalPositionTarget.msg"
  "msg/HilActuatorControls.msg"
  "msg/HilControls.msg"
  "msg/HilGPS.msg"
  "msg/HilSensor.msg"
  "msg/HilStateQuaternion.msg"
  "msg/HomePosition.msg"
  "msg/LandingTarget.msg"
  "msg/LogData.msg"
  "msg/LogEntry.msg"
  "msg/ManualControl.msg"
  "msg/Mavlink.msg"
  "msg/MountControl.msg"
  "msg/OnboardComputerStatus.msg"
  "msg/OpticalFlowRad.msg"
  "msg/OverrideRCIn.msg"
  "msg/Param.msg"
  "msg/ParamValue.msg"
  "msg/PlayTuneV2.msg"
  "msg/PositionTarget.msg"
  "msg/RCIn.msg"
  "msg/RCOut.msg"
  "msg/RTCM.msg"
  "msg/RTKBaseline.msg"
  "msg/RadioStatus.msg"
  "msg/State.msg"
  "msg/StatusText.msg"
  "msg/Thrust.msg"
  "msg/TimesyncStatus.msg"
  "msg/Trajectory.msg"
  "msg/VehicleInfo.msg"
  "msg/VfrHud.msg"
  "msg/Vibration.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/WaypointReached.msg"
  "msg/WheelOdomStamped.msg"
  # [[[end]]] (checksum: 70caaa5138f6ac84b4510b1eaf78aa51)
)

set(srv_files
  # [[[cog:
  # for f in sorted(Path('srv').glob('*.srv')):
  #     cog.outl(f'"{f}"')
  # ]]]
  "srv/CommandBool.srv"
  "srv/CommandHome.srv"
  "srv/CommandInt.srv"
  "srv/CommandLong.srv"
  "srv/CommandTOL.srv"
  "srv/CommandTriggerControl.srv"
  "srv/CommandTriggerInterval.srv"
  "srv/CommandVtolTransition.srv"
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
  "srv/MountConfigure.srv"
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
  # [[[end]]] (checksum: 9506ecb4b8f5aca4be29f1f4d30569f4)
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

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # NOTE(vooon): Does not support our custom triple-license, tiered to make it to work.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# vim: ts=2 sw=2 et:
