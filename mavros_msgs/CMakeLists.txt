cmake_minimum_required(VERSION 2.8.3)
project(mavros_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs geographic_msgs message_generation std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  ADSBVehicle.msg
  ActuatorControl.msg
  Altitude.msg
  AttitudeTarget.msg
  BatteryStatus.msg
  CamIMUStamp.msg
  CommandCode.msg
  DebugValue.msg
  ExtendedState.msg
  FileEntry.msg
  GlobalPositionTarget.msg
  HilActuatorControls.msg
  HilControls.msg
  HilGPS.msg
  HilSensor.msg
  HilStateQuaternion.msg
  HomePosition.msg
  LogData.msg
  LogEntry.msg
  ManualControl.msg
  Mavlink.msg
  OpticalFlowRad.msg
  OverrideRCIn.msg
  ParamValue.msg
  PositionTarget.msg
  RCIn.msg
  RCOut.msg
  RadioStatus.msg
  RTCM.msg
  State.msg
  StatusText.msg
  Thrust.msg
  Trajectory.msg
  TimesyncStatus.msg
  VFR_HUD.msg
  Vibration.msg
  Waypoint.msg
  WaypointList.msg
  WaypointReached.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  CommandBool.srv
  CommandHome.srv
  CommandInt.srv
  CommandLong.srv
  CommandTOL.srv
  CommandTriggerControl.srv
  FileChecksum.srv
  FileClose.srv
  FileList.srv
  FileMakeDir.srv
  FileOpen.srv
  FileRead.srv
  FileRemove.srv
  FileRemoveDir.srv
  FileRename.srv
  FileTruncate.srv
  FileWrite.srv
  LogRequestData.srv
  LogRequestEnd.srv
  LogRequestList.srv
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  ParamSet.srv
  SetMavFrame.srv
  SetMode.srv
  StreamRate.srv
  WaypointClear.srv
  WaypointPull.srv
  WaypointPush.srv
  WaypointSetCurrent.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs geographic_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs sensor_msgs geographic_msgs message_runtime std_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
