cmake_minimum_required(VERSION 2.8.3)
project(mavros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Mavlink.msg
  Waypoint.msg
  WaypointList.msg
  BatteryStatus.msg
  State.msg
  RCIn.msg
  RCOut.msg
  RadioStatus.msg
  OverrideRCIn.msg
  VFR_HUD.msg
  FileEntry.msg
  ActuatorControl.msg
  CommandCode.msg
  ParamValue.msg
  OpticalFlowRad.msg
  Vibration.msg
  CamIMUStamp.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ParamSet.srv
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  WaypointSetCurrent.srv
  WaypointClear.srv
  WaypointPull.srv
  WaypointPush.srv
  WaypointGOTO.srv
  CommandLong.srv
  CommandInt.srv
  CommandBool.srv
  CommandHome.srv
  CommandTOL.srv
  CommandTriggerControl.srv
  StreamRate.srv
  SetMode.srv
  FileOpen.srv
  FileClose.srv
  FileRead.srv
  FileWrite.srv
  FileList.srv
  FileRemove.srv
  FileMakeDir.srv
  FileRemoveDir.srv
  FileTruncate.srv
  FileRename.srv
  FileChecksum.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
