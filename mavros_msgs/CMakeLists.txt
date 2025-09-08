cmake_minimum_required(VERSION 2.8.3)
project(mavros_msgs)

find_package(catkin REQUIRED COMPONENTS geographic_msgs geometry_msgs message_generation sensor_msgs std_msgs)

include_directories(include)

# [[[cog:
# import mavros_cog
# ]]]
# [[[end]]] (sum: 1B2M2Y8Asg)


add_message_files(
  DIRECTORY msg
  FILES
  # [[[cog:
  # mavros_cog.outl_glob_files('msg', '*.msg', basename_only=True)
  # ]]]
  ADSBVehicle.msg
  ActuatorControl.msg
  Altitude.msg
  AttitudeTarget.msg
  BatteryStatus.msg
  CamIMUStamp.msg
  CameraImageCaptured.msg
  CellularStatus.msg
  CommandCode.msg
  CompanionProcessStatus.msg
  DebugValue.msg
  ESCInfo.msg
  ESCInfoItem.msg
  ESCStatus.msg
  ESCStatusItem.msg
  ESCTelemetry.msg
  ESCTelemetryItem.msg
  EstimatorStatus.msg
  ExtendedState.msg
  FileEntry.msg
  GPSINPUT.msg
  GPSRAW.msg
  GPSRTK.msg
  GimbalDeviceAttitudeStatus.msg
  GimbalDeviceInformation.msg
  GimbalDeviceSetAttitude.msg
  GimbalManagerInformation.msg
  GimbalManagerSetAttitude.msg
  GimbalManagerSetPitchyaw.msg
  GimbalManagerStatus.msg
  GlobalPositionTarget.msg
  HilActuatorControls.msg
  HilControls.msg
  HilGPS.msg
  HilSensor.msg
  HilStateQuaternion.msg
  HomePosition.msg
  LandingTarget.msg
  LogData.msg
  LogEntry.msg
  MagnetometerReporter.msg
  ManualControl.msg
  Mavlink.msg
  MountControl.msg
  NavControllerOutput.msg
  OnboardComputerStatus.msg
  OpenDroneIDBasicID.msg
  OpenDroneIDOperatorID.msg
  OpenDroneIDSelfID.msg
  OpenDroneIDSystem.msg
  OpenDroneIDSystemUpdate.msg
  OpticalFlow.msg
  OpticalFlowRad.msg
  OverrideRCIn.msg
  Param.msg
  ParamValue.msg
  PlayTuneV2.msg
  PositionTarget.msg
  RCIn.msg
  RCOut.msg
  RTCM.msg
  RTKBaseline.msg
  RadioStatus.msg
  State.msg
  StatusEvent.msg
  StatusText.msg
  SysStatus.msg
  TerrainReport.msg
  Thrust.msg
  TimesyncStatus.msg
  Trajectory.msg
  Tunnel.msg
  VFR_HUD.msg
  VehicleInfo.msg
  Vibration.msg
  Waypoint.msg
  WaypointList.msg
  WaypointReached.msg
  WheelOdomStamped.msg
  # [[[end]]] (sum: H4Qd6NbCtE)
)

add_service_files(
  DIRECTORY srv
  FILES
  # [[[cog:
  # mavros_cog.outl_glob_files('srv', '*.srv', basename_only=True)
  # ]]]
  CommandAck.srv
  CommandBool.srv
  CommandHome.srv
  CommandInt.srv
  CommandLong.srv
  CommandTOL.srv
  CommandTOLLocal.srv
  CommandTriggerControl.srv
  CommandTriggerInterval.srv
  CommandVtolTransition.srv
  EndpointAdd.srv
  EndpointDel.srv
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
  GimbalGetInformation.srv
  GimbalManagerCameraTrack.srv
  GimbalManagerConfigure.srv
  GimbalManagerPitchyaw.srv
  GimbalManagerSetRoi.srv
  LogRequestData.srv
  LogRequestEnd.srv
  LogRequestList.srv
  MessageInterval.srv
  MountConfigure.srv
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  ParamSet.srv
  SetMavFrame.srv
  SetMode.srv
  StreamRate.srv
  VehicleInfoGet.srv
  WaypointClear.srv
  WaypointPull.srv
  WaypointPush.srv
  WaypointSetCurrent.srv
  # [[[end]]] (sum: MKkJeuPGSB)
)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geographic_msgs geometry_msgs message_runtime sensor_msgs std_msgs)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
