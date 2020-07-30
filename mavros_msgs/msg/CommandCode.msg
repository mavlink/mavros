# MAV_CMD command codes.
# Actual meaning and params you may find in MAVLink documentation
# https://mavlink.io/en/messages/common.html#MAV_CMD

# [[[cog:
# from pymavlink.dialects.v20 import common
# from collections import OrderedDict
# import re
#
# def wr_enum(enum, ename, pfx='', bsz=16):
#     cog.outl("# " + ename + "_" + pfx)
#     for k, e in enum:
#         # exclude also deprecated commands
#         if 'MAV_CMD' + "_" + pfx in e.name and not re.search('deprecated', e.description, re.IGNORECASE):
#             sn = e.name[len('MAV_CMD') + 1:]
#             l = "uint{bsz} {sn} = {k}".format(**locals())
#             if e.description:
#                 l += ' ' * (50 - len(l)) + ' # ' + e.description
#             cog.outl(l)
#     cog.out('\n')
#
# def decl_enum(ename):
#     enum = sorted(common.enums[ename].items())
#     enum.pop() # remove ENUM_END
#
#     enumt = []
#     # exception list of commands to not include
#     exlist = ['SPATIAL', 'USER', 'WAYPOINT']
#     for k, e in enum:
#         enumt.extend(e.name[len(ename) + 1:].split('_')[0:1])
#
#     enumt = sorted(set(enumt))
#     enumt = [word for word in enumt if word not in exlist]
#
#     for key in enumt:
#         wr_enum(enum, ename, key)
#
# decl_enum('MAV_CMD')
# ]]]
# MAV_CMD_AIRFRAME
uint16 AIRFRAME_CONFIGURATION = 2520

# MAV_CMD_ARM
uint16 ARM_AUTHORIZATION_REQUEST = 3001            # Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
        

# MAV_CMD_COMPONENT
uint16 COMPONENT_ARM_DISARM = 400                  # Arms / Disarms a component

# MAV_CMD_CONDITION
uint16 CONDITION_DELAY = 112                       # Delay mission state machine.
uint16 CONDITION_CHANGE_ALT = 113                  # Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
uint16 CONDITION_DISTANCE = 114                    # Delay mission state machine until within desired distance of next NAV point.
uint16 CONDITION_YAW = 115                         # Reach a certain target angle.
uint16 CONDITION_LAST = 159                        # NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration

# MAV_CMD_CONTROL
uint16 CONTROL_HIGH_LATENCY = 2600                 # Request to start/stop transmitting over the high latency telemetry

# MAV_CMD_DO
uint16 DO_FOLLOW = 32                              # Being following a target
uint16 DO_FOLLOW_REPOSITION = 33                   # Reposition the MAV after a follow target command has been sent
uint16 DO_SET_MODE = 176                           # Set system mode.
uint16 DO_JUMP = 177                               # Jump to the desired command in the mission list.  Repeat this action only the specified number of times
uint16 DO_CHANGE_SPEED = 178                       # Change speed and/or throttle set points.
uint16 DO_SET_HOME = 179                           # Changes the home location either to the current location or a specified location.
uint16 DO_SET_PARAMETER = 180                      # Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
uint16 DO_SET_RELAY = 181                          # Set a relay to a condition.
uint16 DO_REPEAT_RELAY = 182                       # Cycle a relay on and off for a desired number of cycles with a desired period.
uint16 DO_SET_SERVO = 183                          # Set a servo to a desired PWM value.
uint16 DO_REPEAT_SERVO = 184                       # Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
uint16 DO_FLIGHTTERMINATION = 185                  # Terminate flight immediately
uint16 DO_CHANGE_ALTITUDE = 186                    # Change altitude set point.
uint16 DO_LAND_START = 189                         # Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
uint16 DO_RALLY_LAND = 190                         # Mission command to perform a landing from a rally point.
uint16 DO_GO_AROUND = 191                          # Mission command to safely abort an autonomous landing.
uint16 DO_REPOSITION = 192                         # Reposition the vehicle to a specific WGS84 global position.
uint16 DO_PAUSE_CONTINUE = 193                     # If in a GPS controlled position mode, hold the current position or continue.
uint16 DO_SET_REVERSE = 194                        # Set moving direction to forward or reverse.
uint16 DO_SET_ROI_LOCATION = 195                   # Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
uint16 DO_SET_ROI_WPNEXT_OFFSET = 196              # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
uint16 DO_SET_ROI_NONE = 197                       # Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
uint16 DO_CONTROL_VIDEO = 200                      # Control onboard camera system.
uint16 DO_SET_ROI = 201                            # Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
uint16 DO_DIGICAM_CONFIGURE = 202                  # Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
uint16 DO_DIGICAM_CONTROL = 203                    # Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
uint16 DO_MOUNT_CONFIGURE = 204                    # Mission command to configure a camera or antenna mount
uint16 DO_MOUNT_CONTROL = 205                      # Mission command to control a camera or antenna mount
uint16 DO_SET_CAM_TRIGG_DIST = 206                 # Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
uint16 DO_FENCE_ENABLE = 207                       # Mission command to enable the geofence
uint16 DO_PARACHUTE = 208                          # Mission command to trigger a parachute
uint16 DO_MOTOR_TEST = 209                         # Mission command to perform motor test.
uint16 DO_INVERTED_FLIGHT = 210                    # Change to/from inverted flight.
uint16 DO_SET_CAM_TRIGG_INTERVAL = 214             # Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
uint16 DO_MOUNT_CONTROL_QUAT = 220                 # Mission command to control a camera or antenna mount, using a quaternion as reference.
uint16 DO_GUIDED_MASTER = 221                      # set id of master controller
uint16 DO_GUIDED_LIMITS = 222                      # Set limits for external control
uint16 DO_ENGINE_CONTROL = 223                     # Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
uint16 DO_SET_MISSION_CURRENT = 224                # Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
uint16 DO_LAST = 240                               # NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
uint16 DO_JUMP_TAG = 601                           # Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.
uint16 DO_TRIGGER_CONTROL = 2003                   # Enable or disable on-board camera triggering system.
uint16 DO_VTOL_TRANSITION = 3000                   # Request VTOL transition

# MAV_CMD_GET
uint16 GET_HOME_POSITION = 410                     # Request the home position from the vehicle.
uint16 GET_MESSAGE_INTERVAL = 510                  # Request the interval between messages for a particular MAVLink message ID

# MAV_CMD_IMAGE
uint16 IMAGE_START_CAPTURE = 2000                  # Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.
uint16 IMAGE_STOP_CAPTURE = 2001                   # Stop image capture sequence Use NaN for reserved values.

# MAV_CMD_JUMP
uint16 JUMP_TAG = 600                              # Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.

# MAV_CMD_LOGGING
uint16 LOGGING_START = 2510                        # Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
uint16 LOGGING_STOP = 2511                         # Request to stop streaming log data over MAVLink

# MAV_CMD_MISSION
uint16 MISSION_START = 300                         # start running a mission

# MAV_CMD_NAV
uint16 NAV_WAYPOINT = 16                           # Navigate to waypoint.
uint16 NAV_LOITER_UNLIM = 17                       # Loiter around this waypoint an unlimited amount of time
uint16 NAV_LOITER_TURNS = 18                       # Loiter around this waypoint for X turns
uint16 NAV_LOITER_TIME = 19                        # Loiter around this waypoint for X seconds
uint16 NAV_RETURN_TO_LAUNCH = 20                   # Return to launch location
uint16 NAV_LAND = 21                               # Land at location.
uint16 NAV_TAKEOFF = 22                            # Takeoff from ground / hand
uint16 NAV_LAND_LOCAL = 23                         # Land at local position (local frame only)
uint16 NAV_TAKEOFF_LOCAL = 24                      # Takeoff from local position (local frame only)
uint16 NAV_FOLLOW = 25                             # Vehicle following, i.e. this waypoint represents the position of a moving vehicle
uint16 NAV_CONTINUE_AND_CHANGE_ALT = 30            # Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
uint16 NAV_LOITER_TO_ALT = 31                      # Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.
uint16 NAV_ROI = 80                                # Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
uint16 NAV_PATHPLANNING = 81                       # Control autonomous path planning on the MAV.
uint16 NAV_SPLINE_WAYPOINT = 82                    # Navigate to waypoint using a spline path.
uint16 NAV_VTOL_TAKEOFF = 84                       # Takeoff from ground using VTOL mode, and transition to forward flight with specified heading.
uint16 NAV_VTOL_LAND = 85                          # Land using VTOL mode
uint16 NAV_GUIDED_ENABLE = 92                      # hand control over to an external controller
uint16 NAV_DELAY = 93                              # Delay the next navigation command a number of seconds or until a specified time
uint16 NAV_PAYLOAD_PLACE = 94                      # Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.
uint16 NAV_LAST = 95                               # NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
uint16 NAV_SET_YAW_SPEED = 213                     # Sets a desired vehicle turn angle and speed change.
uint16 NAV_FENCE_RETURN_POINT = 5000               # Fence return point. There can only be one fence return point.
        
uint16 NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001   # Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
        
uint16 NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002   # Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
        
uint16 NAV_FENCE_CIRCLE_INCLUSION = 5003           # Circular fence area. The vehicle must stay inside this area.
        
uint16 NAV_FENCE_CIRCLE_EXCLUSION = 5004           # Circular fence area. The vehicle must stay outside this area.
        
uint16 NAV_RALLY_POINT = 5100                      # Rally point. You can have multiple rally points defined.
        

# MAV_CMD_OVERRIDE
uint16 OVERRIDE_GOTO = 252                         # Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.

# MAV_CMD_PANORAMA
uint16 PANORAMA_CREATE = 2800                      # Create a panorama at the current position

# MAV_CMD_PAYLOAD
uint16 PAYLOAD_PREPARE_DEPLOY = 30001              # Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
uint16 PAYLOAD_CONTROL_DEPLOY = 30002              # Control the payload deployment.

# MAV_CMD_PREFLIGHT
uint16 PREFLIGHT_CALIBRATION = 241                 # Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
uint16 PREFLIGHT_SET_SENSOR_OFFSETS = 242          # Set sensor offsets. This command will be only accepted if in pre-flight mode.
uint16 PREFLIGHT_UAVCAN = 243                      # Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
uint16 PREFLIGHT_STORAGE = 245                     # Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
uint16 PREFLIGHT_REBOOT_SHUTDOWN = 246             # Request the reboot or shutdown of system components.

# MAV_CMD_REQUEST
uint16 REQUEST_MESSAGE = 512                       # Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
uint16 REQUEST_AUTOPILOT_CAPABILITIES = 520        # Request autopilot capabilities
uint16 REQUEST_CAMERA_INFORMATION = 521            # Request camera information (CAMERA_INFORMATION).
uint16 REQUEST_CAMERA_SETTINGS = 522               # Request camera settings (CAMERA_SETTINGS).
uint16 REQUEST_STORAGE_INFORMATION = 525           # Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
uint16 REQUEST_CAMERA_CAPTURE_STATUS = 527         # Request camera capture status (CAMERA_CAPTURE_STATUS)
uint16 REQUEST_FLIGHT_INFORMATION = 528            # Request flight information (FLIGHT_INFORMATION)

# MAV_CMD_RESET
uint16 RESET_CAMERA_SETTINGS = 529                 # Reset all camera settings to Factory Default

# MAV_CMD_SET
uint16 SET_MESSAGE_INTERVAL = 511                  # Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM
uint16 SET_CAMERA_MODE = 530                       # Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.
uint16 SET_GUIDED_SUBMODE_STANDARD = 4000          # This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
                  
uint16 SET_GUIDED_SUBMODE_CIRCLE = 4001            # This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                  

# MAV_CMD_START
uint16 START_RX_PAIR = 500                         # Starts receiver pairing.

# MAV_CMD_STORAGE
uint16 STORAGE_FORMAT = 526                        # Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.

# MAV_CMD_UAVCAN
uint16 UAVCAN_GET_NODE_INFO = 5200                 # Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.

# MAV_CMD_VIDEO
uint16 VIDEO_START_CAPTURE = 2500                  # Starts video capture (recording). Use NaN for reserved values.
uint16 VIDEO_STOP_CAPTURE = 2501                   # Stop the current video capture (recording). Use NaN for reserved values.

# [[[end]]] (checksum: 6546a8ab3dac44945e7bbfadae5b0d6f)
