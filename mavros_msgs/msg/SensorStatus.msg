# The controllers and sensors state
#
# https://mavlink.io/en/messages/common.html#SYS_STATUS


# [[[cog:
# from pymavlink.dialects.v20 import common
#
# def decl_enum(ename, mav_pfx='', pfx='', bsz=8):
#     enum = sorted(common.enums[ename].items())
#     enum.pop() # remove ENUM_END
#
#     cog.outl("# " + ename)
#     for k, e in enum:
#         sn = e.name[len(mav_pfx) + 1:]
#         l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
#         if e.description:
#             l += ' ' * (40 - len(l)) + ' # ' + e.description
#         cog.outl(l)
#
# decl_enum('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS', '', 32)
# ]]]
# MAV_SYS_STATUS_SENSOR
uint32 SENSOR_3D_GYRO = 1                # 0x01 3D gyro
uint32 SENSOR_3D_ACCEL = 2               # 0x02 3D accelerometer
uint32 SENSOR_3D_MAG = 4                 # 0x04 3D magnetometer
uint32 SENSOR_ABSOLUTE_PRESSURE = 8      # 0x08 absolute pressure
uint32 SENSOR_DIFFERENTIAL_PRESSURE = 16 # 0x10 differential pressure
uint32 SENSOR_GPS = 32                   # 0x20 GPS
uint32 SENSOR_OPTICAL_FLOW = 64          # 0x40 optical flow
uint32 SENSOR_VISION_POSITION = 128      # 0x80 computer vision position
uint32 SENSOR_LASER_POSITION = 256       # 0x100 laser based position
uint32 SENSOR_EXTERNAL_GROUND_TRUTH = 512 # 0x200 external ground truth (Vicon or Leica)
uint32 SENSOR_ANGULAR_RATE_CONTROL = 1024 # 0x400 3D angular rate control
uint32 SENSOR_ATTITUDE_STABILIZATION = 2048 # 0x800 attitude stabilization
uint32 SENSOR_YAW_POSITION = 4096        # 0x1000 yaw position
uint32 SENSOR_Z_ALTITUDE_CONTROL = 8192  # 0x2000 z/altitude control
uint32 SENSOR_XY_POSITION_CONTROL = 16384 # 0x4000 x/y position control
uint32 SENSOR_MOTOR_OUTPUTS = 32768      # 0x8000 motor outputs / control
uint32 SENSOR_RC_RECEIVER = 65536        # 0x10000 rc receiver
uint32 SENSOR_3D_GYRO2 = 131072          # 0x20000 2nd 3D gyro
uint32 SENSOR_3D_ACCEL2 = 262144         # 0x40000 2nd 3D accelerometer
uint32 SENSOR_3D_MAG2 = 524288           # 0x80000 2nd 3D magnetometer
uint32 GEOFENCE = 1048576                # 0x100000 geofence
uint32 AHRS = 2097152                    # 0x200000 AHRS subsystem health
uint32 TERRAIN = 4194304                 # 0x400000 Terrain subsystem health
uint32 REVERSE_MOTOR = 8388608           # 0x800000 Motors are reversed
uint32 LOGGING = 16777216                # 0x1000000 Logging
uint32 SENSOR_BATTERY = 33554432         # 0x2000000 Battery
uint32 SENSOR_PROXIMITY = 67108864       # 0x4000000 Proximity
uint32 SENSOR_SATCOM = 134217728         # 0x8000000 Satellite Communication 
uint32 PREARM_CHECK = 268435456          # 0x10000000 pre-arm check status. Always healthy when armed
uint32 OBSTACLE_AVOIDANCE = 536870912    # 0x20000000 Avoidance/collision prevention
uint32 SENSOR_PROPULSION = 1073741824    # 0x40000000 propulsion (actuator, esc, motor or propellor)
# [[[end]]] (checksum: d03f9e5f8b03b3cec98c0c3a97272d16)

std_msgs/Header header

uint32 sensors_present
uint32 sensors_enabled
uint32 sensors_health