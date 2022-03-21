# Represent sensors status from SYS_STATUS
# https://mavlink.io/en/messages/common.html#SYS_STATUS

# [[[cog:
# from pymavlink.dialects.v20 import common
#
# def decl_enum(ename, pfx='', bsz=8):
#     enum = sorted(common.enums[ename].items())
#     enum.pop() # remove ENUM_END
#
#     cog.outl("# " + ename)
#     for k, e in enum:
#         sn = e.name[len(ename) - 7 + 1:]
#         l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
#         if e.description:
#             l += ' ' * (40 - len(l)) + ' # ' + e.description
#         cog.outl(l)
#
# decl_enum('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_', 32)
# ]]]
# MAV_SYS_STATUS_SENSOR
uint32 MAV_SYS_STATUS_SENSOR_3D_GYRO = 1 # 0x01 3D gyro
uint32 MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2 # 0x02 3D accelerometer
uint32 MAV_SYS_STATUS_SENSOR_3D_MAG = 4  # 0x04 3D magnetometer
uint32 MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8 # 0x08 absolute pressure
uint32 MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16 # 0x10 differential pressure
uint32 MAV_SYS_STATUS_SENSOR_GPS = 32    # 0x20 GPS
uint32 MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64 # 0x40 optical flow
uint32 MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128 # 0x80 computer vision position
uint32 MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256 # 0x100 laser based position
uint32 MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512 # 0x200 external ground truth (Vicon or Leica)
uint32 MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024 # 0x400 3D angular rate control
uint32 MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048 # 0x800 attitude stabilization
uint32 MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096 # 0x1000 yaw position
uint32 MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192 # 0x2000 z/altitude control
uint32 MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384 # 0x4000 x/y position control
uint32 MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768 # 0x8000 motor outputs / control
uint32 MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536 # 0x10000 rc receiver
uint32 MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072 # 0x20000 2nd 3D gyro
uint32 MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144 # 0x40000 2nd 3D accelerometer
uint32 MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288 # 0x80000 2nd 3D magnetometer
uint32 MAV_SYS_STATUS_GEOFENCE = 1048576 # 0x100000 geofence
uint32 MAV_SYS_STATUS_AHRS = 2097152     # 0x200000 AHRS subsystem health
uint32 MAV_SYS_STATUS_TERRAIN = 4194304  # 0x400000 Terrain subsystem health
uint32 MAV_SYS_STATUS_REVERSE_MOTOR = 8388608 # 0x800000 Motors are reversed
uint32 MAV_SYS_STATUS_LOGGING = 16777216 # 0x1000000 Logging
uint32 MAV_SYS_STATUS_SENSOR_BATTERY = 33554432 # 0x2000000 Battery
uint32 MAV_SYS_STATUS_SENSOR_PROXIMITY = 67108864 # 0x4000000 Proximity
uint32 MAV_SYS_STATUS_SENSOR_SATCOM = 134217728 # 0x8000000 Satellite Communication 
uint32 MAV_SYS_STATUS_PREARM_CHECK = 268435456 # 0x10000000 pre-arm check status. Always healthy when armed
uint32 MAV_SYS_STATUS_OBSTACLE_AVOIDANCE = 536870912 # 0x20000000 Avoidance/collision prevention
uint32 MAV_SYS_STATUS_SENSOR_PROPULSION = 1073741824 # 0x40000000 propulsion (actuator, esc, motor or propellor)
# [[[end]]] (checksum: cbcdd7bcf1aaf292236dded50be538c0)

std_msgs/Header header
uint32 present
uint32 enabled
uint32 health


