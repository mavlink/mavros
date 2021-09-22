# Tunnel
#
# https://mavlink.io/en/messages/common.html#TUNNEL

uint8 target_system
uint8 target_component
uint16 payload_type
uint8 payload_length
uint8[128] payload

# [[[cog:
# from pymavlink.dialects.v20 import common
#
# def decl_enum(ename, pfx='', bsz=8):
#     enum = sorted(common.enums[ename].items())
#     enum.pop() # remove ENUM_END
#
#     cog.outl("# " + ename)
#     for k, e in enum:
#         sn = e.name[len(ename) + 1:]
#         l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
#         if e.description:
#             l += ' ' * (40 - len(l)) + ' # ' + e.description
#         cog.outl(l)
#
# decl_enum('MAV_TUNNEL_PAYLOAD_TYPE', 'PAYLOAD_TYPE_', 16)
# ]]]
# MAV_TUNNEL_PAYLOAD_TYPE
uint16 PAYLOAD_TYPE_UNKNOWN = 0          # Encoding of payload unknown.
uint16 PAYLOAD_TYPE_STORM32_RESERVED0 = 200 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED1 = 201 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED2 = 202 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED3 = 203 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED4 = 204 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED5 = 205 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED6 = 206 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED7 = 207 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED8 = 208 # Registered for STorM32 gimbal controller.
uint16 PAYLOAD_TYPE_STORM32_RESERVED9 = 209 # Registered for STorM32 gimbal controller.
# [[[end]]] (checksum: 3327b212af02c2d47d940cd6de049624)
