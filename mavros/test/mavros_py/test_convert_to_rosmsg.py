"""
MAvlink message header: std_msgs/Header

# check Header type
- stamp field type: builtin_interfaces/Time
ros2 interface show std_msgs/msg/Header

builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
string frame_id

system_now() method return rclpy.Time type and not built_interface/Time type

Current version Excetion
------------------------
ros_msg = convert_to_rosmsg(msg) # , stamp=stamp
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/mavros/mavlink.py", line 110, in convert_to_rosmsg
    header = Header(stamp=stamp)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/std_msgs/msg/_header.py", line 81, in __init__
    self.stamp = kwargs.get('stamp', Time())
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/std_msgs/msg/_header.py", line 133, in stamp
    assert \
AssertionError: The 'stamp' field must be a sub message of type 'Time'
[ros2run]: Process exited with failure 1

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock
from builtin_interfaces.msg import Time as HTime
from mavros_msgs.msg import Mavlink
from builtin_interfaces.msg import Time
from pymavlink.dialects.v10 import ardupilotmega
from mavros.mavlink import convert_to_rosmsg
from pymavlink.dialects.v20 import ardupilotmega as apm

DRONE_NO = 1
TOPIC_MAVLINK = f"/uas{DRONE_NO}/mavlink_sink"

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

class MavWriterNode(Node):
    def __init__(self) -> None:
        node_name = "mav_writer"
        super().__init__(node_name)
        f = fifo()
        self.__mav = apm.MAVLink(f, srcSystem=1, srcComponent=1)
        self.__pub_mavlink = self.create_publisher(
            Mavlink,
            TOPIC_MAVLINK,
            qos_profile=qos_profile_sensor_data,
        )
        self.get_logger().info("init mavlink write demo")
        self.set_home()

    def set_home(self) -> None:
        target_system = 0  # broadcast to everyone
        msg = ardupilotmega.MAVLink_command_long_message(
            target_system,
            1,
            ardupilotmega.MAV_CMD_DO_SET_HOME,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )
        
        msg.pack(self.__mav)
        ros_msg = convert_to_rosmsg(msg)
        self.__pub_mavlink.publish(ros_msg)
        print(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MavWriterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()