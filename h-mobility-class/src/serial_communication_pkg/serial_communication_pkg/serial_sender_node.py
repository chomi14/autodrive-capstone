import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/arduino'
#----------------------------------------------

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('port', PORT)
    self.declare_parameter('baudrate', 115200)
    self.declare_parameter('timeout', 1)
    self.declare_parameter('startup_delay', 1.0)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.ser = serial.Serial(
        self.get_parameter('port').value,
        self.get_parameter('baudrate').value,
        timeout=self.get_parameter('timeout').value
    )
    time.sleep(self.get_parameter('startup_delay').value)
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)

  def data_callback(self, msg):
    steering = msg.steering
    left_speed = msg.left_speed
    right_speed = msg.right_speed

    serial_msg =  PCFL.convert_serial_message(steering, left_speed, right_speed)
    self.ser.write(serial_msg.encode())

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      steering = 0
      left_speed = 0
      right_speed = 0
      message = PCFL.convert_serial_message(steering, left_speed, right_speed)
      node.ser.write(message.encode())
      pass
    
  finally:
    node.ser.close()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
