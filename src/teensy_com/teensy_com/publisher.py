import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import serial.tools.list_ports

TIMEOUT = 0.1
BAUDRATE = 9600

class TeensyPublisher(Node):
    def __init__(self, teensy_port, timer_period=0.1, baudrate=9600):
        super().__init__('Teensy_publisher')

        self.publisher_ = self.create_publisher(String, 'teensy_topic', 10)

        self.teensy_port = teensy_port
        self.baudrate = baudrate
        
        # Open serial port
        try:
            self.teensy = serial.Serial(self.teensy_port, baudrate=self.baudrate)
        except:
            self.publish_msg("Cannot connect to Teensy!")
            self.destroy_node()
            exit()

        self.timer_period = timer_period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        

    def timer_callback(self):
        self.publish_msg(self.teensy.readline().decode("utf-8").rstrip())

    def publish_msg(self, msg):
        message = String()
        message.data = msg
        self.publisher_.publish(message)
        self.get_logger().info(f'Publishing: {message.data}')

def main(args=None):
    rclpy.init(args=args)

    #========================Find teensy port======================= 
    teensy_port = "/dev/"
    all_ports = serial.tools.list_ports.comports(include_links=True)
    for port in all_ports:
        if port.manufacturer == "Teensyduino":
            teensy_port += port.name
    #================================================================
    
    publisher = TeensyPublisher(teensy_port, TIMEOUT, BAUDRATE)

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()