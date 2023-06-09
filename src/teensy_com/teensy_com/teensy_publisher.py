import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import serial.tools.list_ports
import threading

BAUDRATE_TEENSY = 115200

class TeensyNode(Node):
    def __init__(self, teensy_port, baudrate=9600):
        super().__init__('Teensy_node')

        self.publisher_ = self.create_publisher(String, 'teensy_topic', 10)
        self.subscription = self.create_subscription(String, 'mtr_cmd_topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.teensy_port = teensy_port
        self.baudrate = baudrate

        # Open serial port
        try:
            self.teensy = serial.Serial(self.teensy_port, baudrate=self.baudrate, timeout=0.1)
            print("Connected to teensy")
        except:
            self.publish_msg("Cannot connect to Teensy!")
            self.destroy_node()
            exit()

        # Read teensy serial on parallel thread
        listener_thread = threading.Thread(target=self.serial_listener, daemon=True)
        listener_thread.start()
    
    def serial_listener(self):
        while True:
            if self.teensy.in_waiting:
                read_line = self.teensy.readline().decode("utf-8").rstrip()
                if len(read_line) > 0:
                    self.publish_msg(read_line)
        
    def listener_callback(self, msg):
        self.get_logger().info(f"Recieved: {msg.data}")
        mtr_cmd_string = f"{msg.data}"
        self.teensy.write(bytes(mtr_cmd_string.encode("utf-8")))

    def publish_msg(self, msg):
        message = String()
        message.data = msg
        self.publisher_.publish(message)
        self.get_logger().info(f'Publishing: {message.data}')


def main(args=None):
    rclpy.init(args=args)

    #=================Find teensy port=============================== 
    teensy_port = "/dev/"
    all_ports = serial.tools.list_ports.comports(include_links=True)
    for port in all_ports:
        if port.manufacturer == "Teensyduino":
            teensy_port += port.name
        
    #================================================================
    
    teensy_node = TeensyNode(teensy_port, BAUDRATE_TEENSY)
    rclpy.spin(teensy_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()