import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import serial.tools.list_ports
import threading

# See https://anavs.com/knowledgebase/nmea-format/
# Publish:
GNGGA = False
GNGSA = False
GPGSV = False
GLGSV = False
GNRMC = True
GNVTG = False

class GPS_Node(Node):
    def __init__(self, gps_port):
        super().__init__('GPS_node')

        self.publisher_ = self.create_publisher(String, 'gps_topic', 10)

        self.gps_port = gps_port

        # Open serial port
        try:
            self.gps_module = serial.Serial(self.gps_port)
            print("Connected to GPS module!")
        except:
            self.publish_msg("Cannot connect to GPS module!")
            self.destroy_node()
            exit()

        # Read GPS serial on parallel thread
        listener_thread = threading.Thread(target=self.serial_listener, daemon=True)
        listener_thread.start()

    def serial_listener(self):
        while True:
            if self.gps_module.in_waiting:
                read_line = self.gps_module.readline().decode("utf-8").rstrip()
                # See https://anavs.com/knowledgebase/nmea-format/
                if read_line.startswith("$GNGGA") and GNGGA:
                    self.publish_msg(read_line)
                elif read_line.startswith("$GNGSA") and GNGSA:
                    self.publish_msg(read_line)
                elif read_line.startswith("$GPGSV") and GPGSV:
                    self.publish_msg(read_line)
                elif read_line.startswith("$GLGSV") and GLGSV:
                    self.publish_msg(read_line)
                elif read_line.startswith("$GNRMC") and GNRMC:
                    self.publish_msg(read_line)
                elif read_line.startswith("$GNVTG") and GNVTG:
                    self.publish_msg(read_line)
                
    def publish_msg(self, msg):
        message = String()
        message.data = msg
        self.publisher_.publish(message)
        self.get_logger().info(f'Publishing: {message.data}')

def main(args=None):
    rclpy.init(args=args)

    #==========================Find GPS port======================= 
    gps_port = "/dev/"
    all_ports = serial.tools.list_ports.comports(include_links=True)
    for port in all_ports:
        if port.manufacturer == "PCB one SIA":
            gps_port += port.name
    #================================================================
    
    gps_node = GPS_Node(gps_port)
    rclpy.spin(gps_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()