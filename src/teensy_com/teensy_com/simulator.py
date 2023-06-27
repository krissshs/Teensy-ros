import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pynmeagps import NMEAReader
import time
from random import randint

SIMULATE_MOTOR_COMMAND = False

# Listen for:
GNGGA = False
GNGSA = False
GPGSV = False
GLGSV = False
GNRMC = True
GNVTG = False

class Simulator(Node):
    def __init__(self):
        super().__init__('Simulator')
        self.start_time = time.time()

        if SIMULATE_MOTOR_COMMAND:
            self.publisher_ = self.create_publisher(String, 'mtr_cmd_topic', 10)
            self.timer = self.create_timer(0.1, self.publish_mtr_cmd)

        self.subscription_1 = self.create_subscription(String, 'teensy_topic', self.listener_callback_1, 10)
        self.subscription_2 = self.create_subscription(String, 'gps_topic', self.listener_callback_2, 10)
        self.subscription_1  # prevent unused variable warning
        self.subscription_2  # prevent unused variable warning

    def publish_mtr_cmd(self):
        # Publish random number every 10 seconds
        if time.time() - self.start_time > 10:
            self.start_time = time.time()
            msg = String()
            msg.data = f"MTR_CMD:({randint(-90,90)};{randint(0,255)})({randint(-90,90)};{randint(0,255)})"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

    def listener_callback_1(self, msg):
        self.get_logger().info(f"Recieved: {msg.data}")

    def listener_callback_2(self, msg):
        if msg.data.startswith("$GNGGA") and GNGGA:
            msg_GNGGA = NMEAReader.parse(msg.data)
            self.get_logger().info(f"Recieved GNGGA:\n\tTime: {msg_GNGGA.time}\n\tlat: {msg_GNGGA.lat} {msg_GNGGA.NS}\n\tlon: {msg_GNGGA.lon} {msg_GNGGA.EW}\n\t")
            # Properties of parsed GNGGA:
            #   time - (UTC) hh:mm:ss
            #   lat - in decimal degrees
            #   lon - in decimal degrees
            #   NS - northen / southern hemisphere (of lattitude)
            #   EW - eastern / western hemisphere (of longitude)
            #   quality - GPS quality indicator (0: GNSS fix not available; 1: GNSS fix valid; 4: RTK fixed ambiguities; 5: RTK float ambiguities)
            #   numSV - number of satellites
            #   HDOP - Horizontal diluation of precision
            #   alt - altiude above sea level
            #   altUnit - altiude unit (typically meters)
            #   sep - geoidal seperation
            #   sepUnit - unit of geoidal seperation
        
        if msg.data.startswith("$GNGSA") and GNGSA:
            msg_GNGSA = NMEAReader.parse(msg.data)
            self.get_logger().info(f"Recieved GNGSA string: {msg_GNGSA}\n\t")
            # Properties of parsed GNGSA:
            #    opMode
            #    navMode
            #    svid_01
            #    svid_02
            #    svid_03
            #    svid_04
            #    svid_05
            #    svid_06
            #    svid_07
            #    svid_08
            #    svid_09
            #    svid_10
            #    svid_11
            #    svid_12
            #    PDOP
            #    HDOP
            #    VDOP

        if msg.data.startswith("$GPGSV") and GPGSV:
            msg_GPGSV = NMEAReader.parse(msg.data)
            self.get_logger().info(f"Recieved GPGSV string: {msg_GPGSV}\n\t")
            # Properties of parsed GPGSV:
            #    numMsg
            #    msgNum
            #    numSV
            #    svid_01
            #    elv_01
            #    az_01
            #    cno_01
            #    svid_02
            #    elv_02
            #    az_02
            #    cno_02
            #    svid_03
            #    elv_03
            #    az_03
            #    cno_03
            #    svid_04
            #    elv_04
            #    az_04
            #    cno_04
        
        if msg.data.startswith("$GLGSV") and GLGSV:
            msg_GLGSV = NMEAReader.parse(msg.data)
            self.get_logger().info(f"Recieved GLGSV string: {msg_GLGSV}\n\t")
            # Properties of parsed GLGSV:
            #    numMsg
            #    msgNum
            #    numSV
            #    svid_01
            #    elv_01
            #    az_01
            #    cno_01
            #    svid_02
            #    elv_02
            #    az_02
            #    cno_02
            #    svid_03
            #    elv_03
            #    az_03
            #    cno_03
            #    svid_04
            #    elv_04
            #    az_04
            #    cno_04

        if msg.data.startswith("$GNRMC") and GNRMC:
            msg_GNRMC = NMEAReader.parse(msg.data)
            self.get_logger().info(f"Recieved GNRMC:\n\tDate: {msg_GNRMC.date}\n\tTime: {msg_GNRMC.time}\n\tlat: {msg_GNRMC.lat} {msg_GNRMC.NS}\n\tlon: {msg_GNRMC.lon} {msg_GNRMC.EW}\n\t")
            # Properties of parsed GNRMC:
            #   time - (UTC) hh:mm:ss
            #   status - A: valid data
            #   lat - in decimal degrees
            #   lon - in decimal degrees
            #   NS - northen / southern hemisphere (of lattitude)
            #   EW - eastern / western hemisphere (of longitude)
            #   spd - speed over ground
            #   cog - course over ground
            #   date - yyyy-mm-dd
            #   mv - magnetic variation
            #   mvEW
            #   mode indicator - A: autonomous; D: differential

        if msg.data.startswith("$GNVTG") and GNVTG:
            msg_GNVTG = NMEAReader.parse(msg.data)
            self.get_logger().info(f"Recieved GNVTG:\n\tTrue COG: {msg_GNVTG.cogt}\n\tMagnetic COG: {msg_GNVTG.cogm}")
            # Properties of parsed GNVTG:
            #   cogt - course over ground (true)
            #   cogtUnit - course over ground (true) unit
            #   cogm - course over ground (magnetic)
            #   cogmUnit - course over ground (magnetic) unit
            #   sogn - speed over ground (knots)
            #   sognUnit - speed over ground (knots) unit
            #   sogk - speed over ground (kmh)
            #   sogkUnit - speed over ground (kmh) unit
            #   posMode - mode indicator; A - autonomous
        
        # https://anavs.com/knowledgebase/nmea-format/
        
def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
