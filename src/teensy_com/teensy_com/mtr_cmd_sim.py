import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

import time
from random import randint


class MtrCmd(Node):
    def __init__(self):
        super().__init__('mtr_cmd_sim')
        self.publisher_ = self.create_publisher(String, 'mtr_cmd_topic', 10)
        self.subscription = self.create_subscription(String, 'teensy_topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(0.1, self.publish_mtr_cmd)

        self.start_time = time.time()

    def publish_mtr_cmd(self):
        # Publish random number every 10 seconds
        if time.time() - self.start_time > 10:
            self.start_time = time.time()
            msg = String()
            msg.data = f"MTR_CMD:({randint(-90,90)};{randint(-90,90)})"
            self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info(f"Recieved: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MtrCmd()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
