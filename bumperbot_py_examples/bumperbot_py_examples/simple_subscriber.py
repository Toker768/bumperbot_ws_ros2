#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  


class SimpleSubscriber(Node): 
    def __init__(self):
        super().__init__("simple_subscriber")
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)


    def msgCallback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")
        # Here you can add any processing logic for the received message


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()