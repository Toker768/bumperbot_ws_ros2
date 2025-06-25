#!/usr/bin/env python3
import rclpy                    # we use definitley these
from rclpy.node import Node
from std_msgs.msg import String  # we use this to publish a string message



class SimplePublisher(Node): 
    def __init__(self):
        super().__init__('simple_publisher')
        self.pub_ = self.create_publisher(String, "chatter", 10)

        self.counter_ = 0
        self.frequency = 1.0  # Hz

        self.get_logger().info(f"Publishing at {self.frequency} Hz")
        self.timer_ = self.create_timer(self.frequency, self.timerCallback)


    def timerCallback(self):
        msg = String()
        msg.data = f"Hello, world! {self.counter_}"
        self.pub_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        self.counter_ += 1
            

        
        
    
def main():
    rclpy.init()
    simple_publisher= SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()