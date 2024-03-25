#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from train_interfaces.msg import TrainParams

class GazeboTrainConverter(Node):
    def __init__(self):
        super().__init__("gazebo_train_converter")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.sub = self.create_subscription(TrainParams, "/train_params", self.callback, 10)

    def callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.velocity
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = GazeboTrainConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()