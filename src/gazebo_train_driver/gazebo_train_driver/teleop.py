	

#!/usr/bin/env python3

###################  import  ###########################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

############# create a node class  #####################
class Teleop(Node):
    def __init__(self): # constructor
        super().__init__("teleop") # parent class constructor
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Joy, "/joy", self.callback, 10)
        self.dms = False # flag for Dead Man's Switch

############## create a node class  ####################
    def callback(self,msg):
        cmd = Twist() # outgoing command
############# check if e-stop is requested or DMS is released ############
        if msg.buttons[0] == 1:
            self.estop()
            return

        if msg.buttons[1] == 1:
            if self.dms:
                self.dms = False
                self.estop()
            return

        if msg.buttons[1] == 0:
            self.dms = True

############# apply joystick commands ##################
        speed_scale = 1.0
        turn_scale = 1.0

        cmd.linear.x = speed_scale * msg.axes[1]
        cmd.angular.z = turn_scale * msg.axes[3]

        self.pub.publish(cmd)

################ publish all-zero speed  ###############
    def estop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

###################  main method  ######################
def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

