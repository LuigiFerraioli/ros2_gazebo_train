# GUI interface zur benutzerfereundlicheren Bedienung 
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tkinter import *

class Teleop(Node):
    def __init__(self): 
        super().__init__("steuerung") 
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.max_speed = 2.0 	# m/s
        self.direction = 1 	# positiv/negativ

    def start_movement(self):
        cmd = Twist()
        cmd.linear.x = self.max_speed * 0.1 * self.direction
        self.pub.publish(cmd)

    def stop_movement(self):
        cmd = Twist()
        self.pub.publish(cmd)

    def toggle_direction(self):
        self.direction *= -1

    def set_speed(self, scale):
        cmd = Twist()
        cmd.linear.x = self.max_speed * scale * self.direction
        self.pub.publish(cmd)

class GUI:
    def __init__(self, node):
        self.node = node
        self.window = Tk(className=" Steuerung Gazebo Train")

        self.speed_buttons = []
        speeds = [0.25, 0.5, 0.75, 1.0]
        for speed in speeds:
            button = Button(self.window, text=f"Speed {int(speed * 100)}%", command=lambda s=speed: self.set_speeds(s))
            button.config(font=('Ink Free', 15, 'bold'))
            button.pack()
            self.speed_buttons.append(button)

        self.start_button = Button(self.window, text="Start", command=self.node.start_movement)
        self.start_button.config(font=('Ink Free', 15, 'bold'))
        self.start_button.pack()

        self.stop_button = Button(self.window, text="Stop", command=self.node.stop_movement)
        self.stop_button.config(font=('Ink Free', 15, 'bold'))
        self.stop_button.pack()

        self.direction_button = Button(self.window, text="Direction", command=self.node.toggle_direction)
        self.direction_button.config(font=('Ink Free', 15, 'bold'))
        self.direction_button.pack()

        self.window.geometry('200x270+0+0')
        self.window.attributes('-topmost', True)
        self.window.mainloop()

    def set_speeds(self, scale):
        self.node.set_speed(scale)

def main():
    rclpy.init()
    node = Teleop()
    gui = GUI(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



