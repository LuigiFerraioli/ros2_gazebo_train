#!/usr/bin/env python3
# GUI interface for more user-friendly operation
# Author: Luigi Feraioli

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tkinter import *
from tkinter import ttk


class Teleop(Node):
    def __init__(self):
        super().__init__("Controller")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.max_speed = 2.0  # m/s (default)
        self.direction = 1  # positive/negative

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

    def update_max_speed(self, new_speed):
        self.max_speed = new_speed


class GUI:
    def __init__(self, node):
        self.node = node
        self.current_scale = 0.0
        self.window = Tk(className=" Controller Gazebo Train")
        self.window.geometry('300x500+0+0')
        self.window.attributes('-topmost', True)

        # Speed buttons
        speeds = [0.25, 0.5, 0.75, 1.0]
        Label(self.window, text="Set Speed (relative):",
              font=('Arial', 12, 'bold')).pack(pady=5)
        for speed in speeds:
            button = Button(
                self.window,
                text=f"{int(speed * 100)}%",
                command=lambda s=speed: self.set_speeds(s)
            )
            button.config(font=('Ink Free', 14, 'bold'))
            button.pack(pady=2)

        # Start
        self.start_button = Button(
            self.window, text="Start", command=self.start_pressed)
        self.start_button.config(font=('Ink Free', 14, 'bold'))
        self.start_button.pack(pady=5)

        # Stop
        self.stop_button = Button(
            self.window, text="Stop", command=self.stop_pressed)
        self.stop_button.config(font=('Ink Free', 14, 'bold'))
        self.stop_button.pack(pady=5)

        # Direction toggle
        self.direction_button = Button(
            self.window, text="Direction", command=self.toggle_direction)
        self.direction_button.config(font=('Ink Free', 14, 'bold'))
        self.direction_button.pack(pady=5)

        # Slider for max speed
        Label(self.window, text="Max Speed (m/s):",
              font=('Arial', 12, 'bold')).pack(pady=10)
        self.speed_slider = Scale(
            self.window,
            from_=2,
            to=6,
            resolution=2,
            orient=HORIZONTAL,
            command=self.slider_changed
        )
        self.speed_slider.set(self.node.max_speed)
        self.speed_slider.pack()

        # Current speed label
        self.current_speed_label = Label(
            self.window, text="Current speed: 0.0 m/s", font=('Arial', 12))
        self.current_speed_label.pack(pady=10)

        self.window.mainloop()

    def update_speed_display(self):
        current_speed = self.node.max_speed * self.current_scale * self.node.direction
        self.current_speed_label.config(
            text=f"Current speed: {current_speed:.2f} m/s")

    def set_speeds(self, scale):
        self.current_scale = scale
        self.node.set_speed(scale)
        self.update_speed_display()

    def slider_changed(self, value):
        self.node.update_max_speed(float(value))

    def toggle_direction(self):
        self.node.toggle_direction()
        self.update_speed_display()

    def start_pressed(self):
        self.node.start_movement()
        self.update_speed_display()

    def stop_pressed(self):
        self.node.stop_movement()
        self.current_scale = 0.0
        self.update_speed_display()


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
