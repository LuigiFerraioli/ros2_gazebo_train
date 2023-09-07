#GUI interface zur benutzerfereundlicheren Bedienung 
#!/usr/bin/env python3

###################  import  ###########################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tkinter import*
from unittest import case
from click import command
############# create a node class  #####################


class Teleop(Node):
    def __init__(self): 
        super().__init__("teleop") 
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        cmd = Twist() 
        global counter
        counter = 0

        def fspeed25():
            print("Geschwindigkeit = 25%")
            speed_scale = 0.3
            cmd.linear.x = speed_scale * direktion
            self.pub.publish(cmd)

        def fspeed50():
            print("Geschwindigkeit = 50%")
            speed_scale = 0.6
            cmd.linear.x = speed_scale * direktion
            self.pub.publish(cmd)

        def fspeed75():
            print("Geschwindigkeit = 75%")
            speed_scale = 0.9
            cmd.linear.x = speed_scale * direktion
            self.pub.publish(cmd)

        def fspeed100():
            print("Geschwindigkeit = 100%")
            speed_scale = 1.2
            cmd.linear.x = speed_scale * direktion
            self.pub.publish(cmd)

        def fstart():
            print("Start wird eingeleitet")
            speed_scale = 0.1
            fgetdirektion()
            cmd.linear.x = speed_scale * direktion
            self.pub.publish(cmd)

        def fstop():
            print("STOP -> Keine Beschleunigung")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)

        def fdirektion():
            print("Richtungsänderung")
            global counter
            global direktion
            counter += 1

            if counter==1:
                direktion = -1
            if counter==2:
                direktion = 1
                counter = 0

        def fgetdirektion():
            global direktion     
            direktion = 1

        window = Tk(className=" Steuerung Gazebo Train")
        direktionb = Button(window,text="Direction")
        start = Button(window,text="Start")
        stop = Button(window,text="Stop")
        speed25= Button(window,text=" Speed 25%")
        speed50 = Button(window,text=" Speed 50%")
        speed75 = Button(window,text=" Speed 75%")
        speed100 = Button(window,text=" Speed 100%")
        speed25.config(command=fspeed25)
        speed50.config(command=fspeed50)
        speed75.config(command=fspeed75)
        speed100.config(command=fspeed100)
        start.config(command=fstart)
        stop.config(command=fstop)
        direktionb.config(command=fdirektion)
        speed25.config(font=('Ink Free',15,'bold'))
        speed50.config(font=('Ink Free',15,'bold'))
        speed75.config(font=('Ink Free',15,'bold'))
        speed100.config(font=('Ink Free',15,'bold'))
        start.config(font=('Ink Free',15,'bold'))
        stop.config(font=('Ink Free',15,'bold'))
        direktionb.config(font=('Ink Free',15,'bold'))
        speed25.pack()
        speed50.pack()
        speed75.pack()
        speed100.pack()
        start.pack()
        stop.pack()
        direktionb.pack()
        window.geometry('200x270+0+0')
        window.attributes('-topmost', True)
        window.mainloop()

	

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

