#!/usr/bin/env python3
import tkinter as tk
import rclpy
from rclpy.node import Node
from threading import Thread
from controller_error_msgs.msg import DesiredConfiguration, Gains
import math
import numpy as np
import quaternion
from visualization_msgs.msg import Marker


class PoseGen(Node):

    def __init__(self):

        super().__init__("PoseGenerator")
        self.root = tk.Tk()

        self.root.wm_geometry("300x730")
        self.root.wm_title("Pose Generator")
        self.root.resizable(tk.FALSE, tk.FALSE)

        self.pub = self.create_publisher(DesiredConfiguration, 'manipulator_controller/end_effector_configuration', 10)
        self.marker_publisher = self.create_publisher(Marker, '/desired_pose', 10)

        self.gain_pub = self.create_publisher(Gains, 'manipulator_controller/gains', 10)

        self.x = tk.StringVar()
        self.x.set("0.088")

        self.y = tk.StringVar()
        self.y.set("0.0")

        self.z = tk.StringVar()
        self.z.set("0.9")

        self.yaw = tk.StringVar()
        self.yaw.set("0.0")

        self.pitch = tk.StringVar()
        self.pitch.set("0.0")

        self.roll = tk.StringVar()
        self.roll.set("180")

        tk.Label(self.root, text="X:").grid(row=0, column=0)
        tk.Spinbox(self.root, from_ = -100.0, to = 100.0, increment=0.01, textvariable=self.x).grid(row=0, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Y:").grid(row=1, column=0)
        tk.Spinbox(self.root, from_ = -100.0, to = 100.0, increment=0.01, textvariable=self.y).grid(row=1, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Z:").grid(row=2, column=0)
        tk.Spinbox(self.root, from_ = -100.0, to = 100.0, increment=0.01, textvariable=self.z).grid(row=2, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Yaw:").grid(row=3, column=0)
        tk.Spinbox(self.root, from_ = -360.0, to = 360.0, increment=1.0, textvariable=self.yaw).grid(row=3, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Pitch:").grid(row=4, column=0)
        tk.Spinbox(self.root, from_ = -360.0, to = 360.0, increment=1.0, textvariable=self.pitch).grid(row=4, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Roll:").grid(row=5, column=0)
        tk.Spinbox(self.root, from_ = -360.0, to = 360.0, increment=1.0, textvariable=self.roll).grid(row=5, column=1, sticky='ew', pady = 10, columnspan=4)


        self.plane = tk.IntVar()

        tk.Label(self.root, text="Circle Plane:").grid(row=6, column=0, padx = 10)
        tk.Radiobutton(self.root, text="XZ", variable=self.plane, value=0).grid(row=6, column=1, sticky='ew', pady = 5)
        tk.Radiobutton(self.root, text="YZ", variable=self.plane, value=1).grid(row=6, column=2, sticky='ew', pady = 5)
        tk.Radiobutton(self.root, text="XY", variable=self.plane, value=2).grid(row=6, column=3, sticky='ew', pady = 5)

        self.radius = tk.StringVar()
        self.radius.set("0.2")


        

        self.spiral = tk.IntVar()

        tk.Checkbutton(self.root, text="Spiral", variable=self.spiral).grid(row=7, column=0, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Circle Radius:").grid(row=8, column=0)
        tk.Spinbox(self.root, from_ = 0.0, to = 1.0, increment=0.01, textvariable=self.radius).grid(row=8, column=1, sticky='ew', pady = 10, columnspan=4)

        self.time_scale = tk.StringVar()
        self.time_scale.set("1.0")

        tk.Label(self.root, text="Time Scale:").grid(row=9, column=0)
        tk.Spinbox(self.root, from_ = 0.0, to = 5.0, increment=0.1, textvariable=self.time_scale).grid(row=9, column=1, sticky='ew', pady = 10, columnspan=4)
        
     

        self.button = tk.Button(self.root, text="Send", command=self.send)
        self.button.grid(row=10, column=0, sticky="ew", columnspan=5, padx=10)

        self.button = tk.Button(self.root, text="Toggle Circle", command=self.toggle_circle)
        self.button.grid(row=11, column=0, sticky="ew", columnspan=5, padx=10)


        self.kp_p = tk.StringVar()
        self.kp_o = tk.StringVar()
        self.kv_p = tk.StringVar()
        self.kv_o = tk.StringVar()

        self.kp_p.set("0.0")
        self.kp_o.set("0.0")
        self.kv_p.set("0.0")
        self.kv_o.set("0.0")


        tk.Label(self.root, text="Kp_p:").grid(row=12, column=0)
        tk.Spinbox(self.root, from_ = 0.0, to = 1000.0, increment=0.1, textvariable=self.kp_p).grid(row=12, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Kp_o:").grid(row=13, column=0)
        tk.Spinbox(self.root, from_ = 0.0, to = 1000.0, increment=0.1, textvariable=self.kp_o).grid(row=13, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Kv_p:").grid(row=14, column=0)
        tk.Spinbox(self.root, from_ = 0.0, to = 1000.0, increment=0.01, textvariable=self.kv_p).grid(row=14, column=1, sticky='ew', pady = 10, columnspan=4)

        tk.Label(self.root, text="Kv_o:").grid(row=15, column=0)
        tk.Spinbox(self.root, from_ = 0.0, to = 1000.0, increment=0.01, textvariable=self.kv_o).grid(row=15, column=1, sticky='ew', pady = 10, columnspan=4)

        self.send_gains_butt = tk.Button(self.root, text="Send Gains", command=self.send_gains)
        self.send_gains_butt.grid(row=16, column=0, sticky="ew", columnspan=5, padx=10)


        self.msg = DesiredConfiguration()
        self.gains = Gains()
        self.marker = Marker()
        self.rate = self.create_rate(30)

        self.send_circle = False
        self.t = 0


    def toggle_circle(self):

        self.send_circle = not self.send_circle



    def run(self):

        while rclpy.ok():

            if self.send_circle:

                self.compute_circle()



            self.root.update()
            self.root.update_idletasks()
            self.t += 0.0001


    def send_gains(self):

        self.gains.kp_p.data = float(self.kp_p.get())
        self.gains.kp_o.data = float(self.kp_o.get())
        self.gains.kv_p.data = float(self.kv_p.get())
        self.gains.kv_o.data = float(self.kv_o.get())


        self.gain_pub.publish(self.gains)


    def compute_circle(self):

        self.marker = Marker()        
        qx = quaternion.from_rotation_vector(np.array([1, 0, 0]) * math.radians(float(self.roll.get())))
        qy = quaternion.from_rotation_vector(np.array([0, 1, 0]) * math.radians(float(self.pitch.get())))
        qz = quaternion.from_rotation_vector(np.array([0, 0, 1]) * math.radians(float(self.yaw.get())))
    
        # Combine the rotations
        q = qz * qy * qx
  

        self.msg.pose.orientation.x = q.x
        self.msg.pose.orientation.y = q.y
        self.msg.pose.orientation.z = q.z
        self.msg.pose.orientation.w = q.w


        if self.plane.get() == 0:

            x = float(self.x.get()) + float(self.radius.get()) * math.cos(float(self.time_scale.get()) * self.t)
            z = float(self.z.get()) + float(self.radius.get()) * math.sin(float(self.time_scale.get()) * self.t)
            y = float(self.y.get())

            vx = - float(self.time_scale.get()) * float(self.radius.get()) * math.sin(float(self.time_scale.get()) * self.t)
            vz = float(self.time_scale.get()) * float(self.radius.get()) * math.cos(float(self.time_scale.get()) * self.t)
            vy = 0.0

            if self.spiral.get():

                y = float(self.y.get()) + float(self.radius.get()) * math.sin((float(self.time_scale.get()) * self.t)/np.pi)
                vy = float(self.time_scale.get()) * float(self.radius.get()) * math.cos((float(self.time_scale.get()) * self.t)/np.pi) / np.pi

        elif self.plane.get() == 1:

            y = float(self.y.get()) + float(self.radius.get()) * math.cos(float(self.time_scale.get()) * self.t)
            z = float(self.z.get()) + float(self.radius.get()) * math.sin(float(self.time_scale.get()) * self.t)
            x = float(self.x.get())

            vy = - float(self.time_scale.get()) * float(self.radius.get()) * math.sin(float(self.time_scale.get()) * self.t)
            vz =   float(self.time_scale.get()) * float(self.radius.get()) * math.cos(float(self.time_scale.get()) * self.t)
            vx = 0.0

            if self.spiral.get():

                x = float(self.x.get()) + float(self.radius.get()) * math.sin((float(self.time_scale.get()) * self.t)/np.pi)
                vx = float(self.time_scale.get()) * float(self.radius.get()) * math.cos((float(self.time_scale.get()) * self.t)/np.pi) / np.pi

        elif self.plane.get() == 2:

            x = float(self.x.get()) + float(self.radius.get()) * math.cos(float(self.time_scale.get()) * self.t)
            y = float(self.y.get()) + float(self.radius.get()) * math.sin(float(self.time_scale.get()) * self.t)
            z = float(self.z.get())            

            vx = - float(self.time_scale.get()) *float(self.radius.get()) * math.sin(float(self.time_scale.get()) * self.t)
            vy = float(self.time_scale.get()) * float(self.radius.get()) * math.cos(float(self.time_scale.get()) * self.t)
            vz = 0.0

            if self.spiral.get():

                z = float(self.z.get()) + float(self.radius.get()) * math.sin((float(self.time_scale.get()) * self.t)/np.pi)
                vz = float(self.time_scale.get()) * float(self.radius.get()) * math.cos((float(self.time_scale.get()) * self.t)/np.pi) / np.pi





        self.msg.pose.position.x = x
        self.msg.pose.position.y = y
        self.msg.pose.position.z = z

        self.msg.twist.linear.x = vx
        self.msg.twist.linear.y = vy
        self.msg.twist.linear.z = vz


        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.header.frame_id = "world"
        self.marker.ns = "desired_position"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.pose.position = self.msg.pose.position
        self.marker.pose.orientation = self.msg.pose.orientation
        self.marker.color.r = 0.7
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.scale.x = .5
        self.marker.scale.y = .05
        self.marker.scale.z = .05

        self.pub.publish(self.msg)
        self.marker_publisher.publish(self.marker)


        


    def send(self):

        self.marker = Marker()        
        qx = quaternion.from_rotation_vector(np.array([1, 0, 0]) * math.radians(float(self.roll.get())))
        qy = quaternion.from_rotation_vector(np.array([0, 1, 0]) * math.radians(float(self.pitch.get())))
        qz = quaternion.from_rotation_vector(np.array([0, 0, 1]) * math.radians(float(self.yaw.get())))
    
        # Combine the rotations
        q = qz * qy * qx

        self.msg.pose.orientation.x = q.x
        self.msg.pose.orientation.y = q.y
        self.msg.pose.orientation.z = q.z
        self.msg.pose.orientation.w = q.w

        self.msg.pose.position.x = float(self.x.get())
        self.msg.pose.position.y = float(self.y.get())
        self.msg.pose.position.z = float(self.z.get())

        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.header.frame_id = "world"
        self.marker.ns = "desired_position"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.pose.position = self.msg.pose.position
        self.marker.pose.orientation = self.msg.pose.orientation
        self.marker.color.r = 0.7
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.scale.x = .5
        self.marker.scale.y = .05
        self.marker.scale.z = .05

        self.pub.publish(self.msg)
        self.marker_publisher.publish(self.marker)




def main(args = None):

    rclpy.init(args=args)

    node = PoseGen()

    node.run()




if __name__ == "__main__":
    main()