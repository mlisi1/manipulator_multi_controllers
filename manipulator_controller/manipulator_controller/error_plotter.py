#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState
from controller_error_msgs.msg import OperationalSpaceError
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
from rclpy.time import Time


class Plotter(Node):

    def __init__(self):

        super().__init__("ErrorPlotter")

        self.sub = self.create_subscription(OperationalSpaceError, "/manipulator_controller/error", self.msg_callback, 10)

        self.x_data = np.empty(0)
        self.pos_err = np.empty((0,3))
        self.vel_err = np.empty((0,3))

        self.time = np.empty(0)

        self.fig = plt.figure()

        self.subfigures = self.fig.subplots(ncols=1, nrows=2)

        self.pos_ax = self.subfigures[0]
        self.vel_ax = self.subfigures[1]

        

        self.first_msg = True

        self.pos_line = []
        self.vel_line = []







         



    def msg_callback(self, msg):

        # self.pos_err = np.append(self.pos_err, msg.position)
        # self.vel_err = np.append(self.vel_err, msg.velocity)

        label_dict = {0 : "X", 1 : "Y", 2 : "Z"}

        tmp1 = np.array([msg.position_error.position.x, msg.position_error.position.y, msg.position_error.position.z]).reshape((3,))
        tmp2 = np.array([msg.velocity_error.position.x, msg.velocity_error.position.y, msg.velocity_error.position.z]).reshape((3,))

        self.pos_err = np.vstack((self.pos_err, tmp1))
        self.vel_err = np.vstack((self.vel_err, tmp2))


        self.x_data = np.array(list(range(0, self.pos_err.shape[0])))

        time = Time.from_msg(msg.stamp)

        self.time = np.append(self.time, time.nanoseconds * 10e-10)


        if self.first_msg:

            for i in range(3):

                self.pos_line.append(self.pos_ax.plot(self.time, self.pos_err[:,0], label = f"Position Error - {label_dict[i]}")[0])
                self.vel_line.append(self.vel_ax.plot(self.time, self.vel_err[:,0], label = f"Velocity Error - {label_dict[i]}")[0])



            self.pos_ax.grid()
            self.vel_ax.grid()
            self.pos_ax.legend()
            self.vel_ax.legend()
            self.first_msg = False



    def run(self):

        while rclpy.ok():

            if not len(self.vel_line) == 0 and not len(self.pos_line) == 0:
                
                for i in range(self.pos_err.shape[1]):

                    self.pos_line[i].set_data(self.time, self.pos_err[:,i])
                    self.vel_line[i].set_data(self.time, self.vel_err[:,i])


                pos_max = np.max(self.pos_err) + 0.1
                vel_max = np.max(self.vel_err) + 0.1

                pos_min = np.min(self.pos_err) - 0.1
                vel_min = np.min(self.vel_err) - 0.1

                y_lim_max = max(pos_max, vel_max)
                y_lim_min = min(pos_min, vel_min)


                self.pos_ax.set_xlim(self.time[0], self.time[-1] + 0.1)
                self.vel_ax.set_xlim(self.time[0], self.time[-1] + 0.1)


                self.pos_ax.set_ylim(ymax=pos_max, ymin=pos_min)
                self.vel_ax.set_ylim(ymax=vel_max, ymin=vel_min)

                self.fig.canvas.draw()


            rclpy.spin_once(self)

        
        self.destroy_node()
        rclpy.shutdown()
        




def main(args=None):

    rclpy.init(args = args)

    node = Plotter()

 

    spin_thread = Thread(target=node.run, daemon=True)
    spin_thread.start()


    plt.subplots_adjust(0.022, 0.043, 0.982, 0.956, 0.2, 0.165)
    plt.show() 

    



if __name__ == '__main__':

    main()