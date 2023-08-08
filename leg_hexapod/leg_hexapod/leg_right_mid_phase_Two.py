#!/usr/bin/env python3
import sys
import os

# add dependcies to actual path folder folder
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

import rclpy
from rclpy.node import Node

from .submod.inverse_kinematics_optimalisation import inverse_kinematics as ik
from .submod.right_tourning_middle_leg import tourning as tourn

from hexapod_message.msg import ThetasAll, Semafor

class mid_right(Node):
    def __init__(self, angle_start_x, angle_start_y, angle_start_z, tourning_var, speed, Z):
        super().__init__('right_middle_publisher')
        self.publisher_RM_walk = self.create_publisher(ThetasAll, 'anglesMRLegList', 10)
        self.timer = self.create_timer(0.01, self.publish_values)
        self.publisher_RM_work = self.create_publisher(Semafor, 'Semafor', 3)

        self.angle_start_x = angle_start_x
        self.angle_start_y = angle_start_y
        self.angle_start_z = angle_start_z
        self.tourning_var = tourning_var
        self.speed = speed
        self.Z = Z

    def publish_values(self):
        list_rm = ThetasAll()
        msg_1 = Semafor()

        theta_1 = self.angle_start_x 
        theta_2 = self.angle_start_y 
        theta_3 = self.angle_start_z

        Z = self.Z

        #print("start angle x, start angle y, z : ", self.angle_start_x, self.angle_start_y, self.angle_start_z)
        for t in tourn(self.tourning_var, self.speed):
            print("tourn RM : ", t)
            for i in ik(t[0], t[1], Z, theta_1 - 90.0, theta_2 - 90.0, -theta_3):
                theta_1 = i[0] 
                theta_2 = i[1] 
                theta_3 = i[2] 
                
                for angl in range(3):
                    list_rm.lista.append(i[angl])

        msg_1.semafor_rm= int(1)
        self.publisher_RM_work.publish(msg_1)
        self.timer.cancel()  
        self.ultra(list_rm)
    
    def ultra(self, msg):
        self.timer_01 = self.create_timer(0.5, self.ultra)
        self.publisher_RM_walk.publish(msg)
        self.timer_01.cancel()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = mid_right()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()