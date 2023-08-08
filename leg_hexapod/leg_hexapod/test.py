#!/usr/bin/env python3
import sys
import os

# Dodaj ścieżkę do folderu z modułami
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import time 

import rclpy
from rclpy.node import Node

from .submod.inverse_kinematics_optimalisation import inverse_kinematics as ik
from .submod.bezier import bezier_curve as bc
from .submod.right_tourning_middle_leg import tourning as tourn

from hexapod_message.msg import ThetasLegRightMiddle

speed = 0
tourning_var = 1000000

class test(Node):
    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(ThetasLegRightMiddle, 'anglesMRLeg', 10)

    def publish_values(self, theta_1, theta_2, theta_3):
        msg = ThetasLegRightMiddle()
        msg.theta_1 = theta_1
        msg.theta_2 = theta_2
        msg.theta_3 = theta_3
        self.get_logger().info('Publishing: "%f" thetas' % msg.theta_1)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = test()
    Start = np.array([35.0, 120.0, -200.0])
    for o in range(100):
        for i in ik(Start[0], Start[1], Start[2], 0.0, 0.0, -90.0):
            convert = [float(konv) for konv in i]
            theta_1 = convert[0] 
            theta_2 = convert[1] 
            theta_3 = convert[2] 
            node.publish_values(theta_1, theta_2, theta_3)
            #print(" all thetas: " ,theta_1, theta_2, theta_3)
        iteration = 0

    for i in tourn(tourning_var) :

        if iteration == 0:
            for j in bc(Start[0], Start[1], Start[2], i[0], i[1], -160.0) :
                for z in ik(j[0], j[1], j[2],theta_1  - 90.0, theta_2 - 90.0, theta_3 - 180.0):
                    convert = [float(konv) for konv in z]
                    theta_1 = convert[0] 
                    theta_2 = convert[1] 
                    theta_3 = convert[2] 
                    node.publish_values(theta_1, theta_2, theta_3)
      

        else :
            for j in ik( i[0], i[1], -160.0, theta_1 - 90.0, theta_2 - 90.0, theta_3 - 180.0):
                convert = [float(konv) for konv in j]
                theta_1 = convert[0] 
                theta_2 = convert[1] 
                theta_3 = convert[2] 
                node.publish_values(theta_1, theta_2, theta_3)
     

        iteration += 1
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
