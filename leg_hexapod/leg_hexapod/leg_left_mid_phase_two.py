#!/usr/bin/env python3
import sys
import os

# add dependcies to actual path folder folder
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rclpy
from rclpy.node import Node

from .submod.inverse_kinematics_optimalisation import inverse_kinematics as ik
from .submod.bezier import bezier_curve as bc
from .submod.left_tourning_middle_leg import tourning as tourn
from .submod.forward_kinematics import forward_kine as fk

from hexapod_message.msg import ThetasAll, Semafor

class mid_left(Node):
    def __init__(self, angle_start_x, angle_start_y, angle_start_z, tourning_var, speed, Z):
        super().__init__('left_middle_publisher')
        self.publisher_LM_walk = self.create_publisher(ThetasAll, 'anglesMLLegList', 10)
        self.timer = self.create_timer(0.01, self.publish_values)
        self.publisher_LM_work = self.create_publisher(Semafor, 'Semafor', 3)

        self.angle_start_x = angle_start_x
        self.angle_start_y = angle_start_y
        self.angle_start_z = angle_start_z
        self.tourning_var = tourning_var
        self.speed = speed
        self.Z = Z

    def publish_values(self):
        list_lm = ThetasAll()
        msg_1 = Semafor()
        
        Z = self.Z

        theta_1 = self.angle_start_x
        theta_2 = self.angle_start_y
        theta_3 = self.angle_start_z

        returner = fk(self.angle_start_x - 90, self.angle_start_y - 90, -self.angle_start_z)
        converter = [float(konv) for konv in returner] 

        iteration = 0

        # !!! PHASE 2
        for i in tourn(self.tourning_var, self.speed) :
            for j in bc(converter[0], converter[1], converter[2], i[0], i[1], Z):            
                for z in ik(j[0], j[1], j[2], theta_1 -90.0, theta_2 -90.0, -theta_3 ):
                    theta_1 = z[0] 
                    theta_2 = z[1] 
                    theta_3 = z[2]

                    for angl in range(4): 
                        list_lm.lista.append(z[angl])

                end_x = j[0]
                end_y = j[1]
                iteration += 1
            break

        Z = -215.0  

        for z in ik(end_x, end_y, Z, theta_1 -90.0, theta_2 -90.0, -theta_3 ):
            theta_1 = z[0] 
            theta_2 = z[1] 
            theta_3 = z[2]

            for angl in range(4): 
                list_lm.lista.append(z[angl])

        msg_1.semafor_lm = int(1)
        self.publisher_LM_work.publish(msg_1)   
        self.timer.cancel()    
        self.ultra(list_lm)
    
    def ultra(self, msg):
        self.timer_01 = self.create_timer(0.5, self.ultra)
        self.publisher_LM_walk.publish(msg)
        self.timer_01.cancel()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = mid_left()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()