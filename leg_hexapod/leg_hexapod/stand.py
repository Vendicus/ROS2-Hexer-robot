#!/usr/bin/env python3
import sys
import os

# add dependcies to actual path folder folder
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time 

import rclpy
from rclpy.node import Node

from hexapod_message.msg import ThetasLegLeftDown, ThetasLegLeftMiddle, ThetasLegLeftup, ThetasLegRightDown, ThetasLegRightMiddle, ThetasLegRightUp

class stand(Node):
    def __init__(self):
        super().__init__('stand')
        self.publisher_RM = self.create_publisher(ThetasLegRightMiddle, 'anglesMRLeg', 1)
        self.publisher_LM = self.create_publisher(ThetasLegLeftMiddle, 'anglesMLLeg', 1)
        self.publisher_RU = self.create_publisher(ThetasLegRightUp, 'anglesURLeg', 1)
        self.publisher_LU = self.create_publisher(ThetasLegLeftup, 'anglesULLeg', 1)
        self.publisher_RD = self.create_publisher(ThetasLegRightDown, 'anglesDRLeg', 1)
        self.publisher_LD = self.create_publisher(ThetasLegLeftDown, 'anglesDLLeg', 1)

    def publish_values(self):
        msg_RM = ThetasLegRightMiddle()
        msg_RM.theta_1 = 90.0
        msg_RM.theta_2 = 90.0
        msg_RM.theta_3 = 90.0
        self.publisher_RM.publish(msg_RM)

        msg_LM = ThetasLegLeftMiddle()
        msg_LM.theta_1 = 90.0
        msg_LM.theta_2 = 90.0
        msg_LM.theta_3 = 90.0
        self.publisher_LM.publish(msg_LM)

        msg_RU = ThetasLegRightUp()
        msg_RU.theta_1 = 145.0
        msg_RU.theta_2 = 90.0
        msg_RU.theta_3 = 90.0
        self.publisher_RU.publish(msg_RU)

        time.sleep(1)
        
        msg_LU = ThetasLegLeftup()
        msg_LU.theta_1 = 145.0
        msg_LU.theta_2 = 90.0
        msg_LU.theta_3 = 90.0
        self.publisher_LU.publish(msg_LU)

        msg_RD = ThetasLegRightDown()
        msg_RD.theta_1 = 145.0
        msg_RD.theta_2 = 90.0
        msg_RD.theta_3 = 90.0
        self.publisher_RD.publish(msg_RD)   

        msg_LD = ThetasLegLeftDown()
        msg_LD.theta_1 = 45.0
        msg_LD.theta_2 = 90.0
        msg_LD.theta_3 = 90.0
        self.publisher_LD.publish(msg_LD)

        # Values return
        return {
            'RM': msg_RM,
            'LM': msg_LM,
            'RU': msg_RU,
            'LU': msg_LU,
            'RD': msg_RD,
            'LD': msg_LD
        }     

def main(args=None):
    rclpy.init(args=args)
    node = stand()
    node.publish_values()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()