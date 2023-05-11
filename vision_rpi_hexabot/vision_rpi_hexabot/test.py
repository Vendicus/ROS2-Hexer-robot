import numpy as np
import time

import rclpy
from rclpy.node import Node

import inverse_kinematics_optimalisation as ik
import bezier
import tourning as t

speed = 0
tourning_var = 1000000
Start = np.array([0.0, 145.0, -160.0])

class test(Node):
    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(MyMessage, 'topic', 10)

    def publish_values(self, theta_1, theta_2, theta_3):
        msg = MyMessage()
        msg.theta_1 = theta_1
        msg.theta_2 = theta_2
        msg.theta_3 = theta_3
        self.get_logger().info('Publishing: "%f" thetas' % msg.theta_1)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = test()

    for i in ik.inverse_kinematics(Start[0], Start[1], Start[2], 0.0, 0.0, -90.0):
        convert = [float(konv) for konv in i]
        theta_1 = convert[0] 
        theta_2 = convert[1] 
        theta_3 = convert[2] 
        node.publish_values(theta_1, theta_2, theta_3)
        time.sleep(speed)

    iteration = 0

    for i in tourn.tourning(tourning_var) :
        print(" tourning values : " + str(i) + "\n\n\n")

        if iteration == 0:
            for j in bezier.bezier_curve(Start[0], Start[1], Start[2], i[0], i[1], -160.0) :
                for z in ik.inverse_kinematics(j[0], j[1], j[2],theta_1  - 90.0, theta_2 - 90.0, theta_3 - 180.0):
                    convert = [float(konv) for konv in z]
                    theta_1 = convert[0] 
                    theta_2 = convert[1] 
                    theta_3 = convert[2] 
                    node.publish_values(theta_1, theta_2, theta_3)
                    time.sleep(speed)

        else :
            for j in ik.inverse_kinematics( i[0], i[1], -160.0, theta_1 - 90.0, theta_2 - 90.0, theta_3 - 180.0):
                convert = [float(konv) for konv in j]
                theta_1 = convert[0] 
                theta_2 = convert[1] 
                theta_3 = convert[2] 
                node.publish_values(theta_1, theta_2, theta_3)
                time.sleep(speed)

        iteration += 1
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()