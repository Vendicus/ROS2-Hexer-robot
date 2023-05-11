import rclpy
import numpy as np
import time

import inverse_kinematics_optimalisation as ik
import bezier
import tourning as tourn

from rclpy.node import Node
from vision_rpi_hexabot.msg import MyMessage


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MyMessage, 'topic', 10)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MyMessage()
        
        speed = 0
        tourning_var = 1000000
        Start = np.array([0.0, 145.0, -160.0])

        for i in ik.inverse_kinematics(Start[0], Start[1], Start[2], 0.0, 0.0, -80.0) :
            
            convert = [float(konv) for konv in i]
            msg.theta_1 = convert[0]
            msg.theta_2 = convert[1] 
            msg.theta_3 = convert[2]
            self.publisher_.publish(msg)
            time.sleep(speed)

        iteration = 0

        for i in tourn.tourning(tourning_var) :

            if iteration == 0:
                for j in bezier.bezier_curve(Start[0], Start[1], Start[2], i[0], i[1], -160.0) :
                    
                    for z in ik.inverse_kinematics(j[0], j[1], j[2], i[0]- 90.0, i[1]- 90.0, i[2] - 180.0):
                        convert = [float(konv) for konv in z]
                        msg.theta_1 = convert[0] 
                        msg.theta_2 = convert[1] 
                        msg.theta_3 = convert[2]
                        self.publisher_.publish(msg)
                        time.sleep(speed)

            else :
                for j in ik.inverse_kinematics( i[0], i[1], -160.0, i[0]- 90.0, i[1] - 90.0, i[2]- 180.0):
                    convert = [float(konv) for konv in j]              
                    msg.theta_1 = convert[0] 
                    msg.theta_2 = convert[1] 
                    msg.theta_3 = convert[2]
                    self.publisher_.publish(msg)
                    time.sleep(speed)
                
        
            iteration += 1



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()