from .submod.bezier import bezier_curve as bc
from .submod.inverse_kinematics_optimalisation import inverse_kinematics as ik
from .submod.forward_kinematics import forward_kine as fk

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from hexapod_message.msg import  ThetasAll, Sensor, Semafor

class stand_loop(Node):
    def __init__(self, angle_start_x, angle_start_y, angle_start_z, Z):
        super().__init__('stand_loop_lu')
        second_cycle_lu = MutuallyExclusiveCallbackGroup()
        self.publisher_LU = self.create_publisher(ThetasAll, 'anglesULLegList', 10, callback_group=second_cycle_lu)
        self.publisher_LU_work = self.create_publisher(Semafor, 'Semafor', 3) 
        self.timer = self.create_timer(0.01, self.stand_in_loop_lu)

        self.angle_start_x = angle_start_x
        self.angle_start_y = angle_start_y
        self.angle_start_z = angle_start_z
        self.Z = Z

    def stand_in_loop_lu(self):
        list_lu = ThetasAll()
        msg_1 = Semafor()

        returner = fk(self.angle_start_x - 90, self.angle_start_y - 90, -self.angle_start_z )
        converter = [float(konv) for konv in returner]    

        Z = self.Z 

        b_end_x = -85.5
        b_end_y = 85.5

        theta_1 = self.angle_start_x
        theta_2 = self.angle_start_y
        theta_3 = self.angle_start_z

        for j in bc(converter[0], converter[1], converter[2], b_end_x, b_end_y, Z):          
            for z in ik(j[0], j[1], j[2], theta_1 - 90, theta_2 - 90, -theta_3):
                theta_1 = z[0] 
                theta_2 = z[1] 
                theta_3 = z[2]
                
                for i in range(3):
                    list_lu.lista.append(z[i])

                #print("kąty do lu : ", theta_1, theta_2, theta_3)

            end_x = j[0]
            end_y = j[1]

        Z = -215.0 

        for z in ik(end_x, end_y, Z, theta_1 -90.0, theta_2 -90.0, -theta_3 ):
            for i in range(3):
                list_lu.lista.append(z[i])
            #print("kąty do lu : ", z[0], z[1], z[2])
        #print(list_lu.lista)
        msg_1.semafor_lu = int(1)
        self.publisher_LU_work.publish(msg_1)
        self.timer.cancel()
        self.ultra(list_lu)
    
    def ultra(self, msg):
        self.timer_01 = self.create_timer(0.1, self.ultra)
        self.timer_cancel = self.create_timer(2, self.canceler)
        self.publisher_LU.publish(msg)
        self.timer_01.cancel()
    
    def canceler(self):
        self.timer_01.cancel()
        self.timer_cancel.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = stand_loop()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()