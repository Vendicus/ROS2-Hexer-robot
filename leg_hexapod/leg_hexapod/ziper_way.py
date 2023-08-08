from .submod.bezier import bezier_curve as bc
from .submod.inverse_kinematics_optimalisation import inverse_kinematics as ik
from .submod.forward_kinematics import forward_kine as fk

from itertools import zip_longest
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from hexapod_message.msg import  ThetasLegLeftMiddle, ThetasLegRightDown, ThetasLegRightUp, Sensor, Semafor

import time 

class stand_loop(Node):
    def __init__(self, angle_start_x_lm, angle_start_y_lm, angle_start_z_lm, angle_start_x_ru, angle_start_y_ru, angle_start_z_ru, angle_start_x_rd, angle_start_y_rd, angle_start_z_rd, Z):
        super().__init__('stand_loop_lm')
        first_cycle_lm = ReentrantCallbackGroup()
        first_cycle_ru = ReentrantCallbackGroup()
        first_cycle_rd = ReentrantCallbackGroup()

        self.publisher_LM = self.create_publisher(ThetasLegLeftMiddle, 'anglesMLLeg', 2, callback_group=first_cycle_lm)
        self.publisher_RU = self.create_publisher(ThetasLegRightUp, 'anglesURLeg', 2, callback_group=first_cycle_ru)
        self.publisher_RD = self.create_publisher(ThetasLegRightDown, 'anglesDRLeg',  2, callback_group=first_cycle_rd)
        self.publisher_work = self.create_publisher(Semafor, 'Semafor', 3)
        self.subscriber_Sensor = self.create_subscription(Sensor, 'Sensor', self.stand_in_loop_first_cycle, 2)

        self.angle_start_x_lm = angle_start_x_lm
        self.angle_start_y_lm = angle_start_y_lm
        self.angle_start_z_lm = angle_start_z_lm

        self.angle_start_x_ru = angle_start_x_ru
        self.angle_start_y_ru = angle_start_y_ru
        self.angle_start_z_ru = angle_start_z_ru

        self.angle_start_x_rd = angle_start_x_rd
        self.angle_start_y_rd = angle_start_y_rd
        self.angle_start_z_rd = angle_start_z_rd

        self.Z = Z


    def stand_in_loop_first_cycle(self, Sensor_info: Sensor):
        msg_lm = ThetasLegLeftMiddle()
        msg_ru = ThetasLegRightUp()
        msg_rd = ThetasLegRightDown()
        msg_1 = Semafor()

        returner_lm = fk(self.angle_start_x_lm - 90, self.angle_start_y_lm - 90, -self.angle_start_z_lm )
        returner_ru = fk(self.angle_start_x_ru - 90, self.angle_start_y_ru - 90, -self.angle_start_z_ru )
        returner_rd = fk(self.angle_start_x_rd - 90, self.angle_start_y_rd - 90, -self.angle_start_z_rd )

        converter_lm = [float(konv) for konv in returner_lm] 
        converter_ru = [float(konv) for konv in returner_ru]
        converter_rd = [float(konv) for konv in returner_rd]

        #print('pozcyja fk lm :', converter, "\n")
        Z = self.Z 

        lm_end_x = 0.0
        lm_end_y = 121.0

        ru_end_x = -85.5
        ru_end_y = 85.5 

        rd_end_x = -85.5
        rd_end_y = 85.5 

        msg_lm.theta_1 = self.angle_start_x_lm
        msg_lm.theta_2 = self.angle_start_y_lm
        msg_lm.theta_3 = self.angle_start_z_lm 

        msg_ru.theta_1 = self.angle_start_x_ru
        msg_ru.theta_2 = self.angle_start_y_ru
        msg_ru.theta_3 = self.angle_start_z_ru 

        msg_rd.theta_1 = self.angle_start_x_rd
        msg_rd.theta_2 = self.angle_start_y_rd
        msg_rd.theta_3 = self.angle_start_z_rd 

        breaker_lm = False
        breaker_ru = False
        breaker_rd = False

        iter = 0
        for j_lm, j_ru, j_rd in zip_longest(bc(converter_lm[0], converter_lm[1], converter_lm[2], lm_end_x, lm_end_y, Z), bc(converter_ru[0], converter_ru[1], converter_ru[2], ru_end_x, ru_end_y, Z), bc(converter_rd[0], converter_rd[1], converter_rd[2], rd_end_x, rd_end_y, Z), fillvalue= [10000, 10000, 10000]):
            
            if j_lm[0] != 10000:
                lm_end_x = j_lm[0]
                lm_end_y = j_lm[1]
            if j_ru[0] != 10000:
                ru_end_x = j_ru[0]
                ru_end_y = j_ru[1]
            if j_rd[0] != 10000:
                ru_end_x = j_ru[0]
                ru_end_y = j_ru[1]

            #print("bezier position :", j, "\n")      
            for z_lm, z_rd, z_ru in zip_longest(ik(j_lm[0], j_lm[1], j_lm[2], msg_lm.theta_1- 90, msg_lm.theta_2 - 90, -msg_lm.theta_3 ), ik(j_rd[0], j_rd[1], j_rd[2], msg_rd.theta_1- 90, msg_rd.theta_2 - 90, -msg_rd.theta_3), ik(j_ru[0], j_ru[1], j_ru[2], msg_ru.theta_1- 90, msg_ru.theta_2 - 90, -msg_ru.theta_3), fillvalue= [10000, 10000, 10000]):
                if (Sensor_info.sensor_5 >= 300 and iter > 3) or j_lm[0] == 10000 or z_lm[0] == 10000:
                    breaker_lm = True
                if (Sensor_info.sensor_3 >= 500 and iter > 3) or j_rd[0] == 10000 or z_rd[0] == 10000:
                    breaker_rd = True
                if (Sensor_info.sensor_1 >= 500 and iter > 3) or j_ru[0] == 10000 or z_ru[0] == 10000:    
                    breaker_ru = True

                if breaker_lm == False:
                    convert_lm = [float(konv) for konv in z_lm]
                    msg_lm.theta_1 = convert_lm[0] 
                    msg_lm.theta_2 = convert_lm[1] 
                    msg_lm.theta_3 = convert_lm[2]
                    self.publisher_LM.publish(msg_lm)
                    #print("msg_lm ", msg_lm, "\n")
                if breaker_rd == False:
                    convert_rd = [float(konv) for konv in z_rd]
                    msg_rd.theta_1 = convert_rd[0] 
                    msg_rd.theta_2 = convert_rd[1] 
                    msg_rd.theta_3 = convert_rd[2]
                    self.publisher_RD.publish(msg_rd)
                    #print("msg_rd ", msg_rd, "\n")
                if breaker_ru == False:
                    convert_ru = [float(konv) for konv in z_ru]
                    msg_ru.theta_1 = convert_ru[0] 
                    msg_ru.theta_2 = convert_ru[1] 
                    msg_ru.theta_3 = convert_ru[2]
                    self.publisher_RU.publish(msg_ru)
                    #print("msg_ru ", msg_ru, "\n")

                time.sleep( 0.00002) 
                #print(" all thetas: " ,msg.theta_1, msg.theta_2, msg.theta_3)
            iter += 1

        Z_lm = Z
        Z_rd = Z
        Z_ru = Z

        if Sensor_info.sensor_5 <= 300 :
            Z_lm = -215.0
        if Sensor_info.sensor_3 <= 500 :
            Z_rd = -215.0  
        if Sensor_info.sensor_1 <= 500 :
            Z_ru = -215.0 
        
        breaker_ru = False
        breaker_rd = False
        breaker_lm = False

        for z_lm, z_rd, z_ru in zip_longest(ik(lm_end_x, lm_end_y, Z_lm, msg_lm.theta_1 -90.0, msg_lm.theta_2 -90.0, -msg_lm.theta_3 ), ik(rd_end_x, rd_end_y, Z_rd, msg_rd.theta_1 -90.0, msg_rd.theta_2 -90.0, -msg_rd.theta_3), ik(ru_end_x, ru_end_y, Z_ru, msg_ru.theta_1 -90.0, msg_ru.theta_2 -90.0, -msg_ru.theta_3), fillvalue= [10000, 10000, 10000]):
            if Sensor_info.sensor_5 >= 300 or z_lm[0] == 10000:
                breaker_lm = True
            if Sensor_info.sensor_3 >= 500 or z_rd[0] == 10000:
                breaker_rd = True
            if Sensor_info.sensor_1 >= 500 or z_ru[0] == 10000:
                breaker_ru = True

            if breaker_lm == False:
                convert_lm = [float(konv) for konv in z_lm]
                msg_lm.theta_1 = convert_lm[0] 
                msg_lm.theta_2 = convert_lm[1] 
                msg_lm.theta_3 = convert_lm[2]
                self.publisher_LM.publish(msg_lm)
                #print("msg_lm ", msg_lm, "\n")
            if breaker_rd == False:
                convert_rd = [float(konv) for konv in z_rd]
                msg_rd.theta_1 = convert_rd[0] 
                msg_rd.theta_2 = convert_rd[1] 
                msg_rd.theta_3 = convert_rd[2]
                self.publisher_RD.publish(msg_rd)
                #print("msg_rd ", msg_rd, "\n")
            if breaker_ru == False:
                convert_ru = [float(konv) for konv in z_ru]
                msg_ru.theta_1 = convert_ru[0] 
                msg_ru.theta_2 = convert_ru[1] 
                msg_ru.theta_3 = convert_ru[2]
                self.publisher_RU.publish(msg_ru)
                #print("msg_ru ", msg_ru, "\n")

            time.sleep(0.00002) 
            #print(" all thetas: " ,msg.theta_1, msg.theta_2, msg.theta_3)

        msg_1.semafor_lm = int(1)
        msg_1.semafor_rd = int(1)
        msg_1.semafor_ru = int(1)
        self.publisher_work.publish(msg_1)
        print("zrobiono")
        self.ultra(msg_lm, msg_rd, msg_ru)
    
    def ultra(self, msg_lm, msg_rd, msg_ru):
        self.timer_01 = self.create_timer(0.1, self.ultra)
        self.publisher_LM.publish(msg_lm)
        self.publisher_RD.publish(msg_rd)
        self.publisher_RU.publish(msg_ru)
        self.timer_01.cancel()
            

def main(args=None):
    rclpy.init(args=args)
    node = stand_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()