#!/usr/bin/env python3
import sys
import os
import time
# --------------------------------------- add dependcies to actual path folder 
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import pygame

# ----------- pygame controller init ---------------
from pygame.locals import *

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .stand import stand
from .stand_in_loop_ld import stand_loop as stand_ld 
from .stand_in_loop_lm import stand_loop as stand_lm
from .stand_in_loop_lu import stand_loop as stand_lu
from .stand_in_loop_rd import stand_loop as stand_rd
from .stand_in_loop_rm import stand_loop as stand_rm
from .stand_in_loop_ru import stand_loop as stand_ru

from .leg_left_down_phase_One import down_left as walker_left_down_phase_I
from .leg_left_down_phase_Two import down_left as walker_left_down_phase_II
from .leg_left_mid_phase_one import mid_left as walker_left_mid_phase_I
from .leg_left_mid_phase_two import mid_left as walker_left_mid_phase_II
from .leg_left_up_phase_One import up_left as walker_left_up_phase_I
from .leg_left_up_phase_Two import up_left as walker_left_up_phase_II
from .leg_right_down_phase_one import down_right as walker_right_down_phase_I
from .leg_right_down_phase_two import down_right as walker_right_down_phase_II
from .leg_right_mid_phase_One import mid_right as walker_right_mid_phase_I
from .leg_right_mid_phase_Two import mid_right as walker_right_mid_phase_II
from .leg_right_up_phase_one import up_right as walker_right_up_phase_I
from .leg_right_up_phase_two import up_right as walker_right_up_phase_II

from hexapod_message.msg import ThetasAll, Semafor, Sygnalizer, ThetasLegLeftMiddle
from std_msgs.msg import Bool, Float32

#  ------------------------------- definitions of nodes and use of functions callbacks --------------------------------------------
class pub_shutdown(Node):
    def __init__(self):
        super().__init__('main_controller_shutdown')
        shutdown_m = MutuallyExclusiveCallbackGroup()
        self.shutdown_pub = self.create_publisher(Bool, 'shutdown', 10, callback_group = shutdown_m)
        self.timer = self.create_timer(1, self.shutdown_pub_callback)
    
    def shutdown_pub_callback(self):
        global x_end 
        msg = Bool()
        msg.data = x_end
        self.shutdown_pub.publish(msg)

class speed_pub(Node):
    def __init__(self):
        super().__init__('main_controller_speed')
        speed_m = MutuallyExclusiveCallbackGroup()
        self.shutdown_pub = self.create_publisher(Float32, 'speed', 10, callback_group = speed_m)
        self.timer = self.create_timer(1, self.speed_pub_callback)
    
    def speed_pub_callback(self):
        global speed 
        msg = Float32()
        msg.data = speed
        self.shutdown_pub.publish(msg)

class signal_sub(Node):
    def __init__(self):
        super().__init__('main_controller_sygnalizer')
        self.signal = self.create_subscription(Sygnalizer,'Donesend', self.signaliser, 10)
    
    def signaliser(self, msg: Sygnalizer):
        if msg.z_rm != 0.0:
            global z_RM 
            z_RM = msg.z_rm
        if msg.z_lm != 0.0:
            global z_LM
            z_LM = msg.z_lm
        if msg.z_ld != 0.0:
            global z_LD
            z_LD = msg.z_ld
        if msg.z_lu != 0.0:
            global z_LU
            z_LU = msg.z_lu
        if msg.z_rd != 0.0:
            global z_RD
            z_RD = msg.z_rd
        if msg.z_ru != 0.0:
            global z_RU
            z_RU = msg.z_ru 

class subscriber_semafor(Node):
    def __init__(self):
        super().__init__('main_controller_sema')
        cos_1 = MutuallyExclusiveCallbackGroup()
        self.semafor_subscriber = self.create_subscription(Semafor,'Semafor', self.semafor_callback, 3, callback_group=cos_1)
        self.msg_received = False
        self.msg = None 

    def semafor_callback(self, msg:Semafor):
        self.msg = msg
        self.msg_received = True

class subscriber_theta_lm(Node):
    def __init__(self):
        super().__init__('main_controller_theta_lm')   
        cos_LM = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_lm = self.create_subscription( ThetasLegLeftMiddle, 'AnglesLM', self.servo_theta_lm_callback , 10, callback_group=cos_LM)

    def servo_theta_lm_callback(self, msg:ThetasLegLeftMiddle):
        global theta_val_LM
        theta_val_LM.theta_1 = msg.theta_1
        theta_val_LM.theta_2 = msg.theta_2
        theta_val_LM.theta_3 = msg.theta_3
        
class subscriber_theta_ld(Node):
    def __init__(self):
        super().__init__('main_controller_theta_ld')
        cos_LD = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_ld = self.create_subscription(ThetasLegLeftMiddle, 'AnglesLD', self.servo_theta_ld_callback , 10, callback_group=cos_LD)
    
    def servo_theta_ld_callback(self, msg: ThetasLegLeftMiddle):
        global theta_val_LD
        theta_val_LD.theta_1 = msg.theta_1
        theta_val_LD.theta_2 = msg.theta_2
        theta_val_LD.theta_3 = msg.theta_3
        
class subscriber_theta_lu(Node):
    def __init__(self):
        super().__init__('main_controller_theta_lu')
        cos_LU = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_lu = self.create_subscription(ThetasLegLeftMiddle, 'AnglesLU', self.servo_theta_lu_callback , 10, callback_group=cos_LU)
        
    def servo_theta_lu_callback(self, msg: ThetasLegLeftMiddle):
        global theta_val_LU
        theta_val_LU.theta_1 = msg.theta_1
        theta_val_LU.theta_2 = msg.theta_2
        theta_val_LU.theta_3 = msg.theta_3

class subscriber_theta_rm(Node):
    def __init__(self):
        super().__init__('main_controller_theta_rm')
        cos_RM = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_rm = self.create_subscription(ThetasLegLeftMiddle, 'AnglesRM', self.servo_theta_rm_callback , 10, callback_group=cos_RM)
    
    def servo_theta_rm_callback(self, msg: ThetasLegLeftMiddle):
        global theta_val_RM
        theta_val_RM.theta_1 = msg.theta_1
        theta_val_RM.theta_2 = msg.theta_2
        theta_val_RM.theta_3 = msg.theta_3
    
class subscriber_theta_rd(Node):
    def __init__(self):
        super().__init__('main_controller_theta_rd')
        cos_RD = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_rd = self.create_subscription(ThetasLegLeftMiddle, 'AnglesRD', self.servo_theta_rd_callback , 10, callback_group=cos_RD)
    
    def servo_theta_rd_callback(self, msg: ThetasLegLeftMiddle):
        global theta_val_RD
        theta_val_RD.theta_1 = msg.theta_1
        theta_val_RD.theta_2 = msg.theta_2
        theta_val_RD.theta_3 = msg.theta_3

class subscriber_theta_ru(Node):
    def __init__(self):
        super().__init__('main_controller_theta_ru')
        cos_RU = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_ru = self.create_subscription(ThetasLegLeftMiddle, 'AnglesRU', self.servo_theta_ru_callback , 10, callback_group=cos_RU)

    def servo_theta_ru_callback(self, msg: ThetasLegLeftMiddle):
        global theta_val_RU
        theta_val_RU.theta_1 = msg.theta_1
        theta_val_RU.theta_2 = msg.theta_2
        theta_val_RU.theta_3 = msg.theta_3
 

# --------------------------------- Main program ------------------------
def main(args=None):
    rclpy.init(args=args)
    pygame.init()

    clock = pygame.time.Clock()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    motion = [0, 0]

    stander = stand()
    semafor_sub = subscriber_semafor()

    Z = -207.0 # height of stance

    global theta_val_LM
    global theta_val_RM
    global theta_val_LU
    global theta_val_LD
    global theta_val_RU
    global theta_val_RD
    global canceler_sem
    global z_LM
    global z_RD
    global z_RU
    global z_RM
    global z_LD
    global z_LU

    global x_end
    global speed

    x_end = False
    speed = 0.0

    speed_publisher = speed_pub()
    rclpy.spin_once(speed_publisher)
    shutdown_publisher = pub_shutdown()
    rclpy.spin_once(shutdown_publisher)

    z_LM = Z
    z_RD = Z
    z_RU = Z
    z_RM = Z
    z_LD = Z
    z_LU = Z

    canceler_sem = False

    # --------- stand pose initialition --------------
    result_theta = stander.publish_values()
    theta_val_LM = result_theta['LM']
    theta_val_RM = result_theta['RM']
    theta_val_LU = result_theta['LU']
    theta_val_LD = result_theta['LD']
    theta_val_RU = result_theta['RU']
    theta_val_RD = result_theta['RD']

    stander.destroy_node()
    signal_subscribe = signal_sub() 

    number = False

    # --------------- main loop of controller -------------------
    while True:
        # ------------ axis controll ----------------
        if abs(motion[0]) < 0.02:
            motion[0] = 0.0
        elif motion[0] > 1.0: 
            motion[0] = 1.0
        elif motion[0] < -1.0:
            motion[0] = -1.0

        if abs(motion[1]) < 0.02:
            motion[1] = 0.0

        # ------------ control of tourning axis and scaling of value --------
        if motion[0] >= 0 :
            tourning_var = 1200.0 - (1200.0 * motion[0])
        elif motion[0] < 0 :
            tourning_var = -(1200.0 + (1200.0 * motion[0]))

        if tourning_var == 1200 or tourning_var == -1200:
            tourning_var = 1000000

        print(" tv  :", tourning_var, "\n")
        
        # ------------ control of speed axis and scaling of value ----------
        
        if motion[1] < 0 :
            speed = 0.005 + (0.0040 * ( motion[1]))
        elif motion[1] > 0:
            speed = -(0.005 + (0.0040 * ( -motion[1])) )  
        elif motion[1] == 0:
            speed = 0.0

        print(" speed :", speed, "\n")
        
        
        # ---------- !!!!!!!!!!! legs controll !!!!!!!!!!!!!!!!!! ------------
        if speed == 0.0:
         # if nothing appears on axis up-down, just stand in position
            # creating of stand threads, !first cycle!
            stand_lm_loop = stand_lm(theta_val_LM.theta_1, theta_val_LM.theta_2, theta_val_LM.theta_3, Z)
            stand_ru_loop = stand_ru(theta_val_RU.theta_1, theta_val_RU.theta_2, theta_val_RU.theta_3, Z)
            stand_rd_loop = stand_rd(theta_val_RD.theta_1, theta_val_RD.theta_2, theta_val_RD.theta_3, Z)
            theta_sub_lm = subscriber_theta_lm()
            theta_sub_rd = subscriber_theta_rd()
            theta_sub_ru = subscriber_theta_ru()
            
            executor_one = MultiThreadedExecutor(num_threads=4)

            executor_one.add_node(stand_lm_loop)
            executor_one.add_node(stand_rd_loop)
            executor_one.add_node(stand_ru_loop)

            semafor_lm = 0
            semafor_rd = 0
            semafor_ru = 0

            if number == True:
                rclpy.spin_once(signal_subscribe)

            while True: 
                executor_one.spin_once()
                rclpy.spin_once(semafor_sub)                
                if semafor_sub.msg_received:
                    if semafor_sub.msg.semafor_lm == 1:
                        semafor_lm = 1

                    if semafor_sub.msg.semafor_rd == 1:
                        semafor_rd = 1

                    if semafor_sub.msg.semafor_ru == 1:
                        semafor_ru = 1

                    if semafor_lm == 1 and semafor_rd == 1 and semafor_ru == 1:
                        break
                
            rclpy.spin_once(theta_sub_lm)
            rclpy.spin_once(theta_sub_rd)
            rclpy.spin_once(theta_sub_ru)
            
            executor_one.shutdown()

            stand_lm_loop.destroy_node()
            stand_ru_loop.destroy_node()
            stand_rd_loop.destroy_node()
            theta_sub_lm.destroy_node()
            theta_sub_rd.destroy_node()
            theta_sub_ru.destroy_node()

            print("zakonczono exec 1")

          # creating of stand threads, !Second cycle!
            stand_rm_loop = stand_rm(theta_val_RM.theta_1, theta_val_RM.theta_2, theta_val_RM.theta_3, Z)
            stand_ld_loop = stand_ld(theta_val_LD.theta_1, theta_val_LD.theta_2, theta_val_LD.theta_3, Z)
            stand_lu_loop = stand_lu(theta_val_LU.theta_1, theta_val_LU.theta_2, theta_val_LU.theta_3, Z)
            theta_sub_rm = subscriber_theta_rm()
            theta_sub_lu = subscriber_theta_lu()
            theta_sub_ld = subscriber_theta_ld()

            executor_two = MultiThreadedExecutor(num_threads=4)

            executor_two.add_node(stand_rm_loop)
            executor_two.add_node(stand_ld_loop)
            executor_two.add_node(stand_lu_loop)
            
            semafor_rm = 0
            semafor_ld = 0
            semafor_lu = 0

            rclpy.spin_once(signal_subscribe)

            while True:  
                executor_two.spin_once()
                rclpy.spin_once(semafor_sub)

                if semafor_sub.msg_received:

                    if semafor_sub.msg.semafor_rm == 1:
                        semafor_rm = 1
                    if semafor_sub.msg.semafor_ld == 1:
                        semafor_ld = 1
                    if semafor_sub.msg.semafor_lu == 1:
                        semafor_lu = 1

                    if semafor_rm == 1 and semafor_ld == 1 and semafor_lu == 1:
                        break

            rclpy.spin_once(theta_sub_rm)
            rclpy.spin_once(theta_sub_ld)
            rclpy.spin_once(theta_sub_lu)

            executor_two.shutdown()

            stand_rm_loop.destroy_node()
            stand_ld_loop.destroy_node()
            stand_lu_loop.destroy_node()
            theta_sub_rm.destroy_node()
            theta_sub_ld.destroy_node()
            theta_sub_lu.destroy_node()
            print("zakonczono exec 2")
            number = False

        else:
            number = True
         # first cycle of tripod walking :
            # creating of threads
            rclpy.spin_once(signal_subscribe)
            
            walk_lm_loop = walker_left_mid_phase_I(theta_val_LM.theta_1, theta_val_LM.theta_2, theta_val_LM.theta_3, tourning_var, speed, z_LM)
            walk_rd_loop = walker_right_down_phase_I(theta_val_RD.theta_1, theta_val_RD.theta_2, theta_val_RD.theta_3, tourning_var, speed, z_RD)
            walk_ru_loop = walker_right_up_phase_I(theta_val_RU.theta_1, theta_val_RU.theta_2, theta_val_RU.theta_3, tourning_var, speed, z_RU)
            walk_rm_loop = walker_right_mid_phase_I(theta_val_RM.theta_1, theta_val_RM.theta_2, theta_val_RM.theta_3, tourning_var, speed, Z)
            walk_lu_loop = walker_left_up_phase_I(theta_val_LU.theta_1, theta_val_LU.theta_2, theta_val_LU.theta_3, tourning_var, speed, Z)
            walk_ld_loop = walker_left_down_phase_I(theta_val_LD.theta_1, theta_val_LD.theta_2, theta_val_LD.theta_3, tourning_var, speed, Z)

            theta_sub_lm = subscriber_theta_lm()
            theta_sub_rd = subscriber_theta_rd()
            theta_sub_ru = subscriber_theta_ru()
            theta_sub_rm = subscriber_theta_rm()
            theta_sub_ld = subscriber_theta_ld()
            theta_sub_lu = subscriber_theta_lu()

            executor_one_walk = MultiThreadedExecutor(num_threads=6)

            semafor_lm = 0
            semafor_rd = 0
            semafor_ru = 0
            semafor_rm = 0
            semafor_ld = 0
            semafor_lu = 0

            executor_one_walk.add_node(walk_lm_loop)
            executor_one_walk.add_node(walk_rd_loop)
            executor_one_walk.add_node(walk_ru_loop)
            executor_one_walk.add_node(walk_rm_loop)
            executor_one_walk.add_node(walk_ld_loop)
            executor_one_walk.add_node(walk_lu_loop)

            while True:
                executor_one_walk.spin_once()
                rclpy.spin_once(semafor_sub)

                if semafor_sub.msg_received:

                    if semafor_sub.msg.semafor_lm == 1:
                        semafor_lm = 1
                    if semafor_sub.msg.semafor_rd == 1:
                        semafor_rd = 1 
                    if semafor_sub.msg.semafor_ru == 1:
                        semafor_ru = 1
                    if semafor_sub.msg.semafor_rm == 1:
                        semafor_rm = 1
                    if semafor_sub.msg.semafor_ld == 1:
                        semafor_ld = 1
                    if semafor_sub.msg.semafor_lu == 1:
                        semafor_lu = 1 

                    if semafor_lm == 1 and semafor_rd == 1 and semafor_ru == 1 and semafor_rm == 1 and semafor_lu == 1 and semafor_ld == 1:
                        break

            rclpy.spin_once(theta_sub_lm)
            rclpy.spin_once(theta_sub_rd)
            rclpy.spin_once(theta_sub_ru)
            rclpy.spin_once(theta_sub_ld)
            rclpy.spin_once(theta_sub_rm)
            rclpy.spin_once(theta_sub_lu)

            executor_one_walk.shutdown()

            time.sleep(0.0002)
            walk_lm_loop.destroy_node()
            walk_rd_loop.destroy_node()
            walk_ru_loop.destroy_node()
            walk_rm_loop.destroy_node()
            walk_lu_loop.destroy_node()
            walk_ld_loop.destroy_node()
            theta_sub_lm.destroy_node()
            theta_sub_rd.destroy_node()
            theta_sub_ru.destroy_node()
            theta_sub_ld.destroy_node()
            theta_sub_lu.destroy_node()
            theta_sub_rm.destroy_node()

            z_LM = Z
            z_RD = Z
            z_RU = Z

         # second cycle of tripod walking :
            # creating of threads
            rclpy.spin_once(signal_subscribe)

            walk_lm_loop = walker_left_mid_phase_II(theta_val_LM.theta_1, theta_val_LM.theta_2, theta_val_LM.theta_3, tourning_var, speed, Z)
            walk_rd_loop = walker_right_down_phase_II(theta_val_RD.theta_1, theta_val_RD.theta_2, theta_val_RD.theta_3, tourning_var, speed, Z)
            walk_ru_loop = walker_right_up_phase_II(theta_val_RU.theta_1, theta_val_RU.theta_2, theta_val_RU.theta_3, tourning_var, speed, Z)
            walk_rm_loop = walker_right_mid_phase_II(theta_val_RM.theta_1, theta_val_RM.theta_2, theta_val_RM.theta_3, tourning_var, speed, z_RM)
            walk_lu_loop = walker_left_up_phase_II(theta_val_LU.theta_1, theta_val_LU.theta_2, theta_val_LU.theta_3, tourning_var, speed, z_LU)
            walk_ld_loop = walker_left_down_phase_II(theta_val_LD.theta_1, theta_val_LD.theta_2, theta_val_LD.theta_3, tourning_var, speed, z_LD)

            theta_sub_lm = subscriber_theta_lm()
            theta_sub_rd = subscriber_theta_rd()
            theta_sub_ru = subscriber_theta_ru()
            theta_sub_rm = subscriber_theta_rm()
            theta_sub_ld = subscriber_theta_ld()
            theta_sub_lu = subscriber_theta_lu()

            executor_two_walk = MultiThreadedExecutor(num_threads=6)

            semafor_lm = 0
            semafor_rd = 0
            semafor_ru = 0
            semafor_rm = 0
            semafor_ld = 0
            semafor_lu = 0

            executor_two_walk.add_node(walk_lm_loop)
            executor_two_walk.add_node(walk_rd_loop)
            executor_two_walk.add_node(walk_ru_loop)
            executor_two_walk.add_node(walk_rm_loop)
            executor_two_walk.add_node(walk_ld_loop)
            executor_two_walk.add_node(walk_lu_loop)

            while True:
                executor_two_walk.spin_once()
                rclpy.spin_once(semafor_sub)
                
                if semafor_sub.msg_received:
                    if semafor_sub.msg.semafor_rm == 1:
                        semafor_rm = 1
                    if semafor_sub.msg.semafor_ld == 1:
                        semafor_ld = 1 
                    if semafor_sub.msg.semafor_lu == 1:
                        semafor_lu = 1
                    if semafor_sub.msg.semafor_lm == 1:
                        semafor_lm = 1
                    if semafor_sub.msg.semafor_rd == 1:
                        semafor_rd = 1
                    if semafor_sub.msg.semafor_ru == 1:
                        semafor_ru = 1 

                    if semafor_lm == 1 and semafor_rd == 1 and semafor_ru == 1 and semafor_rm == 1 and semafor_lu == 1 and semafor_ld == 1:
                        break
            
            print("doszedłem tu!")
            rclpy.spin_once(theta_sub_rm)
            rclpy.spin_once(theta_sub_lu)
            rclpy.spin_once(theta_sub_ld)
            rclpy.spin_once(theta_sub_lm)
            rclpy.spin_once(theta_sub_rd)
            rclpy.spin_once(theta_sub_ru)

            executor_two_walk.shutdown()

            time.sleep(0.002)
            walk_lm_loop.destroy_node()
            walk_rd_loop.destroy_node()
            walk_ru_loop.destroy_node()
            walk_rm_loop.destroy_node()
            walk_lu_loop.destroy_node()
            walk_ld_loop.destroy_node()
            theta_sub_lm.destroy_node()
            theta_sub_rd.destroy_node()
            theta_sub_ru.destroy_node()
            theta_sub_ld.destroy_node()
            theta_sub_lu.destroy_node()
            theta_sub_rm.destroy_node()

            z_RM = Z
            z_LD = Z
            z_LU = Z        


        # ------------ controller events -------------

        for event in pygame.event.get():

            if event.type == JOYBUTTONDOWN:
                if event.button == 1:
                    x_end = True
                    speed_publisher.destroy_node()
                    shutdown_publisher.destroy_node()
                    rclpy.shutdown()
                    pygame.quit()
                    sys.exit()

            if event.type == JOYAXISMOTION:
                if event.axis > 2 and event.axis < 5:
                    motion[event.axis-3] = event.value

            if event.type == JOYDEVICEADDED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

            if event.type == JOYDEVICEREMOVED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

            if event.type == QUIT:
                x_end = True
                speed_publisher.destroy_node()
                shutdown_publisher.destroy_node()
                pygame.quit()
                rclpy.shutdown()
                sys.exit()

            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    x_end = True
                    speed_publisher.destroy_node()
                    shutdown_publisher.destroy_node()
                    rclpy.shutdown()
                    pygame.quit()
                    sys.exit()

        clock.tick(60)

if __name__ == '__main__':
    main()