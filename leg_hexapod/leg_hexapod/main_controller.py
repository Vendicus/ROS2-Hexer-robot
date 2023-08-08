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

from hexapod_message.msg import Sensor, ThetasAll, Semafor,ThetasLegLeftDown, ThetasLegLeftMiddle, ThetasLegLeftup, ThetasLegRightDown, ThetasLegRightMiddle, ThetasLegRightUp, Sygnalizer
from itertools import zip_longest

#  ------------------------------- definitions of nodes and use of functions callbacks --------------------------------------------
class sensors_subscription(Node):
    def __init__(self):
        super().__init__('main_controller_sensor_sub')
        sensor_1 = MutuallyExclusiveCallbackGroup()
        self.subscriber_sensor = self.create_subscription(Sensor, 'Sensor', self.subscriber, 2, callback_group=sensor_1)

    def subscriber(self, sensor_info: Sensor):
        global sensors_data 
        sensors_data = sensor_info

class sender_first_cycle(Node):
    def __init__(self):
        super().__init__('main_controller_sender_stand_cycle_one')
        stand_lm = MutuallyExclusiveCallbackGroup()
        stand_rd = MutuallyExclusiveCallbackGroup()
        stand_ru = MutuallyExclusiveCallbackGroup()
        sensor = MutuallyExclusiveCallbackGroup()
        
        self.publisher_LM = self.create_publisher(ThetasLegLeftMiddle, 'anglesMLLeg', 10, callback_group=stand_lm)
        self.publisher_RD = self.create_publisher(ThetasLegRightDown, 'anglesDRLeg', 10, callback_group=stand_rd)
        self.publisher_RU = self.create_publisher(ThetasLegRightUp, 'anglesURLeg', 10, callback_group=stand_ru)
        self.timer = self.create_timer(0.01, self.send_one, callback_group=sensor)
    
    def send_one(self):
        global canceler_sem
        global theta_val_LM
        global theta_val_RD
        global theta_val_RU
        if canceler_sem == False:
            msg_lm = ThetasLegLeftMiddle()
            msg_rd = ThetasLegRightDown()
            msg_ru = ThetasLegRightUp()
            sen_lm = False
            sen_rd = False
            sen_ru = False

            global sensors_data 
            sensors_data = None

            global list_lm
            global list_rd
            global list_ru

            length_lm = len(list_lm)
            length_rd = len(list_rd)
            length_ru = len(list_ru)

            length_max = max(length_lm, length_rd, length_ru)

            nested_list_lm = [list_lm[i:i+3] for i in range(0, length_lm, 3)]
            nested_list_rd = [list_rd[i:i+3] for i in range(0, length_rd, 3)]
            nested_list_ru = [list_ru[i:i+3] for i in range(0, length_ru, 3)]

            sema_sec_one = False
            sema_sec_two = False
            sema_sec_three = False
            iteration = 0

            sleeper = 0.01

            for ii, jj, zz in zip_longest(nested_list_lm, nested_list_rd, nested_list_ru, fillvalue=[None,None,None]):   
                print(sensors_data)
                if sensors_data != None:
                    if sensors_data.sensor_2 >= 300 and iteration > 30:
                        sen_lm = True
                if ii[0] != None and sen_lm == False:
                    msg_lm.theta_1 = float(ii[0])
                    msg_lm.theta_2 = float(ii[1])
                    msg_lm.theta_3 = float(ii[2])
                    self.publisher_LM.publish(msg_lm)
                    sema_sec_one = True

                if sensors_data != None:
                    if sensors_data.sensor_6 >= 500 and iteration > 30:
                        sen_rd = True
                if jj[0] != None and sen_rd == False:
                    msg_rd.theta_1 = float(jj[0])
                    msg_rd.theta_2 = float(jj[1])
                    msg_rd.theta_3 = float(jj[2])
                    self.publisher_RD.publish(msg_rd)
                    sema_sec_two = True

                #if sensors_data != None:
                    #if sensors_data.sensor_4 >= 500 and iteration > 30:
                    #    sen_ru = True
                if zz[0] != None and sen_ru == False:
                    msg_ru.theta_1 = float(zz[0])
                    msg_ru.theta_2 = float(zz[1])
                    msg_ru.theta_3 = float(zz[2])
                    self.publisher_RU.publish(msg_ru)
                    sema_sec_three = True

                iteration += 1 
                
                #smoothing steps
                if iteration < (length_max/2):
                    iterator = 1 + iteration/5
                else:
                    iterator = -1 -(length_max - iteration)/5

                smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
                time.sleep(smooth_step)

            if sema_sec_one == True:
                theta_val_LM.theta_1 = msg_lm.theta_1
                theta_val_LM.theta_2 = msg_lm.theta_2
                theta_val_LM.theta_3 = msg_lm.theta_3
            if sema_sec_two == True:
                theta_val_RD.theta_1 = msg_rd.theta_1
                theta_val_RD.theta_2 = msg_rd.theta_2
                theta_val_RD.theta_3 = msg_rd.theta_3
            if sema_sec_three == True:
                theta_val_RU.theta_1 = msg_ru.theta_1
                theta_val_RU.theta_2 = msg_ru.theta_2
                theta_val_RU.theta_3 = msg_ru.theta_3

        self.timer.cancel()
        canceler_sem = True
    
class sender_second_cycle(Node):
    def __init__(self):
        super().__init__('main_controller_sender_stand_cycle_two')
        stand_rm = MutuallyExclusiveCallbackGroup()
        stand_ld = MutuallyExclusiveCallbackGroup()
        stand_lu = MutuallyExclusiveCallbackGroup()
        sensor = MutuallyExclusiveCallbackGroup()
        
        self.publisher_RM = self.create_publisher(ThetasLegRightMiddle, 'anglesMRLeg', 10, callback_group=stand_rm)
        self.publisher_LD = self.create_publisher(ThetasLegLeftDown, 'anglesDLLeg', 10, callback_group=stand_ld)
        self.publisher_LU = self.create_publisher(ThetasLegLeftup, 'anglesULLeg', 10, callback_group=stand_lu)
        self.timer = self.create_timer(0.01, self.send_two, callback_group=sensor)
    
    def send_two(self):
        global canceler_sem
        if canceler_sem == False:
            msg_rm = ThetasLegRightMiddle()
            msg_ld = ThetasLegLeftDown()
            msg_lu = ThetasLegLeftup()
            sen_rm = False
            sen_ld = False
            sen_lu = False
            
            global sensors_data 
            sensors_data = None

            global list_rm
            global list_ld
            global list_lu
            global theta_val_RM
            global theta_val_LD
            global theta_val_LU

            length_rm = len(list_rm)
            length_ld = len(list_ld)
            length_lu = len(list_lu)

            length_max = max(length_rm, length_ld, length_lu)
            sleeper = 0.01

            nested_list_rm = [list_rm[i:i+3] for i in range(0, length_rm, 3)]
            nested_list_ld = [list_ld[i:i+3] for i in range(0, length_ld, 3)]
            nested_list_lu = [list_lu[i:i+3] for i in range(0, length_lu, 3)]

            iteration = 0
            sema_sec_one = False
            sema_sec_two = False
            sema_sec_three = False
            for ii, jj, zz in zip_longest(nested_list_rm, nested_list_ld, nested_list_lu, fillvalue=[None,None,None]): 
                
                if sensors_data != None:
                    if sensors_data.sensor_5 >= 500 and iteration > 30:
                        sen_rm = True   
                if ii[0] != None and sen_rm == False:
                    #print(" i: ", ii[0])
                    msg_rm.theta_1 = float(ii[0])
                    msg_rm.theta_2 = float(ii[1])
                    msg_rm.theta_3 = float(ii[2])
                    self.publisher_RM.publish(msg_rm)
                    sema_sec_one = True
                
                if sensors_data != None:
                    if sensors_data.sensor_3 >= 500 and iteration > 30:
                        sen_ld = True
                if jj[0] != None and sen_ld == False:
                    #print("j :", jj)
                    msg_ld.theta_1 = float(jj[0])
                    msg_ld.theta_2 = float(jj[1])
                    msg_ld.theta_3 = float(jj[2])
                    self.publisher_LD.publish(msg_ld)
                    sema_sec_two = True

                if sensors_data != None:
                    if sensors_data.sensor_1 >= 500 and iteration > 30:
                        sen_lu = True
                    #print("błąd sensora!, iteracja : ", iteration)
                if zz[0] != None and sen_lu == False:
                    #print("z :", zz, " iteracja :", iteration)
                    msg_lu.theta_1 = float(zz[0])
                    msg_lu.theta_2 = float(zz[1])
                    msg_lu.theta_3 = float(zz[2])
                    self.publisher_LU.publish(msg_lu)
                    sema_sec_three = True

                iteration += 1 

                #smoothing steps
                if iteration < (length_max/2):
                    iterator = 1 + iteration/5
                else:
                    iterator = -1 -(length_max - iteration)/5
                    
                smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
                time.sleep(smooth_step)

            if sema_sec_one == True:
                theta_val_RM.theta_1 = msg_rm.theta_1
                theta_val_RM.theta_2 = msg_rm.theta_2
                theta_val_RM.theta_3 = msg_rm.theta_3
            if sema_sec_two == True:
                theta_val_LD.theta_1 = msg_ld.theta_1
                theta_val_LD.theta_2 = msg_ld.theta_2
                theta_val_LD.theta_3 = msg_ld.theta_3
            if sema_sec_three == True:
                theta_val_LU.theta_1 = msg_lu.theta_1
                theta_val_LU.theta_2 = msg_lu.theta_2
                theta_val_LU.theta_3 = msg_lu.theta_3
                #print("nested list lu :", msg_lu.theta_1, msg_lu.theta_2, msg_lu.theta_3)
        
        self.timer.cancel()
        canceler_sem = True
        self.destroy_node()

class sender_cycle_one_walk (Node):
    def __init__(self, speed):
        super().__init__('main_controller_sender_stand_cycle_one_walk')
        self.speed = speed

        stand_rm = MutuallyExclusiveCallbackGroup()
        stand_ld = MutuallyExclusiveCallbackGroup()
        stand_lu = MutuallyExclusiveCallbackGroup()
        stand_lm = MutuallyExclusiveCallbackGroup()
        stand_rd = MutuallyExclusiveCallbackGroup()
        stand_ru = MutuallyExclusiveCallbackGroup()
        sensor = MutuallyExclusiveCallbackGroup()
        
        self.publisher_LM = self.create_publisher(ThetasLegLeftMiddle, 'anglesMLLeg', 10, callback_group=stand_lm)
        self.publisher_RD = self.create_publisher(ThetasLegRightDown, 'anglesDRLeg', 10, callback_group=stand_rd)
        self.publisher_RU = self.create_publisher(ThetasLegRightUp, 'anglesURLeg', 10, callback_group=stand_ru)
        self.publisher_RM = self.create_publisher(ThetasLegRightMiddle, 'anglesMRLeg', 10, callback_group=stand_rm)
        self.publisher_LD = self.create_publisher(ThetasLegLeftDown, 'anglesDLLeg', 10, callback_group=stand_ld)
        self.publisher_LU = self.create_publisher(ThetasLegLeftup, 'anglesULLeg', 10, callback_group=stand_lu)

        self.timer = self.create_timer(0.01, self.sender_walker_one, callback_group=sensor)

    
    def sender_walker_one(self):
        global canceler_sem
        if canceler_sem == False:

            speed = self.speed
            msg_rm = ThetasLegRightMiddle()
            msg_ld = ThetasLegLeftDown()
            msg_lu = ThetasLegLeftup()
            msg_lm = ThetasLegLeftMiddle()
            msg_rd = ThetasLegRightDown()
            msg_ru = ThetasLegRightUp()

            sen_rm = False
            sen_ld = False
            sen_lu = False

            global sensors_data 
            global list_rm
            global list_ld
            global list_lu
            global list_lm
            global list_rd
            global list_ru

            sleeper = abs(speed) * 8
            length_rm = len(list_rm)
            length_ld = len(list_ld)
            length_lu = len(list_lu)
            length_lm = len(list_lm)
            length_rd = len(list_rd)
            length_ru = len(list_ru)
            length_max = max(length_rm,length_rd,length_ru,length_lm,length_ld,length_lu)

            global theta_val_LM
            global theta_val_RD
            global theta_val_RU
            global theta_val_RM
            global theta_val_LD
            global theta_val_LU

            global z_LD
            global z_LU
            global z_RM
            z_LD = None
            z_LU = None
            z_RM = None

            nested_list_rm = [list_rm[i:i+4] for i in range(0, length_rm, 4)]
            nested_list_ld = [list_ld[i:i+4] for i in range(0, length_ld, 4)]
            nested_list_lu = [list_lu[i:i+4] for i in range(0, length_lu, 4)]
            nested_list_lm = [list_lm[i:i+3] for i in range(0, length_lm, 3)]
            nested_list_rd = [list_rd[i:i+3] for i in range(0, length_rd, 3)]
            nested_list_ru = [list_ru[i:i+3] for i in range(0, length_ru, 3)]
            
            #print("nested list ld :", nested_list_ld)
            #print([[None] * 3]*4 + nested_list_lm )
            
            iteration = 0
            sema_sec_one = False
            sema_sec_two = False
            sema_sec_three = False
            sema_sec_four = False
            sema_sec_five = False
            sema_sec_six = False

            for ii, jj, zz, qq, ww, xx in zip_longest(nested_list_rm, nested_list_ld, nested_list_lu,[[None] * 3]*50 + nested_list_lm , [[None] * 3]*50 + nested_list_rd,[[None] * 3]*50 + nested_list_ru, fillvalue=[None,None,None, None]):   
                if sensors_data != None:
                    if sensors_data.sensor_5 >= 500 and iteration > 50 and ii[0] != None:
                        sen_rm = True   
                        z_RM = ii[3]
                if ii[0] != None and sen_rm == False:
                    #print(" i: ", ii[0])
                    msg_rm.theta_1 = float(ii[0])
                    msg_rm.theta_2 = float(ii[1])
                    msg_rm.theta_3 = float(ii[2])
                    self.publisher_RM.publish(msg_rm)
                    sema_sec_one = True
                
                if sensors_data != None:
                    if sensors_data.sensor_3 >= 500 and iteration > 50 and jj[0] != None:
                        sen_ld = True
                        z_LD = jj[3]
                if jj[0] != None and sen_ld == False:
                    #print("j :", jj)
                    msg_ld.theta_1 = float(jj[0])
                    msg_ld.theta_2 = float(jj[1])
                    msg_ld.theta_3 = float(jj[2])
                    self.publisher_LD.publish(msg_ld)
                    sema_sec_two = True

                if sensors_data != None:
                    if sensors_data.sensor_1 >= 500 and iteration > 50 and zz[0] != None:
                        sen_lu = True
                        z_LU = zz[3]
                        #print("błąd sensora!, iteracja : ", iteration)
                if zz[0] != None and sen_lu == False:
                    #print("z :", zz, " iteracja :", iteration)
                    msg_lu.theta_1 = float(zz[0])
                    msg_lu.theta_2 = float(zz[1])
                    msg_lu.theta_3 = float(zz[2])
                    self.publisher_LU.publish(msg_lu)
                    sema_sec_three = True

                if qq[0] != None and iteration > 50:
                    msg_lm.theta_1 = float(qq[0])
                    msg_lm.theta_2 = float(qq[1])
                    msg_lm.theta_3 = float(qq[2])
                    self.publisher_LM.publish(msg_lm)
                    sema_sec_four = True

                if ww[0] != None and iteration > 50:
                    msg_rd.theta_1 = float(ww[0])
                    msg_rd.theta_2 = float(ww[1])
                    msg_rd.theta_3 = float(ww[2])
                    self.publisher_RD.publish(msg_rd)
                    sema_sec_five = True

                if xx[0] != None and iteration > 50:
                    msg_ru.theta_1 = float(xx[0])
                    msg_ru.theta_2 = float(xx[1])
                    msg_ru.theta_3 = float(xx[2])
                    self.publisher_RU.publish(msg_ru)
                    sema_sec_six = True

                iteration += 1 

                #smoothing steps
                if iteration < (length_max/2):
                    iterator = 1 + iteration/5
                else:
                    iterator = -1 -(length_max - iteration)/5
                    
                smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
                time.sleep(smooth_step)


            if sema_sec_one == True:
                theta_val_RM.theta_1 = msg_rm.theta_1
                theta_val_RM.theta_2 = msg_rm.theta_2
                theta_val_RM.theta_3 = msg_rm.theta_3
            if sema_sec_two == True:
                theta_val_LD.theta_1 = msg_ld.theta_1
                theta_val_LD.theta_2 = msg_ld.theta_2
                theta_val_LD.theta_3 = msg_ld.theta_3
            if sema_sec_three == True:
                theta_val_LU.theta_1 = msg_lu.theta_1
                theta_val_LU.theta_2 = msg_lu.theta_2
                theta_val_LU.theta_3 = msg_lu.theta_3
            if sema_sec_four == True:
                theta_val_LM.theta_1 = msg_lm.theta_1
                theta_val_LM.theta_2 = msg_lm.theta_2
                theta_val_LM.theta_3 = msg_lm.theta_3
            if sema_sec_five == True:
                theta_val_RD.theta_1 = msg_rd.theta_1
                theta_val_RD.theta_2 = msg_rd.theta_2
                theta_val_RD.theta_3 = msg_rd.theta_3
            if sema_sec_six == True:
                theta_val_RU.theta_1 = msg_ru.theta_1
                theta_val_RU.theta_2 = msg_ru.theta_2
                theta_val_RU.theta_3 = msg_ru.theta_3

                #print("nested list lu :", msg_lu.theta_1, msg_lu.theta_2, msg_lu.theta_3)

            if z_LD == None:
                z_LD = -215.0
            if z_RM == None:
                z_RM = -215.0
            if z_LU == None:
                z_LU = -215.0

        self.timer.cancel()    
        canceler_sem = True
        self.destroy_node()

class sender_cycle_two_walk (Node):
    def __init__(self, speed):
        super().__init__('main_controller_sender_stand_cycle_two_walk')
        self.speed = speed

        stand_rm = MutuallyExclusiveCallbackGroup()
        stand_ld = MutuallyExclusiveCallbackGroup()
        stand_lu = MutuallyExclusiveCallbackGroup()
        stand_lm = MutuallyExclusiveCallbackGroup()
        stand_rd = MutuallyExclusiveCallbackGroup()
        stand_ru = MutuallyExclusiveCallbackGroup()
        sensor = MutuallyExclusiveCallbackGroup()
        
        self.publisher_LM = self.create_publisher(ThetasLegLeftMiddle, 'anglesMLLeg', 10, callback_group=stand_lm)
        self.publisher_RD = self.create_publisher(ThetasLegRightDown, 'anglesDRLeg', 10, callback_group=stand_rd)
        self.publisher_RU = self.create_publisher(ThetasLegRightUp, 'anglesURLeg', 10, callback_group=stand_ru)
        self.publisher_RM = self.create_publisher(ThetasLegRightMiddle, 'anglesMRLeg', 10, callback_group=stand_rm)
        self.publisher_LD = self.create_publisher(ThetasLegLeftDown, 'anglesDLLeg', 10, callback_group=stand_ld)
        self.publisher_LU = self.create_publisher(ThetasLegLeftup, 'anglesULLeg', 10, callback_group=stand_lu)

        self.timer = self.create_timer(0.01, self.sender_walker_one, callback_group=sensor)
     
    def sender_walker_one(self):
        global canceler_sem
        if canceler_sem == False:
            speed = self.speed
            msg_rm = ThetasLegRightMiddle()
            msg_ld = ThetasLegLeftDown()
            msg_lu = ThetasLegLeftup()
            msg_lm = ThetasLegLeftMiddle()
            msg_rd = ThetasLegRightDown()
            msg_ru = ThetasLegRightUp()
            
            sen_lm = False
            sen_rd = False
            sen_ru = False

            global sensors_data 
            global list_rm
            global list_ld
            global list_lu
            global list_lm
            global list_rd
            global list_ru

            sleeper = abs(speed) * 8
            length_rm = len(list_rm)
            length_ld = len(list_ld)
            length_lu = len(list_lu)
            length_lm = len(list_lm)
            length_rd = len(list_rd)
            length_ru = len(list_ru)
            length_max = max(length_rm,length_rd,length_ru,length_lm,length_ld,length_lu)

            global theta_val_LM
            global theta_val_RD
            global theta_val_RU
            global theta_val_RM
            global theta_val_LD
            global theta_val_LU

            global z_LM
            global z_RD
            global z_RU
            z_LM = None
            z_RD = None
            z_RU = None

            nested_list_rm = [list_rm[i:i+3] for i in range(0, len(list_rm), 3)]
            nested_list_ld = [list_ld[i:i+3] for i in range(0, len(list_ld), 3)]
            nested_list_lu = [list_lu[i:i+3] for i in range(0, len(list_lu), 3)]
            nested_list_lm = [list_lm[i:i+4] for i in range(0, len(list_lm), 4)]
            nested_list_rd = [list_rd[i:i+4] for i in range(0, len(list_rd), 4)]
            nested_list_ru = [list_ru[i:i+4] for i in range(0, len(list_ru), 4)]
            #print("nested list ru :", nested_list_ru)

            iteration = 0
            sema_sec_one = False
            sema_sec_two = False
            sema_sec_three = False
            sema_sec_four = False
            sema_sec_five = False
            sema_sec_six = False

            for ii, jj, zz, qq, ww, xx in zip_longest([[None] * 3] * 50 + nested_list_rm,[[None] * 3] * 50 + nested_list_ld,[[None] * 3] * 50 + nested_list_lu, nested_list_lm, nested_list_rd, nested_list_ru, fillvalue=[None,None,None]):   
                if sensors_data != None:
                    if sensors_data.sensor_2 >= 300 and iteration > 50 and qq[0] != None:
                        sen_lm = True
                        z_LM = qq[3]
                if qq[0] != None and sen_lm == False:
                    msg_lm.theta_1 = float(qq[0])
                    msg_lm.theta_2 = float(qq[1])
                    msg_lm.theta_3 = float(qq[2])
                    self.publisher_LM.publish(msg_lm)
                    sema_sec_four = True

                if sensors_data != None:
                    if sensors_data.sensor_6 >= 500 and iteration > 50 and ww[0] != None:
                        sen_rd = True
                        z_RD = ww[3]
                if ww[0] != None and sen_rd == False:
                    msg_rd.theta_1 = float(ww[0])
                    msg_rd.theta_2 = float(ww[1])
                    msg_rd.theta_3 = float(ww[2])
                    self.publisher_RD.publish(msg_rd)
                    sema_sec_five = True

                #if sensors_data != None:
                    #if sensors_data.sensor_4 >= 500 and iteration > 50 and xx[0] != None:
                    #    sen_ru = True
                    #    z_RU = xx[3]
                if xx[0] != None and sen_ru == False:
                    msg_ru.theta_1 = float(xx[0])
                    msg_ru.theta_2 = float(xx[1])
                    msg_ru.theta_3 = float(xx[2])
                    self.publisher_RU.publish(msg_ru)
                    sema_sec_six = True
                

                if ii[0] != None and iteration > 50:
                    #print(" i: ", ii[0])
                    msg_rm.theta_1 = float(ii[0])
                    msg_rm.theta_2 = float(ii[1])
                    msg_rm.theta_3 = float(ii[2])
                    self.publisher_RM.publish(msg_rm)
                    sema_sec_one = True

                if jj[0] != None and iteration > 50:
                    #print("j :", jj)
                    msg_ld.theta_1 = float(jj[0])
                    msg_ld.theta_2 = float(jj[1])
                    msg_ld.theta_3 = float(jj[2])
                    self.publisher_LD.publish(msg_ld)
                    sema_sec_two = True

                if zz[0] != None and iteration > 50:
                    #print("z :", zz, " iteracja :", iteration)
                    msg_lu.theta_1 = float(zz[0])
                    msg_lu.theta_2 = float(zz[1])
                    msg_lu.theta_3 = float(zz[2])
                    self.publisher_LU.publish(msg_lu)
                    sema_sec_three = True

                iteration += 1 

                #smoothing steps
                if iteration < (length_max/2):
                    iterator = 1 + iteration/5
                else:
                    iterator = -1 -(length_max - iteration)/5
                    
                smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
                time.sleep(smooth_step)

            if sema_sec_one == True:
                theta_val_RM.theta_1 = msg_rm.theta_1
                theta_val_RM.theta_2 = msg_rm.theta_2
                theta_val_RM.theta_3 = msg_rm.theta_3
            if sema_sec_two == True:
                theta_val_LD.theta_1 = msg_ld.theta_1
                theta_val_LD.theta_2 = msg_ld.theta_2
                theta_val_LD.theta_3 = msg_ld.theta_3
            if sema_sec_three == True:
                theta_val_LU.theta_1 = msg_lu.theta_1
                theta_val_LU.theta_2 = msg_lu.theta_2
                theta_val_LU.theta_3 = msg_lu.theta_3
            if sema_sec_four == True:
                theta_val_LM.theta_1 = msg_lm.theta_1
                theta_val_LM.theta_2 = msg_lm.theta_2
                theta_val_LM.theta_3 = msg_lm.theta_3
            if sema_sec_five == True:
                theta_val_RD.theta_1 = msg_rd.theta_1
                theta_val_RD.theta_2 = msg_rd.theta_2
                theta_val_RD.theta_3 = msg_rd.theta_3
            if sema_sec_six == True:
                theta_val_RU.theta_1 = msg_ru.theta_1
                theta_val_RU.theta_2 = msg_ru.theta_2
                theta_val_RU.theta_3 = msg_ru.theta_3

                #print("nested list lu :", msg_lu.theta_1, msg_lu.theta_2, msg_lu.theta_3)

            if z_LM == None:
                z_LM = -215.0
            if z_RD == None:
                z_RD = -215.0
            if z_RU == None:
                z_RU = -215.0
        
        self.timer.cancel()
        canceler_sem = True

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
        self.theta_subscriber_lm = self.create_subscription( ThetasAll, 'anglesMLLegList', self.servo_theta_lm_callback , 10, callback_group=cos_LM)

    def servo_theta_lm_callback(self, msg:ThetasAll):
        global list_lm 
        list_lm = list(msg.lista)
        
class subscriber_theta_ld(Node):
    def __init__(self):
        super().__init__('main_controller_theta_ld')
        cos_LD = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_ld = self.create_subscription(ThetasAll, 'anglesDLLegList', self.servo_theta_ld_callback , 10, callback_group=cos_LD)
    
    def servo_theta_ld_callback(self, msg: ThetasAll):
        global list_ld 
        list_ld = list(msg.lista)
        
class subscriber_theta_lu(Node):
    def __init__(self):
        super().__init__('main_controller_theta_lu')
        cos_LU = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_lu = self.create_subscription(ThetasAll, 'anglesULLegList', self.servo_theta_lu_callback , 10, callback_group=cos_LU)
        
    def servo_theta_lu_callback(self, msg: ThetasAll):
        global list_lu 
        list_lu = list(msg.lista)
   
class subscriber_theta_rm(Node):
    def __init__(self):
        super().__init__('main_controller_theta_rm')
        cos_RM = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_rm = self.create_subscription(ThetasAll, 'anglesMRLegList', self.servo_theta_rm_callback , 10, callback_group=cos_RM)
    
    def servo_theta_rm_callback(self, msg: ThetasAll):
        global list_rm
        list_rm = list(msg.lista)
    
class subscriber_theta_rd(Node):
    def __init__(self):
        super().__init__('main_controller_theta_rd')
        cos_RD = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_rd = self.create_subscription(ThetasAll, 'anglesDRLegList', self.servo_theta_rd_callback , 10, callback_group=cos_RD)
    
    def servo_theta_rd_callback(self, msg: ThetasAll):
        global list_rd
        list_rd = list(msg.lista)

class subscriber_theta_ru(Node):
    def __init__(self):
        super().__init__('main_controller_theta_ru')
        cos_RU = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_ru = self.create_subscription(ThetasAll, 'anglesURLegList', self.servo_theta_ru_callback , 10, callback_group=cos_RU)

    def servo_theta_ru_callback(self, msg: ThetasAll):
        global list_ru
        list_ru = list(msg.lista)


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

    z_LM = -207.0
    z_RD = -207.0
    z_RU = -207.0
    z_RM = -207.0
    z_LD = -207.0
    z_LU = -207.0

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
            speed = 0

        print(" speed :", speed, "\n")

        # ---------- !!!!!!!!!!! legs controll !!!!!!!!!!!!!!!!!! ------------

        if speed == 0:
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

            subsricption_sensor_one = sensors_subscription()
            sender_cycle_one = sender_first_cycle()
            
            executor_one.shutdown()

            executor_one_cycle_sub = MultiThreadedExecutor(num_threads=3)
            executor_one_cycle_sub.add_node(sender_cycle_one)
            executor_one_cycle_sub.add_node(subsricption_sensor_one)

            canceler_sem = False
            while canceler_sem != True:
                executor_one_cycle_sub.spin_once()    

            time.sleep(0.0002)
            stand_lm_loop.destroy_node()
            stand_ru_loop.destroy_node()
            stand_rd_loop.destroy_node()
            theta_sub_lm.destroy_node()
            theta_sub_rd.destroy_node()
            theta_sub_ru.destroy_node()
            executor_one_cycle_sub.shutdown()
            sender_cycle_one.destroy_node()
            subsricption_sensor_one.destroy_node()

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

            sender_cycle_two = sender_second_cycle()
            subsricption_sensor = sensors_subscription()

            executor_two_cycle_sub = MultiThreadedExecutor(num_threads=2)
            executor_two_cycle_sub.add_node(sender_cycle_two)
            executor_two_cycle_sub.add_node(subsricption_sensor)

            canceler_sem = False
            while canceler_sem != True:
                executor_two_cycle_sub.spin_once()

            time.sleep(0.0002)
            executor_two_cycle_sub.shutdown()
            sender_cycle_two.destroy_node()
            subsricption_sensor.destroy_node()
            stand_rm_loop.destroy_node()
            stand_ld_loop.destroy_node()
            stand_lu_loop.destroy_node()
            theta_sub_rm.destroy_node()
            theta_sub_ld.destroy_node()
            theta_sub_lu.destroy_node()
            print("zakonczono exec 2")

            z_LM = Z
            z_RD = Z
            z_RU = Z
            z_RM = Z
            z_LD = Z
            z_LU = Z

        
        else:
         # first cycle of tripod walking :
            # creating of threads
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

            sender_walker_one = sender_cycle_one_walk(speed)
            subsricption_sensor = sensors_subscription()

            executor_one_cycle_sub = MultiThreadedExecutor(num_threads=2)
            executor_one_cycle_sub.add_node(sender_walker_one)
            executor_one_cycle_sub.add_node(subsricption_sensor)

            canceler_sem = False
            while canceler_sem != True:
                executor_one_cycle_sub.spin_once()

            time.sleep(0.0002)
            executor_one_cycle_sub.shutdown()
            sender_walker_one.destroy_node()
            subsricption_sensor.destroy_node()
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

            sender_walker_two = sender_cycle_two_walk(speed)
            subsricption_sensor = sensors_subscription()

            executor_one_cycle_sub = MultiThreadedExecutor(num_threads=2)
            executor_one_cycle_sub.add_node(sender_walker_two)
            executor_one_cycle_sub.add_node(subsricption_sensor)

            canceler_sem = False
            while canceler_sem != True:
                executor_one_cycle_sub.spin_once()

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
            executor_one_cycle_sub.shutdown()
            sender_walker_two.destroy_node()
            subsricption_sensor.destroy_node()

            z_RM = Z
            z_LD = Z
            z_LU = Z        


        # ------------ controller events -------------

        for event in pygame.event.get():

            if event.type == JOYBUTTONDOWN:
                if event.button == 1:
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()

            if event.type == JOYAXISMOTION:

                if event.axis > 2 and event.axis < 5:
                    motion[event.axis-3] = event.value

            if event.type == JOYDEVICEADDED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

            if event.type == JOYDEVICEREMOVED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

            if event.type == QUIT:
                 pygame.quit()
                 rclpy.shutdown()
                 sys.exit()

            if event.type == KEYDOWN:

                if event.key == K_ESCAPE:
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()

        clock.tick(60)

if __name__ == '__main__':
    main()