import time 
import numpy as np
from .bezier import bezier_curve as bc
from .inverse_kinematics_optimalisation import inverse_kinematics as ik

import rclpy
from rclpy.node import Node
from hexapod_message.msg import ThetasLegLeftDown, ThetasLegLeftMiddle, ThetasLegLeftup, ThetasLegRightDown, ThetasLegRightMiddle, ThetasLegRightUp, Sensor

def walking( touch_sensor, tourning_var, tourn, start_angle_x, start_angle_y, start_angle_z, speed, publisher, Z):
    iterator = iter(tourn(tourning_var))
    for item in iterator:
        X = item[0] 
        Y = item[1]
        #print(" tourning values : ",item,"\n")
    
    Start = np.array([X, Y, Z])

    for i in ik(Start[0], Start[1], Start[2], start_angle_x - 90.0, start_angle_y - 90.0, -start_angle_z):
        convert = [float(konv) for konv in i]
        theta_1 = convert[0] 
        theta_2 = convert[1] 
        theta_3 = convert[2] 
        publisher(theta_1, theta_2, theta_3)
        time.sleep(speed) 
        #print(" all thetas: " ,theta_1, theta_2, theta_3)

    iteration = 0

    for i in tourn(tourning_var) :
        #print (" tourning values :",i[0],i[1])
        if iteration == 0:
            for iter, j in enumerate(bc(Start[0], Start[1], Start[2], i[0], i[1], Z)):
                if touch_sensor == True and iter > 3: 
                    Z = z_pos   
                    break
                    
                for z in ik(j[0], j[1], j[2], theta_1 -90.0, theta_2 -90.0, -theta_3 ):
                    if touch_sensor == True and iter > 3:
                        Z = z_pos
                        break

                    convert = [float(konv) for konv in z]
                    theta_1 = convert[0] 
                    theta_2 = convert[1] 
                    theta_3 = convert[2]
                    z_pos = convert[3] 
                    publisher(theta_1, theta_2, theta_3)
                    time.sleep(speed)

                end_x = j[0]
                end_y = j[1]
                #print(" all thetas: " ,theta_1, theta_2, theta_3)

            if touch_sensor == False:
                Z = 230.0
                for z in ik(end_x, end_y, Z, theta_1 -90.0, theta_2 -90.0, -theta_3 ):

                    if touch_sensor == True:
                        Z = z_pos 
                        break

                    convert = [float(konv) for konv in z]
                    theta_1 = convert[0] 
                    theta_2 = convert[1] 
                    theta_3 = convert[2]
                    z_pos = convert[3]
                    publisher(theta_1, theta_2, theta_3)
                    time.sleep(speed) 
                    #print(" all thetas: " ,theta_1, theta_2, theta_3)
                
        else :
            for j in ik( i[0], i[1], Z, theta_1 - 90, theta_2 -90, -theta_3):
                convert = [float(konv) for konv in j]
                theta_1 = convert[0] 
                theta_2 = convert[1] 
                theta_3 = convert[2] 
                publisher(theta_1, theta_2, theta_3)
                time.sleep(speed) 
                #print(" all thetas: " ,theta_1, theta_2, theta_3)               
        #print(" iteration number : ", iteration, "\n \n")
        iteration += 1   
    
    time.sleep(0.25)
    return {
        theta_1,
        theta_2,
        theta_3
    }
    