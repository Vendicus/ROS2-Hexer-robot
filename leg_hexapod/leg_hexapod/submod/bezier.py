
import numpy as np

def bezier_curve(Start_x, Start_y, Start_z, End_x, End_y, End_z):
    np.set_printoptions(precision=3)

    # number of points (with border and floating points) in curve
    n = 7

    # startung values for counting on first loop in Bezier theorem
    n_factorial = 1.0
    i_factorial = 1.0
    differ_factorial = 1.0

    # creating arrays from input data
    Start = np.array([Start_x, Start_y, Start_z])
    End = np.array([End_x, End_y, End_z])

    # counting atan for 3D direction of curve
    a = np.arctan((Start[1] - End[1])/( Start[0] - End[0])) 
    # counting cos and sin for points coordinates
    cos_a = np.cos(a)
    sin_a = np.sin(a) 
    
    # counting arrays for floating points
    P_1 = np.array([ Start[0] - (20* cos_a), Start[1] - (26* sin_a), Start[2]])
    P_2 = np.array([ Start[0] - (26* cos_a), Start[1] - (26* sin_a), Start[2] + 20])
    P_3 = np.array([ (Start[0] + End[0])/2, (Start[1] + End[1])/2, ((Start[2] + End[2])/2)+100])
    P_4 = np.array([ End[0] + (20* cos_a), End[1] + (26* sin_a), End[2] + 20])
    P_5 = np.array([ End[0] + (26* cos_a), End[1] + (26* sin_a), End[2]])

    # arrays for points x, y, z in one list 
    Point_x = np.array([Start[0], P_1[0], P_2[0], P_3[0], P_4[0], P_5[0], End[0]])
    Point_y = np.array([Start[1], P_1[1], P_2[1], P_3[1], P_4[1], P_5[1], End[1]])
    Point_z = np.array([Start[2], P_1[2], P_2[2], P_3[2], P_4[2], P_5[2], End[2]])

    # counting first main factorial
    for i in range(1, n):
        n_factorial *= i
    
    # table for all t parametr value in range 0 to 1, with resolution 0.05
    t_table = np.arange(0,1.05,0.05)

    # calculating bezier 
    for t in range(0,t_table.size):
       

        # reanalizing bezier
        Bezier_curve_x = 0.0 
        Bezier_curve_y = 0.0 
        Bezier_curve_z = 0.0 

        for i in range(n):
            for j in range(i+1): 
                i_factorial *= j                
                if i_factorial == 0:
                   i_factorial = 1
                  
            for k in range(n-i):
                differ_factorial *= k
               
                if differ_factorial == 0:
                   differ_factorial = 1

            binomial_theorem = (n_factorial/ (i_factorial * differ_factorial)) * ((1-(t_table[t]))**(n-i-1)) * ((t_table[t])**i)

            Bezier_curve_x += binomial_theorem * Point_x[i]
            Bezier_curve_y += binomial_theorem * Point_y[i]
            Bezier_curve_z += binomial_theorem * Point_z[i]
            
        # return on one iteration
        yield Bezier_curve_x, Bezier_curve_y, Bezier_curve_z 
