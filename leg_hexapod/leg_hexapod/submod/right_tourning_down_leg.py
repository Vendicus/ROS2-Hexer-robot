
import numpy as np

def tourning(tourning_value, speed):
    def regulate(table):
        length = len(table)
        if length == 0:
                print("error, this data tourining list is empty")
        else:  
                middle_index = length // 2

                if length % 2 == 1:
                        differ = 85.0 - table[middle_index] 
                        for i in range(length):
                              table[i] = table[i] + differ
                        return table
                else:
                        differ = 85.0 - ((table[middle_index] + table[middle_index - 1]) / 2) 
                        for i in range(length):
                              table[i] = table[i] + differ
                        return table
                
    np.set_printoptions(precision=1)

    point1 = np.array([-246.922,  30.0])
    point2 = np.array([-246.922, -30.0])

    if tourning_value >= 0.0:

        r1 = tourning_value  + 201.0
        r2 = r1 - 402.0
        r3 = r1 - 75.5
        r4 = r1 - 306.5
    
        stabilisation = 45.0/(r1/200.0)
        stabilisation1 = 45.0/(r4/200.0)   
    
        resolution = np.float16(1.0/(r1/250.0))

        if ( r2 < 0.0 ) and ( r2 > -50.0 ) :
                r4 = np.sqrt( r2**2 + 95.5**2)
                stabilisation1 = 45.0/(r4/135.0)
        elif (r2 <= -50.0) and (r2 > -150.0 ) :
                r4 = - np.sqrt( r2**2 + 95.5**2) 
                stabilisation1 = 45.0/(r4/200.0)
                point1 = np.array([-246.922,  31.0])
                point2 = np.array([-246.922, -31.0])
        elif (r2 <= -150.0) and (r2 >= -201.0) :
                r4 = r2 + tourning_value/2
                stabilisation1 = 45.0/(r4/200.0)
                point1 = np.array([-246.922,  31.0])
                point2 = np.array([-246.922, -31.0])
    
        if r3 < 220 :
           r3 = r1 - (tourning_value/2)
    
    else :

        r2 = tourning_value - 201.0
        r1 = r2 + 402.0
        r3 = r2 + 306.5
        r4 = r2 + 75.5
    
        stabilisation = 40.0/(r2/200.0)
        stabilisation1 = 40.0/(r3/200.0)   
    
        resolution = np.float16(1.0/(r2/250.0))

        if ( r1 > 0.0 ) and ( r1 < 50.0 ) :
                r3 = - np.sqrt( r1**2 + 95.5**2)
                stabilisation1 = 45.0/(r3/135.0)
        elif (r1 >= 50.0) and (r1 < 150.0 ) :
                r3 = np.sqrt( r1**2 + 95.5**2) 
                stabilisation1 = 45.0/(r3/200.0)
                point1 = np.array([-246.922,  31.0])
                point2 = np.array([-246.922, -31.0])
        elif (r1 >= 150.0) and (r1 <= 201.0) :
                r3 = r1 - tourning_value/2
                stabilisation1 = 45.0/(r3/200.0)
                point1 = np.array([-246.922,  31.0])
                point2 = np.array([-246.922, -31.0])
    
        if r4 > - 220 :
           r4 = r2 - (tourning_value/2)

    prosta = np.linalg.norm(point2 - point1)

    theta = 2 * np.degrees(np.arcsin(prosta / (2*r1 if tourning_value >= 0.0 else 2*r2)))
    theta_plus = (theta/2) + 180
    theta_minus = -(theta/2) + 180
    theta_90 = theta/90

    y5_tourning = []
    x5_tourning = []

    if tourning_value >= 0.0 :
        angle3 = np.arange( theta_minus + stabilisation1, theta_plus + stabilisation1 + theta_90, resolution)
        if speed >= 0:
                for i in range(0, angle3.size) :
                        y5_tourning.append(np.float16( r4 * np.cos(np.deg2rad(angle3[i])) + tourning_value ) ) 
                        x5_tourning.append(np.float16( r4 * np.sin(np.deg2rad(angle3[i])) ))
        else:
                for i in reversed( range(0, angle3.size) ):
                        y5_tourning.append(np.float16( r4 * np.cos(np.deg2rad(angle3[i])) + tourning_value ) ) 
                        x5_tourning.append(np.float16( r4 * np.sin(np.deg2rad(angle3[i])) ))

    else :
        angle3 = np.arange( theta_minus - stabilisation, theta_plus - stabilisation + theta_90, resolution)
        if speed >= 0:
                for i in reversed( range(0, angle3.size) ):
                        y5_tourning.append(np.float16( r4 * np.cos(np.deg2rad(angle3[i])) + tourning_value ) )  
                        x5_tourning.append(np.float16( -r4 * np.sin(np.deg2rad(angle3[i])) )) 
        else:
                for i in range(0, angle3.size):
                        y5_tourning.append(np.float16( r4 * np.cos(np.deg2rad(angle3[i])) + tourning_value ) )  
                        x5_tourning.append(np.float16( -r4 * np.sin(np.deg2rad(angle3[i])) ))
  
    r_y5_tourning = regulate(y5_tourning)

    for i in range(len(r_y5_tourning)):
        yield x5_tourning[i], r_y5_tourning[i]
 
