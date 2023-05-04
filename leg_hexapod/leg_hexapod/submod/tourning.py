
import numpy as np

def tourning(tourning_value):
    np.set_printoptions(precision=3)

    point1 = np.array([-246.922,  41.0])
    point2 = np.array([-246.922, -41.0])

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
    
        stabilisation = 45.0/(r2/200.0)
        stabilisation1 = 45.0/(r3/200.0)   
    
        resolution = np.float16(1.0/(r2/250.0))

        if ( r1 > 0.0 ) and ( r1 < 50.0 ) :
                r3 = - np.sqrt( r1**2 + 95.5**2)
                stabilisation1 = 45.0/(r3/135.0)
        elif (r1 >= 50.0) and (r1 < 150.0 ) :
                r3 = np.sqrt( r1**2 + 95.5**2) 
                stabilisation1 = 45.0/(r3/200.0)
                point1 = np.array[-246.922,  31.0]
                point2 = np.array[-246.922, -31.0]
        elif (r1 >= 150.0) and (r1 <= 201.0) :
                r3 = r1 - tourning_value/2
                stabilisation1 = 45.0/(r3/200.0)
                point1 = np.array[-246.922,  31.0]
                point2 = np.array[-246.922, -31.0]
    
        if r4 > - 220 :
           r4 = r2 - (tourning_value/2)

    prosta = np.linalg.norm(point2 - point1)

    theta = 2 * np.degrees(np.arcsin(prosta / (2*r1 if tourning_value >= 0.0 else 2*r2)))
    theta_plus = (theta/2) + 180
    theta_minus = -(theta/2) + 180
    theta_90 = theta/90

    angle = np.arange( theta_minus, theta_plus + theta_90, resolution )

    if tourning_value >= 0.0 :
        angle1 = np.arange( theta_minus + stabilisation, theta_plus + stabilisation + theta_90, resolution)
        angle2 = np.arange( theta_minus - stabilisation, theta_plus - stabilisation + theta_90, resolution)
        angle3 = np.arange( theta_minus + stabilisation1, theta_plus + stabilisation1 + theta_90, resolution)
        angle4 = np.arange( theta_minus - stabilisation1, theta_plus - stabilisation1 + theta_90, resolution)
    else :
        angle1 = np.arange( theta_minus - stabilisation1, theta_plus - stabilisation1 + theta_90, resolution)
        angle2 = np.arange( theta_minus + stabilisation1, theta_plus + stabilisation1 + theta_90, resolution)
        angle3 = np.arange( theta_minus - stabilisation, theta_plus - stabilisation + theta_90, resolution)
        angle4 = np.arange( theta_minus + stabilisation, theta_plus + stabilisation + theta_90, resolution) 


    for i in range(0, angle.size) :

        y1_tourning = np.float16( -( r1*np.cos(np.deg2rad(angle[i])) + tourning_value + 80) )
        x1_tourning = np.float16( r1 * np.sin(np.deg2rad(angle[i])) )

        y2_tourning = np.float16( r2 * np.cos(np.deg2rad(angle[i])) + tourning_value - 80 )
        x2_tourning = np.float16( - r2 * np.sin(np.deg2rad(angle[i])) ) 

        y3_tourning = np.float16( - ( r3 * np.cos(np.deg2rad(angle1[i])) + tourning_value + 40) )
        x3_tourning = np.float16( r3 * np.sin(np.deg2rad(angle1[i])) ) 

        y4_tourning = np.float16( - ( r3 * np.cos(np.deg2rad(angle2[i])) + tourning_value + 40) )
        x4_tourning = np.float16( r3 * np.sin(np.deg2rad(angle2[i])) )
    
        y5_tourning = np.float16( r4 * np.cos(np.deg2rad(angle3[i])) + tourning_value - 40 )
        x5_tourning = np.float16( - r4 * np.sin(np.deg2rad(angle3[i])) ) 

        y6_tourning = np.float16( r4 * np.cos(np.deg2rad(angle4[i])) + tourning_value - 40 ) 
        x6_tourning = np.float16( - r4 * np.sin(np.deg2rad(angle4[i])))

        yield x1_tourning, y1_tourning, x2_tourning, y2_tourning, x3_tourning, y3_tourning, x4_tourning, y4_tourning, x5_tourning, y5_tourning, x6_tourning, y6_tourning

