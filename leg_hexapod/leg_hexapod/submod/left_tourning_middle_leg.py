
import numpy as np

def tourning(tourning_value):
    np.set_printoptions(precision=1)

    point1 = np.array([-246.922,  41.0])
    point2 = np.array([-246.922, -41.0])

    if tourning_value >= 0.0:

        r1 = tourning_value  + 201.0
        r2 = r1 - 402.0
        r3 = r1 - 75.5
        r4 = r1 - 306.5 
    
        resolution = np.float16(1.0/(r1/250.0))

        if ( r2 < 0.0 ) and ( r2 > -50.0 ) :
                r4 = np.sqrt( r2**2 + 95.5**2)
        elif (r2 <= -50.0) and (r2 > -150.0 ) :
                r4 = - np.sqrt( r2**2 + 95.5**2) 
                point1 = np.array([-246.922,  31.0])
                point2 = np.array([-246.922, -31.0])
        elif (r2 <= -150.0) and (r2 >= -201.0) :
                r4 = r2 + tourning_value/2
                point1 = np.array([-246.922,  31.0])
                point2 = np.array([-246.922, -31.0])
    
        if r3 < 220 :
           r3 = r1 - (tourning_value/2)
    
    else :

        r2 = tourning_value - 201.0
        r1 = r2 + 402.0
        r3 = r2 + 306.5
        r4 = r2 + 75.5 
    
        resolution = np.float16(1.0/(r2/250.0))

        if ( r1 > 0.0 ) and ( r1 < 50.0 ) :
                r3 = - np.sqrt( r1**2 + 95.5**2)
        elif (r1 >= 50.0) and (r1 < 150.0 ) :
                r3 = np.sqrt( r1**2 + 95.5**2) 
                point1 = np.array[-246.922,  31.0]
                point2 = np.array[-246.922, -31.0]
        elif (r1 >= 150.0) and (r1 <= 201.0) :
                r3 = r1 - tourning_value/2
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

    for i in range(0, angle.size) :

        y1_tourning = np.float16( -( r1*np.cos(np.deg2rad(angle[i])) + tourning_value + 80) )
        x1_tourning = np.float16( r1 * np.sin(np.deg2rad(angle[i])) )

        yield x1_tourning, y1_tourning

