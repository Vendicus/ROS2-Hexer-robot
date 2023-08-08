
import numpy as np 

def inverse_kinematics(pos_end_x, pos_end_y, pos_end_z, theta_1, theta_2, theta_3):

    """
    inverse kinematics function created by Michal Aksamit, all rights reserved. 
    The method for computation use Jacobian matrix with least squares to decrease possibility of singularities. 
    For each loop one set of values from differentials of angles are returned, so it is possible to use it with close loop system, 
    like sensors to break when wanted value appears. Because function uses yield, we need to use it with for loop to work effectively,
    for example: for i in ik(pos_end_x, pos_end_y, pos_end_z, angle_start_x, angle_start_y, angle_start_z): 
                     print("read value by servo :", i[0], i[1], i[2])   

    :param pos_end_x: position of effector after computing inverse kinematics on x axis. 
    :param pos_end_y: position of effector after computing inverse kinematics on y axis. 
    :param pos_end_z: position of effector after computing inverse kinematics on z axis. 
    :param theta_1: angle before starting computation of joint moving across x axis.
    :param theta_2: angle before starting computation of joint moving across y axis.
    :param theta_3: angle before starting computation of joint moving across z axis.

    :yield: angles after computation for servo read with x, y, z axis for each servomotor.
    """
    np.set_printoptions(precision=3)

    # dimensions lengths of robot leg (mm)
    coxa = 51.0
    femur = 70.0
    tibia = 207.0

    # angle in iteration during time
    actual_angle = np.array([theta_1, theta_2, theta_3])

    # set weight for least squares
    weight = np.array([1.0, 2.0, 3.0])

    for _ in range(1200):
         
        # trigonometry used in process
        cos_0 = np.cos(np.deg2rad(actual_angle[0]))
        sin_0 = np.sin(np.deg2rad(actual_angle[0]))
        cos_1 = np.cos(np.deg2rad(actual_angle[1]))
        sin_1 = np.sin(np.deg2rad(actual_angle[1]))
        cos_2 = np.cos(np.deg2rad(actual_angle[2]))
        sin_2 = np.sin(np.deg2rad(actual_angle[2]))

        # forward kinematics for efector (point A)
        x_pos = tibia*(sin_0*sin_1*sin_2 - cos_1*cos_2*sin_0) - coxa*sin_0 -femur*cos_1*sin_0
        y_pos = coxa*cos_0 - tibia*(cos_0*sin_1*sin_2 - cos_0*cos_1*cos_2) +femur*cos_0*cos_1
        z_pos = tibia*(cos_1*sin_2 + cos_2*sin_1) + femur*sin_1

        # criteria to break inverse kinematic loop
        error = np.array([pos_end_x - x_pos, pos_end_y - y_pos, pos_end_z - z_pos])
        if np.linalg.norm(error) < 2:
            break

        # regularisation building
        R = np.array([actual_angle[0]**2, actual_angle[1]**2, actual_angle[2]**2])

        # creating jacobian for analitycal computation
        J_num = np.array([
        [tibia*(cos_0*sin_1*sin_2 - cos_0*cos_1*cos_2) - coxa*cos_0 - femur*cos_0*cos_1, tibia*(cos_1*sin_0*sin_2 + sin_1*cos_2*sin_0) + femur*sin_0*sin_1, tibia*(cos_1*sin_0*sin_2 + cos_2*sin_0*sin_1)],
        [tibia*(sin_0*sin_1*sin_2 - cos_1*cos_2*sin_0) - coxa*sin_0 - femur*cos_1*sin_0, -tibia*(cos_0*cos_1*sin_2 + cos_0*cos_2*sin_1) - femur*cos_0*sin_1, -tibia*(cos_0*cos_1*sin_2 + cos_0*cos_2*sin_1)],
        [1.0, tibia*(cos_1*cos_2 - sin_1*sin_2) + femur*cos_1, tibia*(cos_1*cos_2 - sin_1*sin_2)]
        ])

        # creating special matrix to use later, copying jacobian into this matrix
        A = J_num.copy()

        # --------- the least squares method, first part -----------
        J_num = np.dot(A.T, A * weight.reshape(-1, 1))

        #  regularisation for diagonal
        np.fill_diagonal(J_num, np.diag(J_num) + R)

        # inverse matrix of jacobian
        J_inv = np.linalg.inv(J_num)

        # ---------- the least squares method, second part -------------
        J_num = np.dot(J_inv, A.T * weight.reshape(-1, 1))

        # counting angle to target by differentials
        diff_angle = J_num.dot(error)
         
        # update actual angle
        if _ == 0:
            prev_angle = actual_angle.copy()
        actual_angle[:] = actual_angle + diff_angle
        
        # smoothing data send to servo by reading diffrence between degs, if your servo is precise you could use to send when diff is 0.3, if less precise use higher diff value.
        if np.linalg.norm(np.array([actual_angle[0] - prev_angle[0],actual_angle[1] - prev_angle[1], actual_angle[2] - prev_angle[2]])) > 1:
            # converting to values ready to read by servo motors
            prev_angle = actual_angle.copy()
            serwo_theta_1 = np.float16( actual_angle[0] + 90) 
            serwo_theta_2 = np.float16( actual_angle[1] + 90)
            serwo_theta_3 = np.float16( -actual_angle[2] )

            yield serwo_theta_1, serwo_theta_2, serwo_theta_3, np.float16(z_pos)


#for i in inverse_kinematics( -41.0, 113.0, -207.0, -20.0, 0.0, -90.0):
#    print (" thetas :",i[0], i[1], i[2])