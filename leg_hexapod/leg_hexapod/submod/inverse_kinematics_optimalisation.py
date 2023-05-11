
import numpy as np 

def inverse_kinematics(pos_end_x, pos_end_y, pos_end_z, theta_1, theta_2, theta_3):

    np.set_printoptions(precision=3)

    # dimensions lengths of robot leg (mm)
    coxa = 51
    femur = 70
    tibia = 160

    # angle in iteration during time
    actual_angle = np.array([theta_1, theta_2, theta_3])

    # set weight for least squares
    weight = np.array([1.0, 2.0, 3.0])

    # set bound for servos reasons
    angleupperbound = np.array([80.0, 80.0, -20.0])
    anglelowerbound = np.array([-80.0, -80.0, -180.0])


    for _ in range(1000):
         # trigonometry used in process
         cos_0 = np.cos(np.deg2rad(actual_angle[0]))
         sin_0 = np.sin(np.deg2rad(actual_angle[0]))
         cos_1 = np.cos(np.deg2rad(actual_angle[1]))
         sin_1 = np.sin(np.deg2rad(actual_angle[1]))
         cos_2 = np.cos(np.deg2rad(actual_angle[2]))
         sin_2 = np.sin(np.deg2rad(actual_angle[2]))

         # forward kinematics for efector (point A)
         x_pos = tibia*(sin_0 * sin_1 * sin_2 - cos_1 * cos_2 * sin_0) - coxa*sin_0 - femur*cos_1*sin_0
         y_pos = coxa*cos_0 - tibia*(cos_1*sin_1*sin_2 - cos_0*cos_1*cos_2) + femur*cos_0*cos_1
         z_pos = tibia*(cos_1*sin_2 + cos_2*sin_1) + femur*sin_1

         # criteria to break inverse kinematic loop
         error = np.array([pos_end_x - x_pos, pos_end_y - y_pos, pos_end_z - z_pos])
         if np.linalg.norm(error) < 3:
            break

         # regularisation building
         R = np.array([actual_angle[0]**2, actual_angle[1]**2, actual_angle[2]**2])

         # creating jacobian for analitycal computation
         J_num = np.array([
         [tibia*(cos_0*sin_1*sin_2 - cos_0*cos_1*cos_2) - coxa*cos_0 - femur*cos_0*cos_1, tibia*(cos_1*sin_0*sin_2 - cos_1*cos_2*sin_0) - coxa*sin_0 - femur*cos_1*sin_0, 1.0],
         [tibia*(sin_0*sin_1*sin_2 - cos_1*cos_2*sin_0) - coxa*cos_0*sin_0 - femur*cos_0*cos_1*sin_0, -tibia*(cos_0*cos_1*sin_2 + cos_0*cos_2*sin_1) + femur*cos_0*sin_1, tibia*(cos_1*cos_2 - sin_1*sin_2) + femur*cos_1],
         [tibia*(cos_1*sin_0*sin_2 + cos_2*sin_0*sin_1), -tibia*(cos_0*cos_1*sin_2 + cos_0*cos_2*sin_1), tibia*(cos_1*cos_2 - sin_1*sin_2)]
         ])

         # creating special matrix for use later, copying jacobian into this matrix
         A = J_num.copy()

         # --------- the least squares method, first part -----------
         J_num = np.dot(A.T, A * weight.reshape(-1, 1))

         #  regularisation for diagonal
         np.fill_diagonal(J_num, np.diag(J_num) + R)

         # inverse matrix of jacobian
         J_inv = np.linalg.inv(J_num)

         # ---------- the least squares method, second part -------------
         J_num = np.dot(J_inv, A.T * weight.reshape(-1, 1))

         # vektor of angle differential
         diff_angle = np.zeros(3)

         # counting angle to target by differentials
         np.dot(J_num, error, out=diff_angle)
         
         # adding differential angel
         next_angle = np.zeros(3)

         # boundaries and angles converting for servos
         next_angle = np.clip(actual_angle + diff_angle, anglelowerbound, angleupperbound)

         # update actual angle
         actual_angle[:] = next_angle

         # converting to values ready to read by servo motors
         serwo_theta_1 = np.float16( next_angle[0] + 90.0) 
         serwo_theta_2 = np.float16( next_angle[1] + 90.0)
         serwo_theta_3 = np.float16( next_angle[2] + 180.0)
    
         yield serwo_theta_1, serwo_theta_2, serwo_theta_3
