import numpy as np

def forward_kine(start_angle_x, start_angle_y, start_angle_z):
        np.set_printoptions(precision=3)

        # dimensions lengths of robot leg (mm)
        coxa = 51.0
        femur = 70.0
        tibia = 207.0

        # trigonometry used in process
        cos_0 = np.cos(np.deg2rad(start_angle_x))
        sin_0 = np.sin(np.deg2rad(start_angle_x))
        cos_1 = np.cos(np.deg2rad(start_angle_y))
        sin_1 = np.sin(np.deg2rad(start_angle_y))
        cos_2 = np.cos(np.deg2rad(start_angle_z))
        sin_2 = np.sin(np.deg2rad(start_angle_z))

        # forward kinematics for efector (point A)
        x_pos = tibia*(sin_0*sin_1*sin_2 - cos_1*cos_2*sin_0) - coxa*sin_0 -femur*cos_1*sin_0
        y_pos = coxa*cos_0 - tibia*(cos_0*sin_1*sin_2 - cos_0*cos_1*cos_2) +femur*cos_0*cos_1
        z_pos = tibia*(cos_1*sin_2 + cos_2*sin_1) + femur*sin_1  

        return x_pos, y_pos, z_pos  
