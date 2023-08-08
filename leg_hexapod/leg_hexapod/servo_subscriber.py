import rclpy
from rclpy.node import Node

from adafruit_servokit import ServoKit
kit_left = ServoKit(channels=16, address = 0x40)
kit_right = ServoKit(channels=16, address = 0x41)

for i in range(16):
    kit_left.servo[i].actuation_range = 165
    kit_right.servo[i].actuation_range = 165

from hexapod_message.msg import ThetasLegLeftMiddle, ThetasLegRightMiddle, ThetasLegRightUp, ThetasLegRightDown, ThetasLegLeftup, ThetasLegLeftDown

class Angle_subscriber(Node):

    def __init__(self):
        super().__init__('theta_subscriber')

        # subsribe topic with all angles to middle right leg 
        self.subscription_MR = self.create_subscription(
            ThetasLegRightMiddle,
            'anglesMRLeg',
            self.listener_callback_MR,
            3)
        self.subscription_MR  # prevent unused variable warning

        # subsribe topic with all angles to middle left leg 
        self.subscription_ML = self.create_subscription(
            ThetasLegLeftMiddle,
            'anglesMLLeg',
            self.listener_callback_ML,
            3)
        self.subscription_ML  # prevent unused variable warning

        # subsribe topic with all angles to middle right leg 
        self.subscription_UR = self.create_subscription(
            ThetasLegRightUp,
            'anglesURLeg',
            self.listener_callback_UR,
            3)
        self.subscription_UR  # prevent unused variable warning

        # subsribe topic with all angles to up right leg 
        self.subscription_UL = self.create_subscription(
            ThetasLegLeftup,
            'anglesULLeg',
            self.listener_callback_UL,
            3)
        self.subscription_UL  # prevent unused variable warning

        self.subscription_DR = self.create_subscription(
            ThetasLegRightDown,
            'anglesDRLeg',
            self.listener_callback_DR,
            3)
        self.subscription_DR  # prevent unused variable warning

        self.subscription_DL = self.create_subscription(
            ThetasLegLeftDown,
            'anglesDLLeg',
            self.listener_callback_DL,
            3)
        self.subscription_DL  # prevent unused variable warning


    def listener_callback_MR(self, msg_rm: ThetasLegRightMiddle):
        kit_right.servo[6].angle = msg_rm.theta_1 + 24.0
        kit_right.servo[7].angle = msg_rm.theta_2 + 2.0
        kit_right.servo[8].angle = msg_rm.theta_3 - 4.0
        
    def listener_callback_ML(self, msg_lm: ThetasLegLeftMiddle):
        kit_left.servo[6].angle = msg_lm.theta_1 - 19.0
        kit_left.servo[7].angle = 180.0 - msg_lm.theta_2 + 4.0
        kit_left.servo[8].angle = 180.0 - msg_lm.theta_3

    def listener_callback_UR(self, msg_ru: ThetasLegRightUp):
        kit_right.servo[13].angle = 180.0 - msg_ru.theta_1
        kit_right.servo[14].angle = msg_ru.theta_2 - 9.0
        kit_right.servo[15].angle = msg_ru.theta_3 - 3.0   

    def listener_callback_UL(self, msg_lu: ThetasLegLeftup):
        kit_left.servo[0].angle = msg_lu.theta_1 - 10.0
        kit_left.servo[1].angle = 180.0 - msg_lu.theta_2 + 7.0
        kit_left.servo[2].angle = 180.0 - msg_lu.theta_3 + 2.0

    def listener_callback_DR(self, msg_rd: ThetasLegRightDown):
        kit_right.servo[0].angle = msg_rd.theta_1 - 25.0
        kit_right.servo[1].angle = msg_rd.theta_2 - 6.0
        kit_right.servo[2].angle = msg_rd.theta_3 - 6.0   
    
    def listener_callback_DL(self, msg_ld: ThetasLegLeftDown):
        kit_left.servo[13].angle = msg_ld.theta_1 - 15.0
        kit_left.servo[14].angle = 180.0 - msg_ld.theta_2 + 3.0
        kit_left.servo[15].angle = 180.0 - msg_ld.theta_3 + 2.0  

def main(args=None):

    rclpy.init(args=args)

    subscriber_for_servos = Angle_subscriber()

    rclpy.spin(subscriber_for_servos)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber_for_servos.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
     main()