import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008


# Hardware SPI configuration:
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# servokit install left and right servo controller
from adafruit_servokit import ServoKit
kit_left = ServoKit(channels=16, address = 0x40)
kit_right = ServoKit(channels=16, address = 0x41)

# limits for servos
for i in range(16):
    kit_left.servo[i].actuation_range = 165
    kit_right.servo[i].actuation_range = 165

from hexapod_message.msg import Sensor, ThetasLegLeftMiddle, ThetasLegRightMiddle, ThetasLegRightUp, ThetasLegRightDown, ThetasLegLeftup, ThetasLegLeftDown

class Angle_subscriber(Node):

    def __init__(self):
        super().__init__('theta_subscriber')
        self.msg = None

        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        # create publisher for all 6 sensors of all legs
        self.publisher_sensor = self.create_publisher(Sensor, 'Sensor', 2, callback_group=client_cb_group)
        self.timer = self.create_timer(0.1, self.publish_sensor, callback_group=timer_cb_group)

        # subsribe topic with all angles to middle right leg 
        self.subscription_MR = self.create_subscription(
            ThetasLegRightMiddle,
            'anglesMRLeg',
            self.listener_callback_MR,
            10 )
        self.subscription_MR  # prevent unused variable warning

        # subsribe topic with all angles to middle left leg 
        self.subscription_ML = self.create_subscription(
            ThetasLegLeftMiddle,
            'anglesMLLeg',
            self.listener_callback_ML,
            10 )
        self.subscription_ML  # prevent unused variable warning

        # subsribe topic with all angles to middle right leg 
        self.subscription_UR = self.create_subscription(
            ThetasLegRightUp,
            'anglesURLeg',
            self.listener_callback_UR,
            10 )
        self.subscription_UR  # prevent unused variable warning

        # subsribe topic with all angles to up right leg 
        self.subscription_UL = self.create_subscription(
            ThetasLegLeftup,
            'anglesULLeg',
            self.listener_callback_UL,
            10 )
        self.subscription_UL  # prevent unused variable warning

        self.subscription_DR = self.create_subscription(
            ThetasLegRightDown,
            'anglesDRLeg',
            self.listener_callback_DR,
            10)
        self.subscription_DR  # prevent unused variable warning

        self.subscription_DL = self.create_subscription(
            ThetasLegLeftDown,
            'anglesDLLeg',
            self.listener_callback_DL,
            10)
        self.subscription_DL  # prevent unused variable warning



    def publish_sensor(self):
        msg = Sensor()
        # The read_adc function will get the value of the specified channel (0-7).
        msg.sensor_1 = float(mcp.read_adc(0))
        msg.sensor_2 = float(mcp.read_adc(1))
        msg.sensor_3 = float(mcp.read_adc(2))
        msg.sensor_4 = float(mcp.read_adc(3))
        msg.sensor_5 = float(mcp.read_adc(6))
        msg.sensor_6 = float(mcp.read_adc(7))
        self.publisher_sensor.publish(msg)
         

    def listener_callback_MR(self, msg_rm: ThetasLegRightMiddle):
        if msg_rm.theta_1 + 24.0 > 165.0:
                print("Warning: Maximum theta bound approached in RM theta 1, going for 165.0, last value in msg : ", msg_rm)
                msg_rm.theta_1 = 165.0
        elif msg_rm.theta_1 + 24.0 < 0:
                print("Warning: Minimum theta bound approached in RM theta 1, going for 0.0, last value in msg : ", msg_rm)
                msg_rm.theta_1 = 0.0
        
        if msg_rm.theta_2 + 2.0 > 165.0:
                print("Warning: Maximum theta bound approached in RM theta 2, going for 165.0, last value in msg : ", msg_rm)
                msg_rm.theta_2 = 165.0
        elif msg_rm.theta_2 + 2.0 < 0:
                print("Warning: Minimum theta bound approached in RM theta 2, going for 0.0, last value in msg : ", msg_rm)
                msg_rm.theta_2 = 0.0
        
        if msg_rm.theta_3 + 2.0 > 165.0:
                print("Warning: Maximum theta bound approached in RM theta 3, going for 165.0, last value in msg : ", msg_rm)
                msg_rm.theta_3 = 165.0
        elif msg_rm.theta_3 + 2.0 < 0:
                print("Warning: Minimum theta bound approached in RM theta 3, going for 0.0, last value in msg : ", msg_rm)
                msg_rm.theta_3 = 0.0

        kit_right.servo[6].angle = msg_rm.theta_1 - 22.0
        kit_right.servo[7].angle = msg_rm.theta_2 + 2.0
        kit_right.servo[8].angle = msg_rm.theta_3 - 4.0
        
    def listener_callback_ML(self, msg_lm: ThetasLegLeftMiddle):
        if msg_lm.theta_1 - 19.0 > 165.0:
                print("Warning: Maximum theta bound approached in LM theta 1, going for 165.0, last value in msg : ", msg_lm)
                msg_lm.theta_1 = 165.0
        elif msg_lm.theta_1 - 19.0 < 0:
                print("Warning: Minimum theta bound approached in LM theta 1, going for 0.0, last value in msg : ", msg_lm)
                msg_lm.theta_1 = 0.0
        
        if  180.0 - msg_lm.theta_2 + 4.0 > 165.0:
                print("Warning: Maximum theta bound approached in LM theta 2, going for 165.0, last value in msg : ", msg_lm)
                msg_lm.theta_2 = 165.0
        elif 180.0 - msg_lm.theta_2 + 4.0 < 0:
                print("Warning: Minimum theta bound approached in LM theta 2, going for 0.0, last value in msg : ", msg_lm)
                msg_lm.theta_2 = 0.0
        
        if  180.0 - msg_lm.theta_3 > 165.0:
                print("Warning: Maximum theta bound approached in LM theta 3, going for 165.0, last value in msg : ", msg_lm)
                msg_lm.theta_3 = 165.0
        elif 180.0 - msg_lm.theta_3 < 0:
                print("Warning: Minimum theta bound approached in LM theta 3, going for 0.0, last value in msg : ", msg_lm)
                msg_lm.theta_3 = 0.0

        kit_left.servo[6].angle = msg_lm.theta_1 - 19.0
        kit_left.servo[7].angle = 180.0 - msg_lm.theta_2 + 4.0
        kit_left.servo[8].angle = 180.0 - msg_lm.theta_3
        #print(" i heard ", kit_left.servo[6].angle, kit_left.servo[7].angle, kit_left.servo[8].angle) 

    def listener_callback_UR(self, msg_ru: ThetasLegRightUp):
        if 180.0 - msg_ru.theta_1 > 165.0:
                print("Warning: Maximum theta bound approached in RU theta 1, going for 165.0, last value in msg : ", msg_ru)
                msg_ru.theta_1 = 165.0
        elif 180.0 - msg_ru.theta_1 < 0:
                print("Warning: Minimum theta bound approached in RU theta 1, going for 0.0, last value in msg : ", msg_ru)
                msg_ru.theta_1 = 0.0
        
        if  msg_ru.theta_2 - 9.0 > 165.0:
                print("Warning: Maximum theta bound approached in RU theta 2, going for 165.0, last value in msg : ", msg_ru)
                msg_ru.theta_2 = 165.0
        elif msg_ru.theta_2 - 9.0 < 0:
                print("Warning: Minimum theta bound approached in RU theta 2, going for 0.0, last value in msg : ", msg_ru)
                msg_ru.theta_2 = 0.0
        
        if msg_ru.theta_3 - 3.0 > 165.0:
                print("Warning: Maximum theta bound approached in RU theta 3, going for 165.0, last value in msg : ", msg_ru)
                msg_ru.theta_3 = 165.0
        elif msg_ru.theta_3 - 3.0 < 0:
                print("Warning: Minimum theta bound approached in RU theta 3, going for 0.0, last value in msg : ", msg_ru)
                msg_ru.theta_3 = 0.0

        kit_right.servo[13].angle = 180.0 - msg_ru.theta_1 + 45.0
        kit_right.servo[14].angle = msg_ru.theta_2 - 9.0
        kit_right.servo[15].angle = msg_ru.theta_3 - 3.0   

    def listener_callback_UL(self, msg_lu: ThetasLegLeftup):

        if msg_lu.theta_1 - 10.0 > 165.0:
                print("Warning: Maximum theta bound approached in LU theta 1, going for 165.0, last value in msg : ", msg_lu)
                msg_lu.theta_1 = 165.0
        elif msg_lu.theta_1 - 10.0 < 0:
                print("Warning: Minimum theta bound approached in LU theta 1, going for 0.0, last value in msg : ", msg_lu)
                msg_lu.theta_1 = 0.0
        
        if 180.0 - msg_lu.theta_2 + 7.0 > 165.0:
                print("Warning: Maximum theta bound approached in LU theta 2, going for 165.0, last value in msg : ", msg_lu)
                msg_lu.theta_2 = 165.0
        elif 180.0 - msg_lu.theta_2 + 7.0 < 0:
                print("Warning: Minimum theta bound approached in LU theta 2, going for 0.0, last value in msg : ", msg_lu)
                msg_lu.theta_2 = 0.0
        
        if 180.0 - msg_lu.theta_3 + 2.0 > 165.0:
                print("Warning: Maximum theta bound approached in LU theta 3, going for 165.0, last value in msg : ", msg_lu)
                msg_lu.theta_3 = 165.0
        elif 180.0 - msg_lu.theta_3 + 2.0 < 0:
                print("Warning: Minimum theta bound approached in LU theta 3, going for 0.0, last value in msg : ", msg_lu)
                msg_lu.theta_3 = 0.0

        kit_left.servo[0].angle = msg_lu.theta_1 - 10.0
        kit_left.servo[1].angle = 180.0 - msg_lu.theta_2 + 7.0
        kit_left.servo[2].angle = 180.0 - msg_lu.theta_3 + 2.0
        #print(" i heard ul:  ", kit_left.servo[0].angle, kit_left.servo[1].angle, kit_left.servo[2].angle) 

    def listener_callback_DR(self, msg_rd: ThetasLegRightDown):
        if  msg_rd.theta_1 - 25.0 > 165.0:
                print("Warning: Maximum theta bound approached in RD theta 1, going for 165.0, last value in msg : ", msg_rd)
                msg_rd.theta_1 = 165.0
        elif msg_rd.theta_1 - 25.0 < 0:
                print("Warning: Minimum theta bound approached in RD theta 1, going for 0.0, last value in msg : ", msg_rd)
                msg_rd.theta_1 = 0.0
        
        if  msg_rd.theta_2 - 6.0 > 165.0:
                print("Warning: Maximum theta bound approached in RD theta 2, going for 165.0, last value in msg : ", msg_rd)
                msg_rd.theta_2 = 165.0
        elif msg_rd.theta_2 - 6.0 < 0:
                print("Warning: Minimum theta bound approached in RD theta 2, going for 0.0, last value in msg : ", msg_rd)
                msg_rd.theta_2 = 0.0
        
        if msg_rd.theta_3 - 6.0 > 165.0:
                print("Warning: Maximum theta bound approached in RD theta 3, going for 165.0, last value in msg : ", msg_rd)
                msg_rd.theta_3 = 165.0
        elif msg_rd.theta_3 - 6.0 < 0:
                print("Warning: Minimum theta bound approached in RD theta 3, going for 0.0, last value in msg : ", msg_rd)
                msg_rd.theta_3 = 0.0

        kit_right.servo[0].angle = msg_rd.theta_1 - 25.0
        kit_right.servo[1].angle = msg_rd.theta_2 - 6.0
        kit_right.servo[2].angle = msg_rd.theta_3 - 6.0   
        #print(" i heard ", kit_right.servo[0].angle, kit_right.servo[1].angle, kit_right.servo[2].angle) 
    
    def listener_callback_DL(self, msg_ld: ThetasLegLeftDown):
        if  msg_ld.theta_1 - 15.0 > 165.0:
                print("Warning: Maximum theta bound approached in LD theta 1, going for 165.0, last value in msg : ", msg_ld)
                msg_ld.theta_1 = 165.0
        elif msg_ld.theta_1 - 15.0 < 0:
                print("Warning: Minimum theta bound approached in LD theta 1, going for 0.0, last value in msg : ", msg_ld)
                msg_ld.theta_1 = 0.0
        
        if  msg_ld.theta_2 + 3.0 > 165.0:
                print("Warning: Maximum theta bound approached in LD theta 2, going for 165.0, last value in msg : ", msg_ld)
                msg_ld.theta_2 = 165.0
        elif msg_ld.theta_2 + 3.0 < 0:
                print("Warning: Minimum theta bound approached in LD theta 2, going for 0.0, last value in msg : ", msg_ld)
                msg_ld.theta_2 = 0.0
        
        if msg_ld.theta_3 + 2.0 > 165.0:
                print("Warning: Maximum theta bound approached in LD theta 3, going for 165.0, last value in msg : ", msg_ld)
                msg_ld.theta_3 = 165.0
        elif msg_ld.theta_3 + 2.0 < 0:
                print("Warning: Minimum theta bound approached in LD theta 3, going for 0.0, last value in msg : ", msg_ld)
                msg_ld.theta_3 = 0.0

        kit_left.servo[13].angle = msg_ld.theta_1 - 15.0
        kit_left.servo[14].angle = 180.0 - msg_ld.theta_2 + 3.0
        kit_left.servo[15].angle = 180.0 - msg_ld.theta_3 + 2.0  
        #print(" i heard ", kit_left.servo[13].angle, kit_left.servo[14].angle, kit_left.servo[15].angle) 
        

def main(args=None):

    rclpy.init(args=args)
    subscriber_for_servos = Angle_subscriber()
    kit_left.servo[3].angle = 90
    rclpy.spin(subscriber_for_servos)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber_for_servos.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()