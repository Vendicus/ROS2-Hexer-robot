import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

from itertools import zip_longest
import time

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

#messages for ros2
from hexapod_message.msg import ThetasAll, Sygnalizer, ThetasLegLeftMiddle, ThetasLegLeftDown, ThetasLegRightDown, ThetasLegLeftup, ThetasLegRightMiddle, ThetasLegRightUp
from std_msgs.msg import Float32, Bool

def stander():
        kit_left.servo[6].angle = 90.0 - 19.0
        kit_left.servo[7].angle = 180.0 - 90.0  + 4.0
        kit_left.servo[8].angle = 180.0 - 90.0 

        kit_right.servo[13].angle = 180.0 - 145.0 + 45.0
        kit_right.servo[14].angle = 90.0 - 9.0
        kit_right.servo[15].angle = 90.0 - 3.0

        kit_right.servo[0].angle = 145.0 - 25.0
        kit_right.servo[1].angle = 90.0 - 6.0
        kit_right.servo[2].angle = 90.0 - 6.0   

        time.sleep(1)

        kit_right.servo[6].angle = 90.0 - 22.0
        kit_right.servo[7].angle = 90.0 + 2.0
        kit_right.servo[8].angle = 90.0 - 4.0

        kit_left.servo[0].angle = 145.0- 10.0
        kit_left.servo[1].angle = 180.0 - 90.0 + 7.0
        kit_left.servo[2].angle = 180.0 - 90.0 + 2.0

        kit_left.servo[13].angle = 45.0 - 15.0
        kit_left.servo[14].angle = 180.0 - 90.0 + 3.0
        kit_left.servo[15].angle = 180.0 - 90.0 + 2.0   

        time.sleep(1)


class speed_sub(Node):
    def __init__(self):
        super().__init__('speed')   
        speeder = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription( Float32, 'speed', self.speed_callback, 10, callback_group=speeder)

    def speed_callback(self, msg:Float32): 
        global speed
        speed = msg.data

class shutdown_sub(Node):
    def __init__(self):
        super().__init__('shutdown')   
        shutdown = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription( Bool, 'shutdown', self.shutdown_callback , 10, callback_group=shutdown)

    def shutdown_callback(self, msg:Bool):
        global x 
        x = msg.data

class subscriber_theta_lm(Node):
    def __init__(self):
        super().__init__('theta_lm')   
        cos_LM = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_lm = self.create_subscription( ThetasAll, 'anglesMLLegList', self.servo_theta_lm_callback , 10, callback_group=cos_LM)

    def servo_theta_lm_callback(self, msg:ThetasAll):
        global list_lm 
        list_lm = list(msg.lista)
        
class subscriber_theta_ld(Node):
    def __init__(self):
        super().__init__('heta_ld')
        cos_LD = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_ld = self.create_subscription( ThetasAll, 'anglesDLLegList', self.servo_theta_ld_callback , 10, callback_group=cos_LD)
    
    def servo_theta_ld_callback(self, msg: ThetasAll):
        global list_ld 
        list_ld = list(msg.lista)
        
class subscriber_theta_lu(Node):
    def __init__(self):
        super().__init__('theta_lu')
        cos_LU = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_lu = self.create_subscription( ThetasAll, 'anglesULLegList', self.servo_theta_lu_callback , 10, callback_group=cos_LU)
        
    def servo_theta_lu_callback(self, msg: ThetasAll):
        global list_lu 
        list_lu = list(msg.lista)
   
class subscriber_theta_rm(Node):
    def __init__(self):
        super().__init__('theta_rm')
        cos_RM = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_rm = self.create_subscription( ThetasAll, 'anglesMRLegList', self.servo_theta_rm_callback , 10, callback_group=cos_RM)
    
    def servo_theta_rm_callback(self, msg: ThetasAll):
        global list_rm
        list_rm = list(msg.lista)
    
class subscriber_theta_rd(Node):
    def __init__(self):
        super().__init__('theta_rd')
        cos_RD = MutuallyExclusiveCallbackGroup() 
        self.theta_subscriber_rd = self.create_subscription( ThetasAll, 'anglesDRLegList', self.servo_theta_rd_callback , 10, callback_group=cos_RD)
    
    def servo_theta_rd_callback(self, msg: ThetasAll):
        global list_rd
        list_rd = list(msg.lista)

class subscriber_theta_ru(Node):
    def __init__(self):
        super().__init__('ru')
        cos_RU = MutuallyExclusiveCallbackGroup()
        self.theta_subscriber_ru = self.create_subscription(ThetasAll, 'anglesURLegList', self.servo_theta_ru_callback , 10, callback_group=cos_RU)

    def servo_theta_ru_callback(self, msg: ThetasAll):
        global list_ru
        list_ru = list(msg.lista)

         
class sender_first_cycle(Node):
    def __init__(self):
        super().__init__('sender_stand_cycle_one')
        stand_lm = MutuallyExclusiveCallbackGroup()
        stand_ru = MutuallyExclusiveCallbackGroup()
        stand_rd = MutuallyExclusiveCallbackGroup()
        
        self.publisher_send_one = self.create_publisher(Sygnalizer, 'Donesend', 10)
        self.publisher_send_theta_LM = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLM', 10, callback_group=stand_lm)
        self.publisher_send_theta_RU = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRU', 10, callback_group=stand_ru)
        self.publisher_send_theta_RD = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRD', 10, callback_group=stand_rd)

    
    def send_one(self):
        sen_lm = False
        sen_rd = False
        sen_ru = False

        global list_lm
        global list_rd
        global list_ru

        length_lm = len(list_lm)
        length_rd = len(list_rd)
        length_ru = len(list_ru)

        length_max = max(length_lm, length_rd, length_ru)

        nested_list_lm = [list_lm[i:i+3] for i in range(0, length_lm, 3)]
        nested_list_rd = [list_rd[i:i+3] for i in range(0, length_rd, 3)]
        nested_list_ru = [list_ru[i:i+3] for i in range(0, length_ru, 3)]

        iteration = 0
        sleeper = 0.01

        for ii, jj, zz in zip_longest(nested_list_lm, nested_list_rd, nested_list_ru, fillvalue=[None,None,None]):   
            if float(mcp.read_adc(1)) >= 300 and iteration > 30:
                sen_lm = True
            if ii[0] != None and sen_lm == False:
                kit_left.servo[6].angle = ii[0] - 19.0
                kit_left.servo[7].angle = 180.0 - ii[1] + 4.0
                kit_left.servo[8].angle = 180.0 - ii[2]

            if float(mcp.read_adc(7)) >= 500 and iteration > 30:
                sen_rd = True
            if jj[0] != None and sen_rd == False:
                kit_right.servo[0].angle = jj[0] - 25.0
                kit_right.servo[1].angle = jj[1] - 6.0
                kit_right.servo[2].angle = jj[2] - 6.0   

            #if sensors_data != None:
                #if sensors_data.sensor_4 >= 500 and iteration > 30:
                #    sen_ru = True
            if zz[0] != None and sen_ru == False:
                kit_right.servo[13].angle = 180.0 - zz[0] + 45.0
                kit_right.servo[14].angle = zz[1] - 9.0
                kit_right.servo[15].angle = zz[2] - 3.0

            iteration += 1 
            
            #smoothing steps
            if iteration < (length_max/2):
                iterator = 1 + iteration/5
            else:
                iterator = -1 -(length_max - iteration)/5

            smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
            time.sleep(smooth_step)
        
        sign = Sygnalizer()
        sign.syg = True
        sign.z_ld = 0.0
        sign.z_lu = 0.0
        sign.z_lm = 0.0
        sign.z_rd = 0.0
        sign.z_ru = 0.0
        sign.z_rm = 0.0
        self.publisher_send_one(sign)

        theta = ThetasLegLeftMiddle()
        theta.theta_1 = float(list_lm[-3])
        theta.theta_2 = float(list_lm[-2])
        theta.theta_3 = float(list_lm[-1])
        self.publisher_send_theta_LM(theta)
        theta.theta_1 = float(list_rd[-3])
        theta.theta_2 = float(list_rd[-2])
        theta.theta_3 = float(list_rd[-1])
        self.publisher_send_theta_RD(theta)
        theta.theta_1 = float(list_ru[-3])
        theta.theta_2 = float(list_ru[-2])
        theta.theta_3 = float(list_ru[-1])
        self.publisher_send_theta_RU(theta)



class sender_second_cycle(Node):
    def __init__(self):
        super().__init__('sender_stand_cycle_two')
        sygnalizer_gr = MutuallyExclusiveCallbackGroup()
        stand_rm = MutuallyExclusiveCallbackGroup()
        stand_lu = MutuallyExclusiveCallbackGroup()
        stand_ld = MutuallyExclusiveCallbackGroup()
        

        self.publisher_send_two = self.create_publisher(Sygnalizer, 'Donesend', 10, callback_group = sygnalizer_gr)
        self.publisher_send_theta_RM = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRM', 10, callback_group=stand_rm)
        self.publisher_send_theta_LU = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLU', 10, callback_group=stand_lu)
        self.publisher_send_theta_LD = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLD', 10, callback_group=stand_ld)

    
    def send_two(self):
        sen_rm = False
        sen_ld = False
        sen_lu = False

        global list_rm
        global list_ld
        global list_lu

        length_rm = len(list_rm)
        length_ld = len(list_ld)
        length_lu = len(list_lu)

        length_max = max(length_rm, length_ld, length_lu)

        nested_list_rm = [list_rm[i:i+3] for i in range(0, length_rm, 3)]
        nested_list_ld = [list_ld[i:i+3] for i in range(0, length_ld, 3)]
        nested_list_lu = [list_lu[i:i+3] for i in range(0, length_lu, 3)]

        iteration = 0
        sleeper = 0.01

        for ii, jj, zz in zip_longest(nested_list_rm, nested_list_ld, nested_list_lu, fillvalue=[None,None,None]):   
            if float(mcp.read_adc(6)) >= 300 and iteration > 30:
                sen_rm = True
            if ii[0] != None and sen_rm == False:
                kit_right.servo[6].angle = ii[0] - 22.0
                kit_right.servo[7].angle = ii[1] + 2.0
                kit_right.servo[8].angle = ii[2] - 4.0

            if float(mcp.read_adc(2)) >= 500 and iteration > 30:
                sen_ld = True
            if jj[0] != None and sen_ld == False:
                kit_left.servo[13].angle = jj[0] - 15.0
                kit_left.servo[14].angle = 180.0 - jj[1] + 3.0
                kit_left.servo[15].angle = 180.0 - jj[2] + 2.0   

            if float(mcp.read_adc(0)) >= 500 and iteration > 30:
                sen_lu = True
            if zz[0] != None and sen_lu == False:
                kit_left.servo[0].angle = zz[0]- 10.0
                kit_left.servo[1].angle = 180.0 - zz[1] + 7.0
                kit_left.servo[2].angle = 180.0 - zz[2] + 2.0

            iteration += 1 
            
            #smoothing steps
            if iteration < (length_max/2):
                iterator = 1 + iteration/5
            else:
                iterator = -1 -(length_max - iteration)/5

            smooth_step = (sleeper + 0.01)/(iterator**2) + sleeper
            time.sleep(smooth_step)
        
        sign = Sygnalizer()
        sign.syg = True
        sign.z_ld = 0.0
        sign.z_lu = 0.0
        sign.z_lm = 0.0
        sign.z_rd = 0.0
        sign.z_ru = 0.0
        sign.z_rm = 0.0
        self.publisher_send_two(sign)

        theta = ThetasLegLeftMiddle()
        theta.theta_1 = float(list_rm[-3])
        theta.theta_2 = float(list_rm[-2])
        theta.theta_3 = float(list_rm[-1])
        self.publisher_send_theta_RM(theta)
        theta.theta_1 = float(list_ld[-3])
        theta.theta_2 = float(list_ld[-2])
        theta.theta_3 = float(list_ld[-1])
        self.publisher_send_theta_LD(theta)
        theta.theta_1 = float(list_lu[-3])
        theta.theta_2 = float(list_lu[-2])
        theta.theta_3 = float(list_lu[-1])
        self.publisher_send_theta_LU(theta)

class sender_first_walk_cycle(Node):
    def __init__(self):
        super().__init__('sender_cycle_one_walk')
        walk_first = MutuallyExclusiveCallbackGroup()
        stand_rm = MutuallyExclusiveCallbackGroup()
        stand_lu = MutuallyExclusiveCallbackGroup()
        stand_ld = MutuallyExclusiveCallbackGroup()
        stand_lm = MutuallyExclusiveCallbackGroup()
        stand_ru = MutuallyExclusiveCallbackGroup()
        stand_rd = MutuallyExclusiveCallbackGroup()

        self.publisher_send_one = self.create_publisher(Sygnalizer, 'Donesend', 10, callback_group = walk_first)
        self.publisher_send_theta_RM = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRM', 10, callback_group=stand_rm)
        self.publisher_send_theta_LU = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLU', 10, callback_group=stand_lu)
        self.publisher_send_theta_LD = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLD', 10, callback_group=stand_ld)
        self.publisher_send_theta_LM = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLM', 10, callback_group=stand_lm)
        self.publisher_send_theta_RU = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRU', 10, callback_group=stand_ru)
        self.publisher_send_theta_RD = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRD', 10, callback_group=stand_rd)

    def send_one(self, speed):
            sen_rm = False
            sen_ld = False
            sen_lu = False

            global list_rm
            global list_ld
            global list_lu
            global list_lm
            global list_rd
            global list_ru

            length_rm = len(list_rm)
            length_ld = len(list_ld)
            length_lu = len(list_lu)
            length_lm = len(list_lm)
            length_rd = len(list_rd)
            length_ru = len(list_ru)
            length_max = max(length_rm,length_rd,length_ru,length_lm,length_ld,length_lu)

            z_LD = None
            z_LU = None
            z_RM = None

            nested_list_rm = [list_rm[i:i+4] for i in range(0, length_rm, 4)]
            nested_list_ld = [list_ld[i:i+4] for i in range(0, length_ld, 4)]
            nested_list_lu = [list_lu[i:i+4] for i in range(0, length_lu, 4)]
            nested_list_lm = [list_lm[i:i+3] for i in range(0, length_lm, 3)]
            nested_list_rd = [list_rd[i:i+3] for i in range(0, length_rd, 3)]
            nested_list_ru = [list_ru[i:i+3] for i in range(0, length_ru, 3)]
            
            #print("nested list ld :", nested_list_ld)
            #print([[None] * 3]*4 + nested_list_lm )
            
            iteration = 0
            theta_rm = ThetasLegLeftMiddle()
            theta_lu = ThetasLegLeftMiddle()
            theta_ld = ThetasLegLeftMiddle()

            for ii, jj, zz, qq, ww, xx in zip_longest(nested_list_rm, nested_list_ld, nested_list_lu,[[None] * 3]*50 + nested_list_lm , [[None] * 3]*50 + nested_list_rd,[[None] * 3]*50 + nested_list_ru, fillvalue=[None,None,None, None]):   
                if float(mcp.read_adc(6)) >= 500 and iteration > 50 and ii[0] != None:
                    sen_rm = True   
                    z_RM = ii[3]
                    theta_rm.theta_1 = ii[0]
                    theta_rm.theta_2 = ii[1]
                    theta_rm.theta_3 = ii[2]
                if ii[0] != None and sen_rm == False:
                    kit_right.servo[6].angle = ii[0] - 22.0
                    kit_right.servo[7].angle = ii[1] + 2.0
                    kit_right.servo[8].angle = ii[2] - 4.0
                
                if float(mcp.read_adc(2)) >= 500 and iteration > 50 and jj[0] != None:
                    sen_ld = True
                    z_LD = jj[3]
                    theta_lu.theta_1 = jj[0]
                    theta_lu.theta_2 = jj[1]
                    theta_lu.theta_3 = jj[2]
                if jj[0] != None and sen_ld == False:
                    kit_left.servo[13].angle = jj[0] - 15.0
                    kit_left.servo[14].angle = 180.0 - jj[1] + 3.0
                    kit_left.servo[15].angle = 180.0 - jj[2] + 2.0   

                if float(mcp.read_adc(0)) >= 500 and iteration > 50 and zz[0] != None:
                    sen_lu = True
                    z_LU = zz[3]
                    theta_ld.theta_1 = zz[0]
                    theta_ld.theta_2 = zz[1]
                    theta_ld.theta_3 = zz[2]
                if zz[0] != None and sen_lu == False:
                    kit_left.servo[0].angle = zz[0]- 10.0
                    kit_left.servo[1].angle = 180.0 - zz[1] + 7.0
                    kit_left.servo[2].angle = 180.0 - zz[2] + 2.0

                if qq[0] != None and iteration > 50:
                    kit_left.servo[6].angle = qq[0] - 19.0
                    kit_left.servo[7].angle = 180.0 - qq[1] + 4.0
                    kit_left.servo[8].angle = 180.0 - qq[2]

                if ww[0] != None and iteration > 50:
                    kit_right.servo[0].angle = ww[0] - 25.0
                    kit_right.servo[1].angle = ww[1] - 6.0
                    kit_right.servo[2].angle = ww[2] - 6.0   

                if xx[0] != None and iteration > 50:
                    kit_right.servo[13].angle = 180.0 - xx[0] + 45.0
                    kit_right.servo[14].angle = xx[1] - 9.0
                    kit_right.servo[15].angle = xx[2] - 3.0

                iteration += 1 

                #smoothing steps
                if iteration < (length_max/2):
                    iterator = 1 + iteration/5
                else:
                    iterator = -1 -(length_max - iteration)/5
                    
                smooth_step = (speed + 0.01)/(iterator**2) + speed
                time.sleep(smooth_step)

            if z_LD == None:
                z_LD = -215.0
            if z_RM == None:
                z_RM = -215.0
            if z_LU == None:
                z_LU = -215.0

            sygn = Sygnalizer()
            sygn.syg = True
            sygn.z_ld = z_LD
            sygn.z_rm = z_RM
            sygn.z_lu = z_LU
            sygn.z_lm = 0.0
            sygn.z_rd = 0.0
            sygn.z_ru = 0.0
            self.publisher_send_one(sygn) 

            theta = ThetasLegLeftMiddle()
            if sen_rm == True:
                self.publisher_send_theta_RM(theta_rm)
            else:   
                theta.theta_1 = float(list_rm[-4])
                theta.theta_2 = float(list_rm[-3])
                theta.theta_3 = float(list_rm[-2])
                self.publisher_send_theta_RM(theta)
            if sen_ld == True:
                self.publisher_send_theta_LD(theta_ld)
            else:
                theta.theta_1 = float(list_ld[-4])
                theta.theta_2 = float(list_ld[-3])
                theta.theta_3 = float(list_ld[-2])
                self.publisher_send_theta_LD(theta)
            if sen_lu == True:
                self.publisher_send_theta_LU(theta_lu)
            else:
                theta.theta_1 = float(list_lu[-4])
                theta.theta_2 = float(list_lu[-3])
                theta.theta_3 = float(list_lu[-2])
                self.publisher_send_theta_LU(theta)
            theta.theta_1 = float(list_lm[-3])
            theta.theta_2 = float(list_lm[-2])
            theta.theta_3 = float(list_lm[-1])
            self.publisher_send_theta_LM(theta)
            theta.theta_1 = float(list_rd[-3])
            theta.theta_2 = float(list_rd[-2])
            theta.theta_3 = float(list_rd[-1])
            self.publisher_send_theta_RD(theta)
            theta.theta_1 = float(list_ru[-3])
            theta.theta_2 = float(list_ru[-2])
            theta.theta_3 = float(list_ru[-1])
            self.publisher_send_theta_RU(theta)

class sender_second_walk_cycle(Node):
    def __init__ (self):
        super().__init__('sender_cycle_two_walk')
        walk_second = MutuallyExclusiveCallbackGroup()
        stand_rm = MutuallyExclusiveCallbackGroup()
        stand_lu = MutuallyExclusiveCallbackGroup()
        stand_ld = MutuallyExclusiveCallbackGroup()
        stand_lm = MutuallyExclusiveCallbackGroup()
        stand_ru = MutuallyExclusiveCallbackGroup()
        stand_rd = MutuallyExclusiveCallbackGroup()

        self.publisher_send_two = self.create_publisher(Sygnalizer, 'Donesend', 10, callback_group = walk_second)
        self.publisher_send_theta_RM = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRM', 10, callback_group=stand_rm)
        self.publisher_send_theta_LU = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLU', 10, callback_group=stand_lu)
        self.publisher_send_theta_LD = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLD', 10, callback_group=stand_ld)
        self.publisher_send_theta_LM = self.create_publisher(ThetasLegLeftMiddle, 'AnglesLM', 10, callback_group=stand_lm)
        self.publisher_send_theta_RU = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRU', 10, callback_group=stand_ru)
        self.publisher_send_theta_RD = self.create_publisher(ThetasLegLeftMiddle, 'AnglesRD', 10, callback_group=stand_rd)
    
    def send_two(self, speed):
            sen_lm = False
            sen_rd = False
            sen_ru = False

            global list_rm
            global list_ld
            global list_lu
            global list_lm
            global list_rd
            global list_ru

            length_rm = len(list_rm)
            length_ld = len(list_ld)
            length_lu = len(list_lu)
            length_lm = len(list_lm)
            length_rd = len(list_rd)
            length_ru = len(list_ru)
            length_max = max(length_rm,length_rd,length_ru,length_lm,length_ld,length_lu)

            z_RD = None
            z_RU = None
            z_LM = None

            nested_list_rm = [list_rm[i:i+3] for i in range(0, length_rm, 3)]
            nested_list_ld = [list_ld[i:i+3] for i in range(0, length_ld, 3)]
            nested_list_lu = [list_lu[i:i+3] for i in range(0, length_lu, 3)]
            nested_list_lm = [list_lm[i:i+4] for i in range(0, length_lm, 4)]
            nested_list_rd = [list_rd[i:i+4] for i in range(0, length_rd, 4)]
            nested_list_ru = [list_ru[i:i+4] for i in range(0, length_ru, 4)]

            
            iteration = 0
            thetas_lm = ThetasLegLeftMiddle()
            thetas_rd = ThetasLegLeftMiddle()
            thetas_ru = ThetasLegLeftMiddle()

            for ii, jj, zz, qq, ww, xx in zip_longest([[None] * 3] * 50 + nested_list_rm,[[None] * 3] * 50 + nested_list_ld,[[None] * 3] * 50 + nested_list_lu, nested_list_lm, nested_list_rd, nested_list_ru, fillvalue=[None,None,None]):    
                if float(mcp.read_adc(1)) >= 300 and iteration > 50 and qq[0] != None:
                    sen_lm = True
                    z_LM = qq[3]
                    thetas_lm.theta_1 = qq[0]
                    thetas_lm.theta_2 = qq[1]
                    thetas_lm.theta_3 = qq[2]
                if qq[0] != None and sen_lm == False:
                    kit_left.servo[6].angle = qq[0] - 19.0
                    kit_left.servo[7].angle = 180.0 - qq[1] + 4.0
                    kit_left.servo[8].angle = 180.0 - qq[2]

                if float(mcp.read_adc(7)) >= 500 and iteration > 50 and ww[0] != None:
                    sen_rd = True
                    z_RD = ww[3]
                    thetas_rd.theta_1 = ww[0]
                    thetas_rd.theta_2 = ww[1]
                    thetas_rd.theta_3 = ww[2]
                if ww[0] != None and sen_rd == False:
                    kit_right.servo[0].angle = ww[0] - 25.0
                    kit_right.servo[1].angle = ww[1] - 6.0
                    kit_right.servo[2].angle = ww[2] - 6.0   

                #if float(mcp.read_adc(3)) >= 500 and iteration > 50 and xx[0] != None:
                #    sen_ru = True
                #    z_RU = xx[3]
                #    thetas_ru.theta_1 = xx[0]
                #    thetas_ru.theta_2 = xx[1]
                #    thetas_ru.theta_3 = xx[2]
                if xx[0] != None and sen_ru == False:
                    kit_right.servo[13].angle = 180.0 - xx[0] + 45.0
                    kit_right.servo[14].angle = xx[1] - 9.0
                    kit_right.servo[15].angle = xx[2] - 3.0
                
                if ii[0] != None and iteration > 50:
                    kit_right.servo[6].angle = ii[0] - 22.0
                    kit_right.servo[7].angle = ii[1] + 2.0
                    kit_right.servo[8].angle = ii[2] - 4.0

                if jj[0] != None and iteration > 50:
                    kit_left.servo[13].angle = jj[0] - 15.0
                    kit_left.servo[14].angle = 180.0 - jj[1] + 3.0
                    kit_left.servo[15].angle = 180.0 - jj[2] + 2.0   

                if zz[0] != None and iteration > 50:
                    kit_left.servo[0].angle = zz[0]- 10.0
                    kit_left.servo[1].angle = 180.0 - zz[1] + 7.0
                    kit_left.servo[2].angle = 180.0 - zz[2] + 2.0
                iteration += 1 

                #smoothing steps
                if iteration < (length_max/2):
                    iterator = 1 + iteration/5
                else:
                    iterator = -1 -(length_max - iteration)/5
                    
                smooth_step = (speed + 0.01)/(iterator**2) + speed
                time.sleep(smooth_step)

            if z_RD == None:
                z_RD = -215.0
            if z_LM == None:
                z_LM = -215.0
            if z_RU == None:
                z_RU = -215.0

            sygn = Sygnalizer()
            sygn.syg = True
            sygn.z_rd = z_RD
            sygn.z_lm = z_LM
            sygn.z_ru = z_RU
            sygn.z_ld = 0.0
            sygn.z_rm = 0.0
            sygn.z_lu = 0.0
            self.publisher_send_two(sygn)   

            theta = ThetasLegLeftMiddle()
            theta.theta_1 = float(list_rm[-3])
            theta.theta_2 = float(list_rm[-2])
            theta.theta_3 = float(list_rm[-1])
            self.publisher_send_theta_RM(theta)
            theta.theta_1 = float(list_ld[-3])
            theta.theta_2 = float(list_ld[-2])
            theta.theta_3 = float(list_ld[-1])
            self.publisher_send_theta_LD(theta)
            theta.theta_1 = float(list_lu[-3])
            theta.theta_2 = float(list_lu[-2])
            theta.theta_3 = float(list_lu[-1])
            self.publisher_send_theta_LU(theta)
            if sen_lm == True:
                self.publisher_send_theta_LM(thetas_lm)
            else:
                theta.theta_1 = float(list_lm[-4])
                theta.theta_2 = float(list_lm[-3])
                theta.theta_3 = float(list_lm[-2])
                self.publisher_send_theta_LM(theta)
            if sen_rd == True:
                self.publisher_send_theta_RD(thetas_rd)
            else:
                theta.theta_1 = float(list_rd[-4])
                theta.theta_2 = float(list_rd[-3])
                theta.theta_3 = float(list_rd[-2])
                self.publisher_send_theta_RD(theta)
            if sen_ru == True:
                self.publisher_send_theta_RU(thetas_ru)
            else:
                theta.theta_1 = float(list_ru[-4])
                theta.theta_2 = float(list_ru[-3])
                theta.theta_3 = float(list_ru[-2])
                self.publisher_send_theta_RU(theta)
   
        
def main(args=None):
    rclpy.init(args=args)
    global x
    x = False
    global speed

    theta_sub_lm = subscriber_theta_lm()
    theta_sub_rd = subscriber_theta_rd()
    theta_sub_ru = subscriber_theta_ru()
    theta_sub_rm = subscriber_theta_rm()
    theta_sub_ld = subscriber_theta_ld()
    theta_sub_lu = subscriber_theta_lu()

    sender_one = sender_first_cycle()
    sender_two = sender_second_cycle()

    sender_walk_one = sender_first_walk_cycle()
    sender_walk_two = sender_second_walk_cycle()

    speed_subscription = speed_sub()
    shutdown_subscription = shutdown_sub()
    
    
    stander()
    
    while True:
        rclpy.spin_once(speed_subscription)
        print("speed")
        if x is True:
            break
        
        # servos angles reading
        if speed == 0.0:
            
            # --- standing cycles ---
            rclpy.spin_once(theta_sub_lm)
            rclpy.spin_once(theta_sub_rd)
            rclpy.spin_once(theta_sub_ru)
            sender_one.send_one()

            rclpy.spin_once(theta_sub_rm)
            rclpy.spin_once(theta_sub_ld)
            rclpy.spin_once(theta_sub_lu)
            sender_two.send_two()
        else:
            # --- walking cycles ---
            rclpy.spin_once(theta_sub_lm)
            rclpy.spin_once(theta_sub_rd)
            rclpy.spin_once(theta_sub_ru)
            rclpy.spin_once(theta_sub_rm)
            rclpy.spin_once(theta_sub_ld)
            rclpy.spin_once(theta_sub_lu)
            sender_walk_one.send_one(speed)

            rclpy.spin_once(theta_sub_lm)
            rclpy.spin_once(theta_sub_rd)
            rclpy.spin_once(theta_sub_ru)
            rclpy.spin_once(theta_sub_rm)
            rclpy.spin_once(theta_sub_ld)
            rclpy.spin_once(theta_sub_lu)
            sender_walk_two.send_two(speed)
        
        rclpy.spin_once(shutdown_subscription)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    theta_sub_lm.destroy_node()
    theta_sub_rm.destroy_node()
    theta_sub_lu.destroy_node()
    theta_sub_ld.destroy_node()
    theta_sub_rd.destroy_node()
    theta_sub_ru.destroy_node()
    sender_one.destroy_node()
    sender_two.destroy_node()
    sender_walk_one.destroy_node()
    sender_walk_two.destroy_node()
    shutdown_subscription.destroy_node()
    speed_subscription.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()