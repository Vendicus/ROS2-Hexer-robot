import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#import tflite_runtime.interpreter as tflite 

import time
import cv2
import numpy as np

class camera_sub(Node):
    def __init__ (self):
        super().__init__("vision_node")
        self.camera_sub = self.create_subscription(Image, 'camera_frame', self.camera_rec, 1)

        #loading image to process
        self.label_file = "/home/michal/ros2_ws/src/ROS2-Hexer-robot/vision_rpi_hexabot/resource/labels.txt"
        self.bridge = CvBridge()
        # AI model paths
        self.conifg_net_file = '/home/michal/ros2_ws/src/ROS2-Hexer-robot/data/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        self.model_weights = '/home/michal/ros2_ws/src/ROS2-Hexer-robot/data/frozen_inference_graph.pb'
        # model build
        self.detector = cv2.dnn.DetectionModel(self.model_weights, self.conifg_net_file)
        self.detector.setInputSize(320,320) # image size to use on detection 320x320
        self.detector.setInputScale(1.0/127.5) # scale = 1, doc
        self.detector.setInputMean((127.5, 127.5, 127.5)) # shape build as :(3, 320, 320, 1) so 3 shapes 
        self.detector.setInputSwapRB(True)

        #segregate in array all labels
        with open(self.label_file,'rt') as f:
            self.label = f.read().rstrip('\n').split('\n')

        self.start_time = 0

    def camera_rec(self, msg):
        #calculate time for fps
        currrent_time = time.time()
        fps = 1/(currrent_time - self.start_time)
        self.start_time = currrent_time

        #brdige between rpi and computer to cath image 
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = frame.astype(np.uint8) # convert frame to format ready to read by AI
    
        # data for rectangle, labels, all obejcts
        label_ID, conf, bbox = self.detector.detect(frame, confThreshold = 0.65)

        #creating boxes and labels on vision for all detected objects
        for class_ID, confidence, border in zip(np.array(label_ID).flatten(),np.array(conf).flatten(),bbox):
            

            if confidence < 0.70:
                box_color = (55,0,255)
            elif confidence >= 0.70 and confidence <= 0.80:
                box_color = (20,220,232)
            else :
                box_color = (52,235,28)

            cv2.rectangle(frame,border,color=box_color ,thickness=1)
            
            if border[0] > 280:
                x = -30
            else:
                x = 7
                
            cv2.putText(frame, self.label[class_ID-1].upper() + str(" ID: ") + str(class_ID), (border[0] + x, border[1] + 15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, box_color , 1) 
            cv2.putText(frame, str("PROB: ") + str(round(confidence, 2)),(border[0] + x, border[3] + border[1] - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.3, box_color ,1)
        
        frame = cv2.resize(frame, (1280,720), interpolation = cv2.INTER_AREA)
        cv2.putText(frame,"FPS : " + str(int(fps)), (20,70), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,0), 2) # fps text representation
        cv2.imshow('Vision from AI',frame)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args=args)

    camera_active = camera_sub()

    rclpy.spin(camera_active)
    
    camera_active.destroy_node()
    rclpy.shutdown()
    