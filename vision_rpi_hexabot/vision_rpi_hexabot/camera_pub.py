import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class camera_publihser(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_frame', 1)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.caper = cv2.VideoCapture(0)

        # --> use when needed to reduce usage of system resource

        self.caper.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        self.caper.set(cv2.CAP_PROP_FRAME_WIDTH, 320)

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.caper.read()
        frame = self.bridge.cv2_to_imgmsg(frame,'bgr8')

        if not ret:
            print("cannot recieve data from camera (stream). ")

        self.publisher_.publish(frame)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = camera_publihser()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()