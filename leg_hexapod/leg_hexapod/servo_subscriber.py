import rclpy
from rclpy.node import Node

from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

from hexapod_message.msg import ThetasLeg

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('theta_subscriber')
        self.subscription = self.create_subscription(
            ThetasLeg,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: ThetasLeg):
        kit.servo[0].angle = msg.theta_1
        kit.servo[1].angle = msg.theta_2
        kit.servo[2].angle = msg.theta_3


def main(args=None):
    kit.servo[0].angle = 90
    kit.servo[1].angle = 90
    kit.servo[2].angle = 110

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()