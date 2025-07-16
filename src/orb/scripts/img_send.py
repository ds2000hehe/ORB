#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq
import cv2



class ImageSender(Node):
    def __init__(self):
        super().__init__('zmq_image_sender')

        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind('tcp://*:5555')
        self.get_logger().info('info')

        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.cbfunc, 10)

        self.bridge = CvBridge()

    def cbfunc(self, msg):
        
        try:

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            _, buffer = cv2.imencode('.jpg', frame)

            self.socket.send(buffer.tobytes())

            
        except Exception as e:
            self.get_logger().error('didnt work')
    

def main(args=None):
    rclpy.init(args=args)
    node = ImageSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()