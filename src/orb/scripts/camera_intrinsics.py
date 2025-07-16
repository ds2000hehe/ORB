import rclpy
import rclpy.signals
from sensor_msgs.msg import CameraInfo
import numpy as np
from rclpy.node import Node

K_matrix = None

    
class CameraInfo(Node):
    def __init__(self):
        super().__init__('camera_info')
        self.sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.K_matrix = None

    def camera_info_callback(self, msg):
        
        if self.K_matrix is None:
            self.K_matrix = np.array(msg.K).reshape((3, 3))
            np.save('camera_intrinsics.py', self.K_matrix)
            rclpy.shutdown()

def main():
    rclpy.init()
    node = CameraInfo
    rclpy.spin(node)

if __name__ == '__main__':
    main()