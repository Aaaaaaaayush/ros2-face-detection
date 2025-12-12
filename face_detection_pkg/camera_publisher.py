#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Timer for publishing (30 FPS = 0.033 seconds)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # OpenCV VideoCapture
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            raise RuntimeError('Cannot open camera')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # CV Bridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        self.get_logger().info('Camera publisher started')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert OpenCV image to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # Publish
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to capture frame')
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
