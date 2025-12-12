#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FaceVisualizer(Node):
    def __init__(self):
        super().__init__('face_visualizer')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/face_detection/image_annotated',
            self.image_callback,
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Create window
        cv2.namedWindow('Face Detection', cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('Face visualizer started. Press "q" to quit.')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display
        cv2.imshow('Face Detection', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FaceVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
