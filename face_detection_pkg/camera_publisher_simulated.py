#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from glob import glob


class SimulatedCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_simulated')
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Timer for publishing (10 FPS for simulation)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Load test images
        self.test_images_dir = os.path.expanduser('~/ros2_ws/test_images')
        self.image_files = sorted(glob(os.path.join(self.test_images_dir, '*.jpg')))
        
        if not self.image_files:
            self.get_logger().warn(f'No images found in {self.test_images_dir}')
            self.get_logger().info('Generating synthetic test image...')
            self.use_synthetic = True
            self.current_frame = self.generate_test_frame()
        else:
            self.get_logger().info(f'Loaded {len(self.image_files)} test images')
            self.use_synthetic = False
            self.current_idx = 0
        
        self.frame_count = 0
        self.get_logger().info('Simulated camera publisher started')
    
    def generate_test_frame(self):
        """Generate a synthetic test frame with a face-like oval"""
        import numpy as np
        
        # Create blank image
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 200
        
        # Draw a face-like oval (will be detected by Haar Cascade)
        cv2.ellipse(frame, (320, 240), (80, 100), 0, 0, 360, (255, 200, 180), -1)
        
        # Draw eyes
        cv2.circle(frame, (290, 220), 15, (50, 50, 50), -1)
        cv2.circle(frame, (350, 220), 15, (50, 50, 50), -1)
        
        # Draw mouth
        cv2.ellipse(frame, (320, 270), (40, 20), 0, 0, 180, (100, 50, 50), 2)
        
        # Add text
        cv2.putText(frame, 'Simulated Camera Feed', (150, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return frame
    
    def timer_callback(self):
        if self.use_synthetic:
            # Use generated frame
            frame = self.current_frame.copy()
            # Add frame counter
            cv2.putText(frame, f'Frame: {self.frame_count}', (10, 450),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Cycle through loaded images
            frame = cv2.imread(self.image_files[self.current_idx])
            
            if frame is None:
                self.get_logger().error(f'Failed to read {self.image_files[self.current_idx]}')
                return
            
            # Add frame info
            cv2.putText(frame, f'Image {self.current_idx + 1}/{len(self.image_files)}',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Move to next image (cycle through)
            self.current_idx = (self.current_idx + 1) % len(self.image_files)
        
        # Convert OpenCV image to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        
        # Publish
        self.publisher_.publish(msg)
        self.frame_count += 1
        
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
