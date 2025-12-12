#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct
import numpy as np


class NetworkCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_network')
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Connect to Windows camera server
        self.get_logger().info('Connecting to Windows camera server...')
        
        # Get Windows IP
        import subprocess
        try:
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if 'default via' in line:
                    windows_ip = line.split()[2]
                    break
            else:
                windows_ip = '172.29.176.1'
        except:
            windows_ip = '172.29.176.1'
        
        self.get_logger().info(f'Connecting to {windows_ip}:9999')
        
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(10)
            self.client_socket.connect((windows_ip, 9999))
            self.client_socket.settimeout(None)
            
            self.get_logger().info('Connected to camera server!')
            
            # Timer for receiving frames
            self.timer = self.create_timer(0.033, self.timer_callback)
            
            self.frame_count = 0
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
    
    def timer_callback(self):
        try:
            # Receive frame size (4 bytes)
            size_data = self.recv_all(4)
            if not size_data:
                return
            
            frame_size = struct.unpack("!I", size_data)[0]
            
            # Receive JPEG data
            jpeg_data = self.recv_all(frame_size)
            if not jpeg_data:
                return
            
            # Decode JPEG to frame
            jpeg_array = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(jpeg_array, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().error('Failed to decode frame')
                return
            
            # Convert to ROS message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # Publish
            self.publisher_.publish(msg)
            
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Error receiving frame: {e}')
    
    def recv_all(self, size):
        """Helper to receive exact number of bytes"""
        data = b''
        while len(data) < size:
            packet = self.client_socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def destroy_node(self):
        if hasattr(self, 'client_socket'):
            self.client_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetworkCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
