#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/face_detection/detections',
            10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            '/face_detection/image_annotated',
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Load Haar Cascade
        try:
            pkg_share = get_package_share_directory('face_detection_pkg')
            cascade_path = os.path.join(pkg_share, 'data', 
                                       'haarcascade_frontalface_default.xml')
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            
            if self.face_cascade.empty():
                raise RuntimeError('Failed to load Haar Cascade')
            
            self.get_logger().info(f'Loaded Haar Cascade from: {cascade_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading Haar Cascade: {e}')
            raise
        
        self.get_logger().info('Face detector started')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_frame'
        
        # Draw bounding boxes and create detection messages
        for (x, y, w, h) in faces:
            # Draw rectangle on image
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Add face count text
            cv2.putText(cv_image, 'Face', (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Create Detection2D message
            detection = Detection2D()
            detection.bbox.center.position.x = float(x + w/2)
            detection.bbox.center.position.y = float(y + h/2)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            
            # Add hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'face'
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
        
        # Add face count to image
        face_count_text = f'Faces: {len(faces)}'
        cv2.putText(cv_image, face_count_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Publish detections
        self.detection_pub.publish(detection_array)
        
        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        annotated_msg.header = detection_array.header
        self.image_pub.publish(annotated_msg)
        
        if len(faces) > 0:
            self.get_logger().info(f'Detected {len(faces)} face(s)')


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
