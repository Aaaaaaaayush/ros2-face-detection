from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='face_detection_pkg',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='face_detection_pkg',
            executable='face_detector',
            name='face_detector',
            output='screen'
        ),
        Node(
            package='face_detection_pkg',
            executable='face_visualizer',
            name='face_visualizer',
            output='screen'
        ),
    ])
