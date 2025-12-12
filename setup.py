from setuptools import setup
import os
from glob import glob

package_name = 'face_detection_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include data files (Haar cascade)
        (os.path.join('share', package_name, 'data'), 
         glob('face_detection_pkg/data/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='advaith',
    maintainer_email='your_email@example.com',
    description='Face detection using ROS2 and OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'camera_publisher = face_detection_pkg.camera_publisher:main',
        'camera_publisher_sim = face_detection_pkg.camera_publisher_simulated:main',
        'camera_publisher_network = face_detection_pkg.camera_publisher_network:main',
        'face_detector = face_detection_pkg.face_detector:main',
        'face_visualizer = face_detection_pkg.face_visualizer:main',
    ],
},
)
