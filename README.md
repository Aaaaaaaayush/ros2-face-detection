# ROS2 Face Detection System

Real-time face detection using ROS2 Humble, OpenCV, and Haar Cascade classifier. Works with WSL2 for cross-platform Windows-Linux integration.

![Face Detection Demo](docs/demo.gif)

## 🎯 Features

- Real-time face detection at 25-30 FPS
- ROS2 modular architecture with 3 nodes
- Cross-platform: Windows camera → WSL2 processing
- JPEG compression for efficient network transfer
- Multiple face detection support
- Bounding box visualization

## 🏗️ Architecture
```
Windows (Camera Server) → TCP/IP → WSL2 Ubuntu
                                    ├─ Camera Publisher Node
                                    ├─ Face Detector Node
                                    └─ Visualizer Node
```

## 📋 Requirements

- Windows 10/11 with WSL2
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- OpenCV 4.x
- Webcam

## 🚀 Installation

### 1. Install ROS2 Humble
```bash
# Add ROS2 repository
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 2. Install Dependencies
```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs -y
pip3 install opencv-python "numpy<2"
```

### 3. Clone This Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/ros2-face-detection.git face_detection_pkg
```

### 4. Build
```bash
cd ~/ros2_ws
colcon build --packages-select face_detection_pkg
source install/setup.bash
```

## 🎮 Usage

### Terminal 1 - Windows PowerShell
```powershell
cd path\to\camera_server.py
python camera_server.py
```

### Terminal 2 - WSL Ubuntu
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch face_detection_pkg face_detection_network_launch.py
```

### Stop the System

- Press `Ctrl+C` in both terminals
- Or press `q` in the visualization window

## 📁 Project Structure
```
face_detection_pkg/
├── face_detection_pkg/
│   ├── __init__.py
│   ├── camera_publisher_network.py    # Camera subscriber node
│   ├── face_detector.py                # Face detection node
│   ├── face_visualizer.py              # Visualization node
│   └── data/
│       └── haarcascade_frontalface_default.xml
├── launch/
│   ├── face_detection_launch.py
│   └── face_detection_network_launch.py
├── scripts/
│   └── camera_server.py                # Windows camera server
├── docs/
│   └── images/
├── README.md
├── package.xml
└── setup.py
```

## 🔧 Configuration

### Adjust Detection Sensitivity

Edit `face_detection_pkg/face_detector.py`:
```python
faces = face_cascade.detectMultiScale(
    gray,
    scaleFactor=1.1,    # Decrease for more detections
    minNeighbors=5,     # Increase to reduce false positives
    minSize=(30, 30)    # Minimum face size in pixels
)
```

### Change Camera Resolution

Edit `camera_server.py`:
```python
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)   # Default: 640
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)   # Default: 480
```

## 📊 Performance

| Metric | Value |
|--------|-------|
| Frame Rate | 25-30 FPS |
| Detection Latency | ~100ms |
| Network Bandwidth | ~2 Mbps |
| CPU Usage | ~15-20% |

## 🐛 Troubleshooting

### Connection Refused Error

**Problem:** `[ERROR] Failed to connect: [Errno 111] Connection refused`

**Solution:**
1. Ensure camera_server.py is running in Windows first
2. Check Windows Firewall (allow port 9999)
3. Verify Windows IP: `ip route | grep default` in WSL

### NumPy Error

**Problem:** `No module named 'numpy._core'`

**Solution:**
```bash
pip3 uninstall numpy -y
pip3 install "numpy<2"
```

### Black Screen in Visualizer

**Problem:** Window opens but shows black screen

**Solution:**
- Check if camera_server.py is connected
- Verify with: `ros2 topic hz /camera/image_raw`

## 🎓 Learning Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [OpenCV Documentation](https://docs.opencv.org/)
- [Haar Cascade Algorithm](https://docs.opencv.org/4.x/db/d28/tutorial_cascade_classifier.html)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👤 Authors

**Aayush Bhanushali**

- GitHub: [Aayush Bhanushali](https://github.com/Aaaaaaaayush)

**Amarpreet Singh Chaman**

- GitHub: [Amarpreet Singh Chaman](https://github.com/amarpreetc-hue)

**Advaith Ajithkumar**

- GitHub: [Advaith Ajithkumar](https://github.com/AdvaithAjithkumar)

## 🙏 Acknowledgments

- ROS2 Community
- OpenCV Contributors
- Haar Cascade pre-trained models

## 📧 Contact

For questions or feedback, please open an issue or contact me at advaithajithkumar@gmail.com

---

⭐ Star this repo if you find it helpful!
