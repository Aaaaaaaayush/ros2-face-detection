import cv2
import socket
import struct
import traceback

def main():
    print("=" * 50)
    print("WINDOWS CAMERA SERVER")
    print("=" * 50)
    
    # Open camera
    print("\n[1/4] Opening camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ ERROR: Cannot open camera!")
        input("Press Enter to exit...")
        return
    
    print("✓ Camera opened successfully!")
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ret, test_frame = cap.read()
    if ret:
        print(f"✓ Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
    
    # Create socket server
    print("\n[2/4] Creating network server...")
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', 9999))
        server_socket.listen(1)
        print("✓ Server listening on port 9999")
    except Exception as e:
        print(f"❌ Failed to create server: {e}")
        cap.release()
        input("Press Enter to exit...")
        return
    
    print("\n[3/4] Waiting for WSL connection...")
    print("👉 Now run ROS2 launch in WSL\n")
    
    try:
        server_socket.settimeout(60)
        conn, addr = server_socket.accept()
        print(f"✓ Connected to: {addr}")
        print("\n[4/4] Streaming camera feed...\n")
        conn.settimeout(None)
        
        frame_count = 0
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # JPEG quality 80%
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("⚠ Failed to capture frame")
                    break
                
                # Encode frame as JPEG (avoids NumPy serialization)
                _, jpeg_data = cv2.imencode('.jpg', frame, encode_params)
                jpeg_bytes = jpeg_data.tobytes()
                
                # Send size + data
                size = len(jpeg_bytes)
                conn.sendall(struct.pack("!I", size))  # Send size (4 bytes)
                conn.sendall(jpeg_bytes)  # Send JPEG data
                
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"📹 Sent {frame_count} frames")
                
                # Optional: Display
                cv2.imshow('Camera Server (Press Q to quit)', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("\n👋 Quit requested")
                    break
                    
        except KeyboardInterrupt:
            print("\n👋 Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
            traceback.print_exc()
            
    except socket.timeout:
        print("\n⏱ Timeout waiting for connection")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        traceback.print_exc()
    finally:
        print("\n🧹 Cleaning up...")
        cap.release()
        cv2.destroyAllWindows()
        try:
            conn.close()
        except:
            pass
        server_socket.close()
        print("✓ Done\n")
        input("Press Enter to exit...")

if __name__ == '__main__':
    main()