import socket
import json
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# Setup pigpio factory for better servo control
try:
    factory = PiGPIOFactory()
    servo = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)
    print("Servo initialized with pigpio")
except:
    # Fallback to default if pigpio isn't available
    servo = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025)
    print("Servo initialized with default pin factory")

# Setup server
HOST = '0.0.0.0'  # Listen on all network interfaces
PORT = 5000

def control_robot_arm(data):
    """
    Process incoming hand data and control the robotic arm.
    Maps finger bend percentage to servo position.
    """
    print(f"\n{'='*50}")
    print(f"Hand: {data['hand']}")
    print(f"Position: X={data['x']:.4f}m, Y={data['y']:.4f}m, Z={data['z']:.4f}m")
    print(f"Index Finger Bend: {data['bend_percentage']:.1f}%")
    print(f"Latency: {(time.time() - data['timestamp'])*1000:.1f}ms")
    
    # Map bend percentage (0-100%) to servo value (-1 to 1)
    # 0% bent (straight finger) = -1 (servo minimum position)
    # 100% bent (fully bent finger) = 1 (servo maximum position)
    bend_percentage = data['bend_percentage']
    servo_value = (bend_percentage / 50.0) - 1.0  # Maps 0-100 to -1 to 1
    
    # Clamp to valid range
    servo_value = max(-1.0, min(1.0, servo_value))
    
    # Move servo
    servo.value = servo_value
    
    print(f"Servo Position: {servo_value:.2f}")
    print(f"{'='*50}")

def main():
    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind and listen
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    
    print(f"Receiver started on {HOST}:{PORT}")
    print("Servo ready on GPIO 17")
    print("Waiting for connection from PC...")
    
    try:
        while True:
            try:
                # Accept connection
                client_socket, address = server_socket.accept()
                print(f"Connected to {address}")
                
                # Buffer for incomplete JSON data
                buffer = ""
                
                while True:
                    try:
                        # Receive data
                        data = client_socket.recv(4096).decode('utf-8')
                        
                        if not data:
                            print("Connection closed by client")
                            break
                        
                        # Add to buffer
                        buffer += data
                        
                        # Process all complete JSON objects in buffer
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            
                            if line.strip():
                                try:
                                    # Parse JSON
                                    hand_data = json.loads(line)
                                    
                                    # Control robot arm with received data
                                    control_robot_arm(hand_data)
                                    
                                except json.JSONDecodeError as e:
                                    print(f"JSON decode error: {e}")
                    
                    except ConnectionResetError:
                        print("Connection reset by client")
                        break
                    except Exception as e:
                        print(f"Error receiving data: {e}")
                        break
                
                client_socket.close()
                print("Connection closed. Waiting for new connection...")
                
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"Server error: {e}")
                time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nShutting down receiver...")
    finally:
        # Center servo before closing
        servo.value = 0
        time.sleep(0.5)
        servo.close()
        server_socket.close()
        print("Receiver stopped.")

if __name__ == "__main__":
    main()