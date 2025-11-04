import socket
import json
import time
from gpiozero import Servo

# ---------------------- CONFIG ----------------------
HOST = '0.0.0.0'   # Listen on all interfaces
PORT = 5000        # Same port as sender
SERVO_PIN = 17     # GPIO pin connected to servo signal
# DS3235 servo pulse range (from datasheet)
MIN_PULSE = 0.0005
MAX_PULSE = 0.0025
# ----------------------------------------------------

# Initialize the servo once (not inside loop)
servo = Servo(SERVO_PIN, min_pulse_width=MIN_PULSE, max_pulse_width=MAX_PULSE)

def map_bend_to_servo_value(bend_percentage):
    """
    Map bend (0–100%) to servo.value (-1 to 1)
    which corresponds to 0°–270° on the DS3235.
    """
    bend_percentage = max(0, min(100, bend_percentage))  # clamp to 0–100
    servo_value = (bend_percentage / 50.0) - 1.0         # linear map 0→-1, 100→+1
    return servo_value

def control_robot_arm(data):
    """
    Process incoming hand data and control the servo.
    """
    bend = data.get('bend_percentage', 0)
    servo_value = map_bend_to_servo_value(bend)
    servo.value = servo_value

    # Optional: print telemetry
    print(f"\n{'='*50}")
    print(f"Hand: {data['hand']}")
    print(f"Position: X={data['x']:.4f}m, Y={data['y']:.4f}m, Z={data['z']:.4f}m")
    print(f"Index Finger Bend: {bend:.1f}% -> Servo Value: {servo_value:.2f}")
    print(f"Latency: {(time.time() - data['timestamp'])*1000:.1f} ms")
    print(f"{'='*50}")

def main():
    # Setup TCP server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    print(f"Receiver started on {HOST}:{PORT}")
    print("Waiting for connection from PC...")

    while True:
        try:
            client_socket, address = server_socket.accept()
            print(f"Connected to {address}")

            buffer = ""
            while True:
                try:
                    data = client_socket.recv(4096).decode('utf-8')
                    if not data:
                        print("Connection closed by client")
                        break

                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            try:
                                hand_data = json.loads(line)
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
            print("\nShutting down receiver...")
            break
        except Exception as e:
            print(f"Server error: {e}")
            time.sleep(1)

    server_socket.close()
    servo.detach()
    print("Receiver stopped.")

if __name__ == "__main__":
    main()