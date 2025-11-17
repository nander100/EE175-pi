#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

try:
    from gpiozero import Servo
    from gpiozero.pins.lgpio import LGPIOFactory
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: gpiozero not available. Running in simulation mode.")

class ServoAngleCalculator(Node):
    def __init__(self):
        super().__init__('servo_angle_calculator')
        
        # Setup GPIO for Raspberry Pi 5
        self.servo_pin = 17
        self.servo = None
        self.gpio_initialized = False
        
        self.get_logger().info(f'GPIO Available: {GPIO_AVAILABLE}')
        
        if GPIO_AVAILABLE:
            try:
                # Use lgpio factory for Pi 5
                factory = LGPIOFactory(chip=4)
                
                # Initialize servo with proper pulse width range
                # min_pulse_width: 1ms (1000us) for -90°
                # max_pulse_width: 2ms (2000us) for +90°
                self.servo = Servo(
                    self.servo_pin,
                    min_pulse_width=1.0/1000,
                    max_pulse_width=2.0/1000,
                    pin_factory=factory
                )
                
                self.gpio_initialized = True
                self.get_logger().info('Servo initialized successfully on GPIO 17!')
                
                # Move to center position on startup
                self.servo.mid()
                self.get_logger().info('Servo moved to center position')
                
            except Exception as e:
                self.get_logger().error(f'Servo initialization failed: {e}')
        
        # Subscribe to hand position topic
        self.hand_position_sub = self.create_subscription(
            Float32MultiArray,
            'hand/position',
            self.hand_position_callback,
            10
        )
        
        self.get_logger().info('Servo Angle Calculator ready!')
        self.get_logger().info('Waiting for messages on /hand/position...')
    
    def hand_position_callback(self, msg):
        x = msg.data[0]
        z = msg.data[2]
        
        # Calculate rotation angle
        angle_rad = math.atan(z/x)
        angle_deg = math.degrees(angle_rad) 

        if x < 0.0:
            angle_deg += 90

        if x >= 0.0:
            angle_deg -= 90

        angle_deg *= -5
        self.get_logger().info(f'Position: x={x:.3f}, z={z:.3f} | Angle: {angle_deg:.2f}°')
        
        # Set servo position
        self.set_servo_angle(angle_deg)
    
    def set_servo_angle(self, angle):
        """Set servo angle using gpiozero"""
        if not self.gpio_initialized or self.servo is None:
            return
        
        try:
            # Clamp angle to servo range (-90 to +90)
            angle = max(-90, min(90, angle))
            
            # Convert angle to gpiozero value range (-1 to +1)
            # -90° -> -1, 0° -> 0, +90° -> +1
            servo_value = angle / 90.0
            
            # Set the servo position
            self.servo.value = servo_value
            
        except Exception as e:
            self.get_logger().error(f'Failed to set servo: {e}')
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.servo is not None:
            try:
                # Move to center before closing
                self.servo.mid()
                self.servo.close()
                self.get_logger().info('Servo cleanup complete')
            except Exception as e:
                self.get_logger().error(f'Cleanup error: {e}')

def main():
    rclpy.init()
    node = ServoAngleCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()