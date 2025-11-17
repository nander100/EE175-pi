#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller') # jon is poop
        
        # Setup servos with lgpio factory
        factory = LGPIOFactory(chip=4)
        
        self.wrist_servo = Servo(23, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)
        self.finger_servo = Servo(24, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)
        
        # Create subscribers
        self.wrist_sub = self.create_subscription(
            Float32MultiArray,
            'hand/wrist_rotation',
            self.wrist_callback,
            10
        )
        
        self.finger_sub = self.create_subscription(
            Float32MultiArray,
            'hand/finger_bend',
            self.finger_callback,
            10
        )
        
        self.get_logger().info('Servo controller initialized')
    
    def wrist_callback(self, msg):
        """Handle wrist rotation data"""
        angle = msg.data[0]  # -180 to 180 degrees
        self.get_logger().info(f'Wrist: {angle:.1f}')
        # Clamp to ±90 degrees
        angle = max(-90, min(90, angle))
        
        # Convert to servo range (-1 to 1)
        servo_value = angle / 90.0
        
        self.wrist_servo.value = servo_value
       # self.get_logger().info(f'Wrist: {angle:.1f}° → Servo: {servo_value:.2f}')
        
    def finger_callback(self, msg):
        """Handle finger bend data"""
        bend_percent = msg.data[0]  # 0 to 100%
        
        # Clamp to 50-100% range (50% = closed, 100% = open)
        bend_percent = max(50, min(100, bend_percent))
        
        # Map 50-100% to -90 to 90 degrees
        # 50% → -90°, 100% → 90°
        angle = ((bend_percent - 70) / 50.0) * 180 - 90
        
        angle = ((bend_percent - 70) / 30.0) * 180 - 90  # Maps 70-100% to -90 to 90
        angle = max(-90, min(90, angle))  # Safety clamp
        
        # Convert to servo range (-1 to 1)
        servo_value = (angle / 90.0) 
        
        self.finger_servo.value = servo_value
        self.get_logger().info(f'Finger: {bend_percent:.1f}% → {angle:.1f}° → Servo: {servo_value:.2f}')

def main():
    rclpy.init()
    node = ServoController()
    rclpy.spin(node)
    node.wrist_servo.close()
    node.finger_servo.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

