#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
import lgpio
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller') # jon is poop
        
        # Setup servos with lgpio factory
        factory = LGPIOFactory(chip=4)
        
        self.GPIO_PIN = 23   # GPIO pin for wrist servo

        self.h = lgpio.gpiochip_open(4)  # Pi 5 uses chip

        # self.wrist_servo = Servo(23, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)
        # self.finger_servo = Servo(self.GPIO_PIN, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)
        
        self.finger_sub = self.create_subscription(
            Float32MultiArray,
            'hand/finger_bend',
            self.finger_callback,
            10
        )
        
        self.get_logger().info('Servo controller initialized')
        
    def finger_callback(self, msg):
        """Handle finger bend data"""
        bend_percent = msg.data[0]  # 0 to 100%
        
        def set_angle(angle):
            """Set servo angle from -90 to +90 degrees"""
            pulse_width = 1500 + (angle * 1000 / 180)  # 1500us = center
            print(f"Setting angle to {angle}° (pulse width: {pulse_width:.0f}us)")
            
            # Send PWM pulses for 1 second
            # for _ in range(50):
            lgpio.gpio_write(self.h, self.GPIO_PIN, 1)
            time.sleep(pulse_width / 1_000_000)
            lgpio.gpio_write(self.h, self.GPIO_PIN, 0)
            time.sleep((20000 - pulse_width) / 1_000_000)

        if bend_percent < 50.0:
            set_angle(20)
            servo_value = 20.0
        else:
            servo_value = set_angle(90.0)
            servo_value = 90.0

        
        # self.finger_servo.value = servo_value
        self.get_logger().info(f'Finger: {bend_percent:.1f}% → {servo_value:.1f}° → Servo: {servo_value:.2f}')

def main():
    rclpy.init()
    node = ServoController()
    rclpy.spin(node)
    node.finger_servo.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

