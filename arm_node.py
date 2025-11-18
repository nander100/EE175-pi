#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

class SimpleArmIK(Node):
    def __init__(self):
        super().__init__('simple_arm_ik')
        
        # Arm dimensions
        self.a1 = 0.15    # Link 1 length: 150mm (base arm)
        self.a2 = 0.18    # Link 2 length: 180mm (second arm)
        self.base_height = 0.1  # Base mounted 100mm above ground

        # Workspace limits
        self.max_reach = self.a1 + self.a2  # 0.33 m
        self.min_reach = abs(self.a1 - self.a2)  # 0.03 m
        
        # Servo configuration
        self.servo1_offset = 45   # Servo 0° is actually 45° from base
        self.servo1_min = -80     # Base intersection limit
        self.servo1_max = 90      # Max angle

        # Servo direction configuration
        self.servo1_ccw_positive = True   # Servo1: positive = counter-clockwise
        self.servo2_ccw_positive = False  # Servo2: positive = clockwise (inverted)

        # Sensitivity scaling (less than 1.0 = less sensitive, more range)
        self.sensitivity = 0.3    # ADJUST: Lower = less sensitive, more hand movement needed

        # Axis inversion flags - adjust these if directions are wrong
        self.invert_z = False  # Set True if forward/back is reversed
        self.invert_y = True   # Set True if up/down is reversed

        # Setup servos
        factory = LGPIOFactory(chip=4)
        self.servo1 = Servo(27, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)  # Base
        self.servo2 = Servo(22, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)  # Joint 2
        
        # Subscribe to hand position
        self.sub = self.create_subscription(Float32MultiArray, 'hand/position', self.callback, 10)
        
        self.get_logger().info('Simple Arm IK Ready!')
        self.get_logger().info(f'Workspace: {self.min_reach:.3f}m to {self.max_reach:.3f}m')
    
    def callback(self, msg):
        z = msg.data[2]  # Hand z (depth/out)
        y = msg.data[1]  # Hand y (vertical/up)

        # Apply axis inversions if needed
        if self.invert_z:
            z = -z
        if self.invert_y:
            y = -y

        # Account for base height offset
        y = y - self.base_height

        # Apply sensitivity scaling for more range
        arm_z = z * self.sensitivity
        arm_y = y * self.sensitivity
        
        # Check if reachable
        distance = math.sqrt(arm_z**2 + arm_y**2)
        
        if distance > self.max_reach:
            self.get_logger().warn(f'Out of reach! {distance:.3f}m > {self.max_reach:.3f}m')
            return
        
        if distance < self.min_reach:
            self.get_logger().warn(f'Too close! {distance:.3f}m < {self.min_reach:.3f}m')
            return
        
        # Law of cosines
        # Calculate q2 (elbow)
        cos_q2 = (arm_z**2 + arm_y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        
        if abs(cos_q2) > 1.0:
            self.get_logger().warn('No IK solution!')
            return
        
        q2 = math.acos(cos_q2)
        
        # Calculate q1 (base)
        q1 = math.atan2(arm_y, arm_z) - math.atan2(self.a2 * math.sin(q2), self.a1 + self.a2 * math.cos(q2))
        
        # Convert to degrees
        q1_deg = math.degrees(q1)
        q2_deg = math.degrees(q2)
        
        # Apply servo offset (servo 0° = 45° actual)
        q1_deg_actual = q1_deg - self.servo1_offset
        
        # Check base intersection limit
        if q1_deg_actual < self.servo1_min:
            self.get_logger().warn(f'Base limit! {q1_deg_actual:.1f}° < {self.servo1_min}°')
            return
        
        # Clamp to servo limits
        q1_deg_actual = max(self.servo1_min, min(self.servo1_max, q1_deg_actual))
        q2_deg = max(-90, min(90, q2_deg))

        # Apply servo direction inversions
        servo1_cmd = q1_deg_actual if self.servo1_ccw_positive else -q1_deg_actual
        servo2_cmd = q2_deg if self.servo2_ccw_positive else -q2_deg

        # Move servos (convert to -1 to +1 range)
        self.servo1.value = servo1_cmd / 90.0
        self.servo2.value = servo2_cmd / 90.0
        
        self.get_logger().info(f'Hand: ({z:.3f}, {y:.3f}) → q1={q1_deg_actual:.1f}° q2={q2_deg:.1f}°')

def main():
    rclpy.init()
    node = SimpleArmIK()
    rclpy.spin(node)
    node.servo1.close()
    node.servo2.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()