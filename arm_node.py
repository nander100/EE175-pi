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
        self.a1 = 0.16    # Link 1 length (meters)
        self.a2 = 0.0865  # Link 2 length (meters)
        
        # Workspace limits
        self.max_reach = self.a1 + self.a2  # 0.2465 m
        self.min_reach = abs(self.a1 - self.a2)  # 0.0735 m
        
        # Servo configuration
        self.servo1_offset = 45   # Servo 0° is actually 45° from base
        self.servo1_min = -80     # Base intersection limit
        self.servo1_max = 90      # Max angle
        
        # Motion amplification (greater than 1.0 = amplify hand movements to larger arm movements)
        # NOTE: Since neutral is at max_reach, only inward movement is possible.
        # Higher amplification = more responsive but smaller usable camera range
        self.amplification_factor = 2.5    # ADJUST: Higher = small hand movements create larger arm movements

        # Neutral/home positions
        # Camera neutral: hand at [0, 0, 0.5] (half meter away)
        self.neutral_hand_z = 0.5
        self.neutral_hand_y = 0.0

        # Arm neutral: fully extended (q1=0°, q2=0°) → arm_z = a1+a2, arm_y = 0
        self.neutral_arm_z = self.a1 + self.a2  # 0.2465m
        self.neutral_arm_y = 0.0

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

        # Calculate offset from neutral position in camera space
        offset_z = z - self.neutral_hand_z
        offset_y = y - self.neutral_hand_y

        # Amplify the offset (not the absolute position)
        amplified_offset_z = offset_z * self.amplification_factor
        amplified_offset_y = -offset_y * self.amplification_factor  # Flip y direction

        # Apply to arm neutral position
        arm_z = self.neutral_arm_z + amplified_offset_z
        arm_y = self.neutral_arm_y + amplified_offset_y
        
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
        
        # Move servos (convert to -1 to +1 range)
        self.servo1.value = q1_deg_actual / 90.0
        self.servo2.value = q2_deg / 90.0
        
        self.get_logger().info(f'Hand: ({z:.3f}, {y:.3f}) Offset: ({offset_z:.3f}, {offset_y:.3f}) → Arm: ({arm_z:.3f}, {arm_y:.3f}) → q1={q1_deg_actual:.1f}° q2={q2_deg:.1f}°')

def main():
    rclpy.init()
    node = SimpleArmIK()
    rclpy.spin(node)
    node.servo1.close()
    node.servo2.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()