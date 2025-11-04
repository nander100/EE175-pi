#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class HandSubscriber(Node):
    def __init__(self):
        super().__init__('hand_subscriber')
        
        self.finger_bend_sub = self.create_subscription(
            Float32MultiArray,
            'hand/finger_bend',
            self.finger_bend_callback,
            10
        )
        
        self.wrist_rotation_sub = self.create_subscription(
            Float32MultiArray,
            'hand/wrist_rotation',
            self.wrist_rotation_callback,
            10
        )
        
        self.hand_position_sub = self.create_subscription(
            Float32MultiArray,
            'hand/position',
            self.hand_position_callback,
            10
        )
        
        self.get_logger().info('Hand subscriber ready!')
    
    def finger_bend_callback(self, msg):
        bend = msg.data[0]
        self.get_logger().info(f'Finger Bend: {bend:.1f}%')
    
    def wrist_rotation_callback(self, msg):
        rotation = msg.data[0]
        self.get_logger().info(f'Wrist Rotation: {rotation:.1f}°')
    
    def hand_position_callback(self, msg):
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        self.get_logger().info(f'Position: [{x:.3f}, {y:.3f}, {z:.3f}]m')

def main():
    rclpy.init()
    subscriber = HandSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()