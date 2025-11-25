import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
import time
import lgpio

GPIO_PIN = 23   #GPIO 27 FIRST ARM, 17 BASE, 22 2nd ARM, 23 WRISt, 24 Gripper
h = lgpio.gpiochip_open(4)  # Pi 5 uses chip
servo = GPIO_PIN
print("Testing servo on GPIO 27 (Pi 5)")
def set_angle(angle):
    """Set servo angle from -90 to +90 degrees"""
    pulse_width = 1500 + (angle * 1000 / 180)  # 1500us = center
    print(f"Setting angle to {angle}° (pulse width: {pulse_width:.0f}us)")
    
    # Send PWM pulses for 1 second
    for _ in range(50):
        lgpio.gpio_write(h, GPIO_PIN, 1)
        time.sleep(pulse_width / 1_000_000)
        lgpio.gpio_write(h, GPIO_PIN, 0)
        time.sleep((20000 - pulse_width) / 1_000_000)

while True:
    # for angle in range(-270, 360, 15):
    #     set_angle(angle)
    #     time.sleep(1)
    
    set_angle(20)    
    time.sleep(2)
    # set_angle(0)
    # time.sleep(2)


# factory = LGPIOFactory(chip=4)
# wrist_servo = Servo(23, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, pin_factory=factory)

# def set_wrist_angle(angle):
#     servo_value = (angle / 180.0) * 225.0 + 45.0
#     servo_value = angle / 270
#     wrist_servo.value = servo_value

# while True:
#     for angle in range(-180, 181, 30):
#         print(f"Setting wrist to {angle}°")
#         # servo_value = (angle / 180.0) * 225.0 + 45.0
#         # servo_value = angle / 270
#         wrist_servo.value = angle / 180
#         time.sleep(1)
    
#     # angle = -270
#     # set_wrist_angle(angle)