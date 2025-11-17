#!/usr/bin/env python3
import lgpio
import time

# Setup
GPIO_PIN = 17
h = lgpio.gpiochip_open(4)  # Pi 5 uses chip 4

print("Testing servo on GPIO 17 (Pi 5)")

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

# Test sequence
print("Center (0°)")
set_angle(0)
time.sleep(1)

print("Left (-90°)")
set_angle(-90)
time.sleep(1)

print("Right (90°)")
set_angle(90)
time.sleep(1)

print("Back to center (0°)")
set_angle(0)

lgpio.gpiochip_close(h)
print("Done!")