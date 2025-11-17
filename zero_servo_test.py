import lgpio
import time
                                    
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
    set_angle(0)
    time.sleep(1)