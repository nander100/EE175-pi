from gpiozero import LED
from time import sleep

led = LED(17)

print("Blinking LED 5 times...")

for i in range(5):
        led.on()
        print("LED ON")
        sleep(0.5)  
        led.off()
        print("LED OFF")
        sleep(0.5) 
print("Done blinking.")

