from picozero import LED
from time import sleep

# Anode of LED is connected to GP16.
led = LED(16)

print("Blinking")
led.blink(on_time=1, off_time=0.5, n=5, wait=True)
sleep(6)
led.off()
print("Blinking finished")