from machine import Pin
import time

led = Pin(25, Pin.OUT)
while True:
    time.sleep_ms(2)
    led.value(1)
    time.sleep_ms(2)
    led.value(0)