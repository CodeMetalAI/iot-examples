# Passive infrared sensor based motion detector

import machine
import utime

# PIR sensor has 3 pins: Vcc - 5V, GND, and IN - any GPIO pin
pir_sensor = machine.Pin(3, machine.Pin.IN)

def motion_detected(pin):
    print("Somebody here!")

pir_sensor.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_detected)