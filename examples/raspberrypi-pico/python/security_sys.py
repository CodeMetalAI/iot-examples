# Motion sensor based security system
#
# When motion is detected,
#    triggers an alaram
#    Blinks RED LED

import machine
import utime
from picozero import Buzzer, LED

# PIR sensor has 3 pins: Vcc - 5V, GND, and IN - any GPIO pin
pir_sensor = machine.Pin(3, machine.Pin.IN)
# Buzzer positive is in GPIO16.
buzzer = Buzzer(16)
# LED positive is in GPIO18.
led = LED(18)

def trigger_alarm():
    print("Trigger alarm")
    buzzer.beep(on_time=1, off_time=1, n=3)
    
def blink_led():
    print("Blink LED")
    led.blink(on_time=1, off_time=0.5, n=3, wait=True)

def motion_detected(pin):
    print("Somebody here!")
    trigger_alarm()
    blink_led()

pir_sensor.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_detected)
