# HCSR04 based ultrasonic sensor for distance measurement

from machine import Pin, SoftI2C
from pico_i2c_lcd import I2cLcd
from picozero import DistanceSensor
from time import sleep

LCD_I2C_ADDR = 0x27
LCD_NUM_ROWS = 2
LCD_NUM_COLS = 16

# Initialize I2C and LCD objects
# SDA of display is connected to GP14.
# SCL of display is connected to GP15.
lcd_i2c = SoftI2C(sda=Pin(14), scl=Pin(15), freq=400000)
lcd = I2cLcd(lcd_i2c, LCD_I2C_ADDR, LCD_NUM_ROWS, LCD_NUM_COLS)

# HCSR04 This economical sensor provides 2cm to 4m of non-contact
# measurement functionality with a ranging accuracy that can reach up to 3mm.
ultrasonic = DistanceSensor(echo=17, trigger=3, max_distance=4)

# details https://thepihut.com/blogs/raspberry-pi-tutorials/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi?srsltid=AfmBOopwDIP0szVzOSC8p9GK3GjsQBeolC2ExQMUScNqY9nv2RHZKnkG
# 1k and 2k resistors needed.
# Vcc - 5V
# echo -> 1k R -> GP17
#              -> 2K R -> GND
# trigger - GP3
while True:
    lcd.clear()
    lcd.move_to(0,0)
    # Distance in meters. Max is max_distance
    lcd.putstr(str(ultrasonic.distance) + "m")
    print(str(ultrasonic.distance) + "m")
