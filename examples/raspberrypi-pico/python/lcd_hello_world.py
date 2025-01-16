from machine import Pin, SoftI2C
from pico_i2c_lcd import I2cLcd

LCD_I2C_ADDR = 0x27
LCD_NUM_ROWS = 2
LCD_NUM_COLS = 16

# Initialize I2C and LCD objects
# SDA of display is connected to GP14.
# SCL of display is connected to GP15.
lcd_i2c = SoftI2C(sda=Pin(14), scl=Pin(15), freq=400000)
lcd = I2cLcd(lcd_i2c, LCD_I2C_ADDR, LCD_NUM_ROWS, LCD_NUM_COLS)

lcd.clear()
lcd.show_cursor()
lcd.move_to(0,0)
lcd.putstr("Hello World!")