# Code to show different color patterns on WS2812 RGB strip with 8 LEDs.

import neopixel
from time import sleep

# 32 LED strip connected to X8.
p = machine.Pin.board.GP18
N_LEDS = 8
n = neopixel.NeoPixel(p, n=N_LEDS)

r = (255, 0, 0)
g = (0, 255, 0)
b = (0, 0, 255)
z = (0, 0, 0)

def set_led(n, i, val):
    n[i] = val

def paint(n):
# Update the strip.
    n.write()

def apply_pattern(n, pat):
    print(f'Applying {pat}')
    for i in range(N_LEDS):
        set_led(n, i, pat[i])
    paint(n)

def cycle(pat, off):
    new_pat = []
    for i in range(N_LEDS):
        #print(i, (i + off) % N_LEDS)
        new_pat.append(pat[(i + off) % N_LEDS])
    return new_pat

rgb_pat = [r, g, b, r, g, b, r, g]
red_pat = [z, z, z, z, z, z, z, r]
green_pat = [z, z, z, z, z, z, z, g]
blue_pat = [z, z, z, z, z, z, z, b]

for sec in range(5, 1, -1):
    delay = sec/30
    
    print("RGB cycle")
    apply_pattern(n, rgb_pat)
    for off in range(25):
         apply_pattern(n, cycle(rgb_pat, off))
         sleep(delay)

    print("Red cycle")
    for off in range(25):
        apply_pattern(n, cycle(red_pat, off))
        sleep(delay)

    print("green cycle")
    for off in range(25):
        apply_pattern(n, cycle(green_pat, off))
        sleep(delay)

    print("blue cycle")
    for off in range(25):
        apply_pattern(n, cycle(blue_pat, off))
        sleep(delay)
