
<div align="center">
  <a href="https://codemetal.ai">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="figs/CM_LIGHT_LOGO.png">
      <source media="(prefers-color-scheme: light)" srcset="figs/CM_DARK_LOGO.png">
      <img src="figs/CM_LIGHT_LOGO.png">
    </picture>
  </a>
</div>

# Introduction

The key advantage of programming IoT applications in MicroPython or CircuitPython is quick prototyping, but the performance of those Python applications on the IoT hardware is typically slow. Same applications programmed in Arduino C are typically much faster on the IoT hardware, but programming in Arduino C (with details such as pointers, etc.) is cumbersome. This is where the [CodeMetal](https://www.codemetal.ai/) IoT pipeline -- the transpilation software built at Code Metal for the hackathon -- steps in. The pipeline allows you to quickly develop IoT applications in Python while automatically translating them to Arduino C for optimized deployment on IoT hardware.

This repository contains MicroPython, CircuitPython, and Arduino based applications and any necessary software for Raspberry Pi Pico from Raspberry Pi Foundation and ESP32 based boards from Heltec Automation. In addition, this repository also contains instructions to connect with the Code Metal hackathon IoT pipeline and leverage it from quick prototyping in Python to optimized deployment via Arduino C.

Happy hacking IoT apps!

**Handy links to sections:**

- [Tracker - board details](#1-heltec-automation-wireless-tracker-v11)
- [Wifi-lora-V3 - board details](#2-heltec-automation-wifi-lora-v31)
- [Installation](#software)
- [Example Apps](#applications)
- [Using Python to Arduino translator](#using-codemetal-hackathon-transpiler)

**Hey, if you are beginner to IoT, ESP32 programming and have not heard these things before, don't worry! We have a [doc](doc/begineers_guide_to_running_lora_on_esp32.pdf) that captures step-by-step instructions to get LoRa-based Send/Recv working with these boards. Additionally, Raspberry Pi [documentation](https://www.raspberrypi.com/documentation/microcontrollers/micropython.html) is quite useful.**

## (1) [Raspberry Pi Pico](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)

<img src="https://www.raspberrypi.com/documentation/microcontrollers/images/pico-2.png?hash=c4b2ccf2504c98fdd111405dd95346b5" width="390">

Raspberry Pi Pico is a low-cost, high-performance microcontroller board with flexible digital interfaces.

The Raspberry Pi Pico family, referred to as Pico or Pico 1, comes in 4 variants: Pico, Pico H, Pico W, Pico WH. While the second-generation Raspberry Pi Pico family, referred to as Pico 2, comes in the following variants: Pico 2 and Pico 2 W. `W` stands for Pico containing 2.4GHz wireless interfaces (802.11n) using the Infineon CYW43439. While `H` stands for Pico with pre-soldered headers.

Overall, some of the key features of Pico family of microcontrollers include:
- [RP2040](https://www.raspberrypi.com/documentation/microcontrollers/silicon.html#rp2040) or similar microcontroller chip designed by Raspberry Pi in the United Kingdom
- Dual-core Arm Cortex M0+ processor, flexible clock running up to 133 MHz (Pico 1)
- on-board memory, USB support, low-power modes, GPIO pins, clock and timer on-chip,
- temperature sensor

Additional Details:

- [Raspberry Pi MicroPython quick reference](https://docs.micropython.org/en/latest/rp2/quickref.html)
- [Pico 1 Pin diagram](https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg)
- [Pico 2 Pin diagram](https://www.raspberrypi.com/documentation/microcontrollers/images/pico-2-r4-pinout.svg)
- [Pico 1 Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf), [Pico 1 W Datasheet](https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf) 
- [Pico 2 Datasheet](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf), [Pico 2 W Datasheet](https://datasheets.raspberrypi.com/picow/pico-2-w-datasheet.pdf)
- [MicroPython SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf), [C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
- [Arduino library from Heltec](https://github.com/HelTecAutomation/Heltec_ESP32)
- [Resetting flash](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#resetting-flash-memory)

## (2) [Heltec Automation Wireless Tracker v1.1](https://resource.heltec.cn/download/Wireless_Tracker/Wireless_Tracker%20_1.1.pdf)

<img src="figs/WirelessTracker.jpg" width="250">

This board contains following components:

- CPU - ESP32-S3FN8 ([datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)). ESP32-S3 is a low-power MCU-based system on a chip (SoC) with integrated 2.4 GHz Wi-Fi and Bluetooth® Low Energy (Bluetooth LE). It consists of a high-performance dual-core microprocessor (Xtensa® 32-bit LX7), a ULP coprocessor, a Wi-Fi baseband, a Bluetooth LE baseband, RF module, and numerous peripherals.
- LCD Display: Onboard 0.96-inch 160*80 dot matrix TFT LCD display, based on [ST7735](https://www.displayfuture.com/Display/datasheet/controller/ST7735.pdf)
- [UC6580](https://en.unicore.com/products/dual-band-gps-chip-uc6580/) (GPS SoC): via GNSS (also other satellite systems - QZSS, GLONASS, NAVIC, Galileo)
- [SX1262](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262) (LoRa transceiver) - 868MHz frequency, about -139dBm high sensitivity, +20 dBm power output, high reliability, transmission distance (measured open area communication distance 3.6Km).

Additional Details:

- [Official development doc from Heltec](https://resource.heltec.cn/download/Wireless_Tracker/Wireless_Tracker%20_1.1.pdf)
- [Pin diagram](https://cdn.shopify.com/s/files/1/0084/7465/6845/files/diagram.png?v=1694250615)
- [MicroPython firmware for ESP32](https://micropython.org/download/ESP32_GENERIC_S3/)
- [Arduino library from Heltec](https://github.com/HelTecAutomation/Heltec_ESP32)

## (3) [Heltec Automation Wifi LoRa v3.1](https://heltec.org/project/wifi-lora-32-v3/)

<img src="figs/wifi_lora_v3.jpg" width="250">

This board contains following components:

- CPU - ESP32-S3FN8 ([datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)). ESP32-S3 is a low-power MCU-based system on a chip (SoC) with integrated 2.4 GHz Wi-Fi and Bluetooth® Low Energy (Bluetooth LE). It consists of a high-performance dual-core microprocessor (Xtensa® 32-bit LX7), a ULP coprocessor, a Wi-Fi baseband, a Bluetooth LE baseband.
- OLED Display: Onboard 0.96-inch 128*64 dot matrix OLED display, based on [SSD1306](https://www.digikey.com/htmldatasheets/production/2047793/0/0/1/ssd1306.html)
- [SX1262](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262) (LoRa transceiver) - 868MHz frequency, about -139dBm high sensitivity, +20 dBm power output, high reliability, transmission distance (measured open area communication distance 3.6Km).
- Integrated [CP2102](https://www.silabs.com/interface/usb-bridges/classic/device.cp2102?tab=specs) USB to serial port chip,
- Onboard 32MByte Flash

Additional Details:

- [Official development doc from Heltec](https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA_V3(Rev1.1).pdf)
- [Official webpage from Heltec](https://heltec.org/project/wifi-lora-32-v3/)
- [Pin diagram](https://heltec.org/wp-content/uploads/2023/09/wifi-lora-32-pin_map.png)
- [MicroPython firmware for ESP32](https://micropython.org/download/ESP32_GENERIC_S3/)
- [CircuitPython firmware for the board](https://circuitpython.org/board/heltec_esp32s3_wifi_lora_v3/)
- [Arduino library from Heltec](https://github.com/HelTecAutomation/Heltec_ESP32)

# Software

Below we list the necessary Python and Arduino software for the boards mentioned above.

## Base Python software

- A good IDE for writing MicroPython or CircuitPython programs is [Thonny](https://thonny.org/).
- Raspberry Pi Pico supports MicroPython. Its firmware can be found [here](https://micropython.org/download/RPI_PICO/).
- Heltec Wireless Tracker supports MicroPython only. Its firmware can be found [here](https://micropython.org/download/ESP32_GENERIC_S3/).
- Heltec Wifi LoRa V3 board supports both MicroPython (shared above) and CircuitPython that can be found [here](https://circuitpython.org/board/heltec_esp32s3_wifi_lora_v3/).

Firmware for Raspberry Pi Pico can be installed via UF2 bootloader by holding down the BOOTSEL button while plugging the board into USB. The UF2 file should then be copied to the USB mass storage device that appears.

Firmware for Heltec boards can be flashed on the board using `esptool.py` as:

```
pip install esptool.py
```

```
# To erase flash on first use
# Find the port under /dev
esptool.py --chip esp32s3 --port <port> erase_flash
```

```
# To flash downloaded firmware
esptool.py --chip esp32s3 --port <port> write_flash -z 0 <downloaded_file>
```
- MicroPython or CircuitPython application may require particular libraries to be installed (mentioned in Applications below). These libraries can be installed by navigating to `Tools -> Manage plug-ins` in Thonny IDE.

## Arduino software

Install Arduino IDE from [here](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE).

### Installing specific Arduino libraries for Raspberry Pi Pico

These libraries cover various sensors that are typically used with Raspberry Pi Pico. Feel free to skip libraries for sensors that you may not be using.

```
arduino-cli core update-index && \
arduino-cli core install \
        --additional-urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json \
        RP2040:rp2040 && \
arduino-cli lib install "Adafruit NeoPixel" && \
arduino-cli lib install "HCSR04" && \
arduino-cli lib install "MFRC522" && \
arduino-cli lib install "Adafruit MPR121" && \
arduino-cli lib install "Adafruit MPU6050" && \
arduino-cli lib install "DHT sensor library" && \
arduino-cli lib install "IRremote" && \
arduino-cli lib install "Servo" && \
arduino-cli lib install "Adafruit ImageReader Library" && \
arduino-cli lib install "Adafruit SH110X" && \
arduino-cli lib install "ArduinoBLE" && \
arduino-cli lib install "RotaryEncoder" && \
arduino-cli config set library.enable_unsafe_install true && \
arduino-cli lib install --git-url https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library.git && \
arduino-cli lib install --git-url https://github.com/jarzebski/Arduino-MPU6050/ && \
arduino-cli lib install --git-url https://github.com/wonho-maker/Adafruit_SH1106.git && \
arduino-cli config set library.enable_unsafe_install false
```

After installation, select `Raspberry Pi Pico` in `tools/Board` or by clicking drop-down box near top-left corner of IDE. After installation, select appropriate board and port as [here](https://support.arduino.cc/hc/en-us/articles/4406856349970-Select-board-and-port-in-Arduino-IDE).

### Installing specific Arduino libraries for Heltec boards

Install necessary Arduino libraries Heltec boards as below.

```
arduino-cli core update-index && \
arduino-cli core install esp32:esp32 && \
arduino-cli lib install "Heltec ESP32 Dev-Boards" && \
arduino-cli lib install "Heltec_ESP32_LoRa_v3" && \
arduino-cli lib install "Adafruit GFX Library" && \
arduino-cli lib install "Adafruit SSD1306" && \
arduino-cli lib install "Adafruit ST7735 and ST7789 Library" && \
arduino-cli lib install "Adafruit Unified Sensor" && \
arduino-cli lib install "Adafruit BME280 Library" && \
arduino-cli lib install "ESP8266 and ESP32 OLED driver for SSD1306 displays" && \
arduino-cli lib install "LiquidCrystal" && \
arduino-cli lib install "LoRaRF" && \
arduino-cli lib install "EByte LoRa E22 library" && \
arduino-cli lib install "EByte LoRa E220 library" && \
arduino-cli lib install "EByte LoRa E32 library" && \
arduino-cli lib install "TinyGPSPlus" && \
arduino-cli lib install "TFT" && \
arduino-cli lib install "TFT_eSPI" && \
arduino-cli lib install "ArduinoJson" && \
arduino-cli lib install "PubSubClient" && \
arduino-cli lib install "Crypto" && \
arduino-cli lib install "Chrono" && \
arduino-cli lib install "Base64"
```

One of the LoRA libraries require a special treatment because of a symbol name conflict between a [popular LoRa library](https://github.com/sandeepmistry/arduino-LoRa/blob/master/src/LoRa.h) and [Heltec's LoRa APIs](https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/src/lora/LoRa.h). Install this special library as:

```
arduino-cli config set library.enable_unsafe_install true && \
arduino-cli lib install --git-url https://github.com/nhasabnic/arduino-LoRa && \
arduino-cli config set library.enable_unsafe_install false
```

- For Heltec Wireless Tracker, select `Heltec Wireless Tracker` in `tools/Board` or by clicking drop-down box near top-left corner of IDE.
- For Heltec Wifi LoRa V3, select `Heltec WiFi LoRa 32 (v3)` in `tools/Board` or by clicking drop-down box near top-left corner of IDE.
- After installation, select appropriate board and port as [here](https://support.arduino.cc/hc/en-us/articles/4406856349970-Select-board-and-port-in-Arduino-IDE).


# Applications

Below we list the applications, their language, and necessary library dependencies to get them working.

## Apps for Raspberry Pi Pico

| Component  | File  |  Board | Language | Necessary software |
| :---- | :---- | :---  | :---- | :--- |
| **Display** | [lcd_hello_world.py](examples/raspberrypi-pico/python/lcd_hello_world.py) | RaspberryPi Pico | MicroPython | [`pico_i2c_lcd`](examples/raspberrypi-pico/python/pico_i2c_lcd.py) |
| | [lcd_api.py](examples/raspberrypi-pico/python/lcd_api.py) | RaspberryPi Pico | MicroPython | Standard MicroPython |
| **LEDs** | [led_blink.py](examples/raspberrypi-pico/python/led_blink.py) | RaspberryPi Pico | MicroPython | Standard MicroPython |
| | [external_led_blink.py](examples/raspberrypi-pico/python/external_led_blink.py) | RaspberryPi Pico | MicroPython | Standard MicroPython |
| **RGB color strip**| [rgb_color_strip_patterns.py](examples/raspberrypi-pico/python/rgb_color_strip_patterns.py) | RaspberryPi Pico | MicroPython | [`adafruit_circuitpython_neopixel`](https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel)|
| **Motion sensor**| [motion_sensor.py](examples/raspberrypi-pico/python/motion_sensor.py) | RaspberryPi Pico | MicroPython | Standard MicroPython |
| | [passive_ir_sensor.py](examples/raspberrypi-pico/python/passive_ir_sensor.py) | RaspberryPi Pico | MicroPython | Standard MicroPython |
| | [security_sys.py](examples/raspberrypi-pico/python/security_sys.py) | RaspberryPi Pico | MicroPython | Standard MicroPython |
| **Ultrasonic sensor**| [lcd_ultrasonic.py](examples/raspberrypi-pico/python/lcd_ultrasonic.py) | RaspberryPi Pico | MicroPython | [`picozero`](https://picozero.readthedocs.io/en/latest/), [`pico_i2c_lcd`](examples/raspberrypi-pico/python/pico_i2c_lcd.py) |

## Apps for Heltec boards

| Component  | File  |  Board | Language | Necessary software |
| :---- | :---- | :---  | :---- | :--- |
|  **Display** | [hello_world_micropy](examples/heltec-wireless-tracker-v1.1/python/display_hello_world.micropy.py)  | Heltec Wireless Tracker    |    MicroPython      | [MicroPython-ST7735](https://github.com/boochow/MicroPython-ST7735), [sysfont.py](https://github.com/GuyCarver/MicroPython/blob/master/lib/sysfont.py) |
| | [hello_world_micropy](examples/heltec-wifi-lora-v3/python/display_hello_world.micropy.py) | Heltec Wifi LoRa V3 | MicroPython | [`ssd1306`](https://github.com/stlehmann/micropython-ssd1306) (_Install through Thonny_) |
| | [hello_world_circuitpy](examples/heltec-wifi-lora-v3/python/display_hello_world.circuitpy.py) | Heltec Wifi LoRa V3 | CircuitPython | [`adafruit-circuitpython-display-text`](https://pypi.org/project/adafruit-circuitpython-display-text/) |
|  | [display-demo](examples/heltec-wireless-tracker-v1.1/arduino/display-demo/)| Both boards | Arduino | [heltec-unofficial](https://github.com/ropg/heltec_esp32_lora_v3/) or [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) [(Installation)](https://github.com/ropg/heltec_esp32_lora_v3/tree/main?tab=readme-ov-file#1-install-this-library) |
|  | [simple-demo](examples/heltec-wireless-tracker-v1.1/arduino/simple-demo/)| Both boards | Arduino | [Heltec_ESP32](https://github.com/HelTecAutomation/Heltec_ESP32) or [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) [(Installation)](https://github.com/HelTecAutomation/Heltec_ESP32?tab=readme-ov-file#how-to-install-this-library)|
| | | | | |
| **Wifi** | [wifi_scan.py](examples/heltec-wireless-tracker-v1.1/python/wifi_scan.micropy.py) | Heltec Wireless Tracker | MicroPython | Standard MicroPython Build |
| | [wifi_ap.py](examples/heltec-wireless-tracker-v1.1/python/wifi_ap.micropy.py) | Heltec Wireless Tracker | MicroPython | Standard MicroPython Build |
| | [wifi_download.py](examples/heltec-wireless-tracker-v1.1/python/wifi_download.micropy.py) | Heltec Wireless Tracker | MicroPython | Standard MicroPython Build |
| | [wifi_packet_monitor.py](examples/heltec-wifi-lora-v3/python/wifi_packer_monitor.circuitpy.py) | Heltec Wifi LoRa V3 | CircuitPython | Standard CircuitPython Build |
| | [wifi_packet_sniffer.py](examples/heltec-wifi-lora-v3/python/wifi_packet_sniffer.circuitpy.py) | Heltec Wifi LoRa V3 | CircuitPython | [**Special CircuitPython Build**](firmware/heltec-wifi-lora-v3.special_circuitpy.firmware.bin) |
| | [wifi-scan](examples/heltec-wifi-lora-v3/arduino/WiFiScan/) | Heltec Wifi LoRa V3 | Arduino | [Official Arduino wifi API](https://docs.arduino.cc/libraries/wifi/) |
| | [wifi-client-connect](examples/heltec-wifi-lora-v3/arduino/WiFiClientConnect/) | Heltec Wifi LoRa V3 | Arduino | [Official Arduino wifi API](https://docs.arduino.cc/libraries/wifi/) |
| | | | | |
| **LoRa** | [lora_sender.py](examples/heltec-wireless-tracker-v1.1/python/lora_sender.micropy.py) | Both boards | MicroPython | [micropysx1262x](https://github.com/ehong-tl/micropySX126X) (Requires [license](https://docs.heltec.org/general/how_to_use_license.html))|
| | [lora_receiver.py](examples/heltec-wireless-tracker-v1.1/python/lora_receiver.micropy.py) | Both boards | MicroPython |[micropysx1262x](https://github.com/ehong-tl/micropySX126X) (Requires [license](https://docs.heltec.org/general/how_to_use_license.html))|
| | [lora-sender](examples/heltec-wireless-tracker-v1.1/arduino/lora-sender/)| Both boards | Arduino | [Heltec_ESP32](https://github.com/HelTecAutomation/Heltec_ESP32), [SX126x-Arduino](https://github.com/beegee-tokyo/SX126x-Arduino/), [(Installation)](https://github.com/HelTecAutomation/Heltec_ESP32?tab=readme-ov-file#how-to-install-this-library) (Requires [license](https://docs.heltec.org/general/how_to_use_license.html)) |
| | [lora-receiver](examples/heltec-wireless-tracker-v1.1/arduino/lora-receiver/)| Both boards | Arduino | [Heltec_ESP32](https://github.com/HelTecAutomation/Heltec_ESP32), [SX126x-Arduino](https://github.com/beegee-tokyo/SX126x-Arduino/), [(Installation)](https://github.com/HelTecAutomation/Heltec_ESP32?tab=readme-ov-file#how-to-install-this-library) (Requires [license](https://docs.heltec.org/general/how_to_use_license.html))|
| | | | | |
| **BLE** | [ble_connect_and_ad.py](examples/heltec-wifi-lora-v3/python/ble_connect_and_ad.circuitpy.py)| Heltec Wifi LoRa V3| CircuitPython | [`adafruit-circuitpython-ble-adafruit`](https://pypi.org/project/adafruit-circuitpython-ble-adafruit/) |
| | [ble-scanner](examples/heltec-wifi-lora-v3/arduino/Beacon_Scanner/) | Heltec Wifi LoRa V3 | Arduino | [Official Arduino ESP32 BLE library](https://docs.arduino.cc/libraries/esp32-ble-arduino/) |
| | [ble-client](examples/heltec-wifi-lora-v3/arduino/Client/) | Both boards | Arduino | [Official Arduino ESP32 BLE library](https://docs.arduino.cc/libraries/esp32-ble-arduino/) |
| | | | | |
| **GPS** | [gps-basic](examples/heltec-wireless-tracker-v1.1/python/gps_basic.micropy.py) | Heltec Wireless Tracker | MicroPython | Standard MicroPython Build |
| | [gps-message-parser](examples/heltec-wireless-tracker-v1.1/python/gps_parser.micropy.py) | Heltec Wireless Tracker | MicroPython | Copy [micropyGPS.py](https://github.com/inmcm/micropyGPS) onto device |
| | [gps-test](examples/heltec-wireless-tracker-v1.1/arduino/GPSDisplayOnTFT/) | Heltec Wireless Tracker | Arduino | [Heltec_ESP32](https://github.com/HelTecAutomation/Heltec_ESP32) |


# Using CodeMetal Hackathon-transpiler

For the purpose of the hackathon, we have deployed our MicroPython to Arduino C SDK and CircuitPython to Arduino C SDK pipelines in cloud. To connect with these pipelines, we have developed a command line based tool that feeds MicroPython or CircuitPython code to the pipeline and fetches corresponding Arduino C code for them. Below we show sample usage of this tool named [`micropy2c.py`](tools/micropy2c.py).

## Usage

```
$ python tools/micropy2c.py -h
usage: micropy2c [-h] [-b {heltec-wireless-tracker,heltec-wifi-lora-v3,raspberrypi-pico}]
                 [-s SOURCE_FILE] [-d SOURCE_DIR] [-o OUTPUT_DIR]
                 [-l {micropython,circuitpython}] [-m HOST] [-n PORT]
                 [-u USER] [-p PASSWD] [-v]

Translate MicroPython or CircuitPython program(s) to to Arduino C SDK for ESP32 boards from Heltec Automation and Raspberry Pi Pico boards.

options:
  -h, --help            show this help message and exit
  -b, --board {heltec-wireless-tracker,heltec-wifi-lora-v3,raspberrypi-pico}
                        Heltec board for which to generate Arduino C code
  -s, --source-file SOURCE_FILE
                        Path to Micro/Circuit Python program file
  -d, --source-dir SOURCE_DIR
                        Input is a directory containing Python source files
  -o, --output-dir OUTPUT_DIR
                        Directory to store generated Arduino C files          [Default: /tmp/out]
  -l, --source-lang {micropython,circuitpython}
                        Language of Python program                            [Default: micropython]
  -m HOST, --host HOST  Translation API host                                  [Default: https://localhost]
  -n PORT, --port PORT  Translation API port                                  [Default: 443]
  -u USER, --user USER  Username for translation API
  -p PASSWD, --passwd PASSWD Password for translation API
  -v, --verbose         Prints response details.                              [Default: False]
```

## Sample examples

### Transpiling single Python program

Below we demonstrate how to transpile single CircuitPython program into Arduino C programs.
First, we specify the board that we want to obtain Arduino C code for. Then we specify an output directory to store generated C code.
We also specify the CircuitPython program that we want to transpile. The transpilation request is sent to CodeMetal transpiler deployed at IP address `http://dummy.url` and port `8080`.

```
python tools/micropy2c.py \
-b heltec-wifi-lora-v3 \
-o /tmp/heltec_wifi_lora_v3 \
-l circuitpython \
-s examples/heltec-wifi-lora-v3/python/display_hello_world.circuitpy.py \
-m http://dummy.url -n 8080
```

Directory `/tmp/heltec_wifi_lora_v3` should contain `display_hello_world.circuitpy/display_hello_world.circuitpy.ino` containing Arduino C code. This code can be compiled using following command:

```
arduino-cli compile \
-b esp32:esp32:heltec_wifi_lora_v3 \
/tmp/heltec_wifi_lora_v3/display_hello_world.circuitpy
```

For Raspberry Pi Pico, use appropriate board name out of: `rp2040:rp2040:rpipico`, `rp2040:rp2040:rpipicow`, `rp2040:rp2040:rpipico2`, `rp2040:rp2040:rpipico2w` with `arduino-cli`.

NOTE: You may see some compilation errors if necessary Arduino boards/libraries are not installed.

### Batch transpilation of Python programs

Below we demonstrate how to transpile all Python programs from a directory into corresponding Arduino C programs.
Only additional option that we provide is `-d` and the input then is `examples/raspberrypi-pico/python`, which is
a directory containing Python programs.

```
python tools/micropy2c.py \
-o /tmp/raspberrypi_pico \
-b raspberrypi-pico \
-d examples/raspberrypi-pico/python \
-m http://dummy.url -n 8080
```

Similar to the output of transpiling single Python program, the output of batch transpilation will be a bunch of directories under `/tmp/raspberrypi_pico`. You can follow steps mentioned above to compile them individually.

# FAQs

- What is this LoRa library called CMLoRa and why do we need it?

There are two Arduino libraries for LoRa on ESP32: Heltec Automation's LoRa library (part of [Heltec_ESP32](https://github.com/HelTecAutomation/Heltec_ESP32)) and the popular [arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa) library. Heltec's library is pre-compiled and installed automatically when Heltec's Arduino board is installed. The other LoRa library can be installed also but that leads to a problem.

Unfortunately, both the libraries use same header file (e.g., [LoRa.h](https://github.com/sandeepmistry/arduino-LoRa/blob/master/src/LoRa.h) and [LoRa.h](https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/src/lora/LoRa.h)) and API names (which leads to a symbol name conflict) but their LoRa APIs differ (in terms of arguments). This prevents coexistance of both the libraries. With [CMLoRa](https://github.com/nhasabnic/arduino-LoRa) library, we have forked the popular `arduino-LoRa` library and modified its header file to prevent the symbol name conflict. And that allows us to install both `Heltec_ESP32` and `CMLoRa` libraries at the same time.

# Disclaimer

THIS REPOSITORY CONTAINS DEMONSTRATIVE MICROPYTHON, CIRCUITPYTHON, AND ARDUINO C PROGRAMS DEVELOPED OR COLLECTED FOR THE PURPOSE OF HACKATHONS. THEY DO NOT REPRESENT CODE METAL PLATFORMS OR PRODUCTS.
