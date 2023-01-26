# Drawer_Controller

Repository for the microcontroller that manages to lock and unlock the drawer.


Overview for the included libraries, the corresponding license and a link to the repository:
| Library | Licence | Description | Repository
| --- | --- | --- | --- |
| acan2515 | MIT | ACAN2515 is a driver for the MCP2515 CAN Controller. It runs on any Arduino compatible board. You can choose any frequency for your MCP2515, the actual frequency is a parameter of the library. The driver supports many bit rates: for a 16 MHz quartz, the CAN bit timing calculator finds settings for standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, 1 Mbit/s, but also for an exotic bit rate as 727 kbit/s. If the desired bit rate cannot be achieved, the begin method does not configure the hardware and returns an error code. | https://github.com/pierremolinaro/acan2515?utm_source=platformio&utm_medium=piohome | 
| FastLED | MIT | This is a library for easily & efficiently controlling a wide variety of LED chipsets, like the ones sold by adafruit (Neopixel, DotStar, LPD8806), Sparkfun (WS2801), and aliexpress. In addition to writing to the leds, this library also includes a number of functions for high-performing 8bit math for manipulating your RGB values, as well as low level classes for abstracting out access to pins and SPI hardware, while still keeping things as fast as possible. Tested with Arduino up to 1.6.5 from arduino.cc. | https://github.com/FastLED/FastLED?utm_source=platformio&utm_medium=piohome |
