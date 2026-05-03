# MPD OLED Spectrum Display for Raspberry Pi

The mpd_oled program displays an information screen including a music
frequency spectrum on an OLED screen connected to a Raspberry Pi (or similar)
running MPD, this includes Moode, Volumio and rAudio (RuneAudio fork).
The program supports I2C and SPI 128x64 OLED displays with an SSD1306,
SSD1309, SH1106 or SSH1106 controller.
![OLED with mpd_oled](mpd_oled.jpg)

## Credits

C.A.V.A. is a bar spectrum audio visualizer: <https://github.com/karlstav/cava>

OLED interface based on ArduiPI_OLED: <https://github.com/hallard/ArduiPi_OLED>
(which is based on the Adafruit_SSD1306, Adafruit_GFX, and bcm2835 library
code).

C library for Broadcom BCM 2835: <https://www.airspayce.com/mikem/bcm2835/>
