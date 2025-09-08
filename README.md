# QFC

Microcontroller: ESP32-S3
IDE: Arduino IDE

1. Download ESP32 Board to the IDE
2. Downloas Necessaey libraries (we chanhed the content of PPM-reader's `.h` and `.cpp` files, so make sure to replace it. you can find it here: `Other/PPM-reader.zip`)

(Why lot of .ino files in one directory?)
For more details on the build process of Arduino IDE, see the official Arduino documentation: https://docs.arduino.cc/arduino-cli/sketch-build-process/

Unzip Other/PPM-reader.zip then add it to your Arduino/libraries/ directory (on Windows).

Don't use this GPIOs:
1) 35, 36 and 38 (they are used for the internal communication between ESP32-S3 and SPI Flash/PSRAM memory, thus not available for external use).
2) 19 and 20 (If you want to connect your board wit the USB port)