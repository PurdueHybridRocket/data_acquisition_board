# data_acquisition_board
Software and hardware associated with the hybrid rocket data acquisition board. Developed by Purdue Space Program's Hybrid Rocket for the HAVOC vehicle. 

Hardware:
The data acquisition board is built on a TEENSY 3.6 microcontroller which reads data from the following sensors:
BMP 280 temperature and pressure sensor
MPU 9250 9-axis inertial measurement unit (IMU)
GNSS Module
Tactile switch

And outputs the data to the following media:
microSD card
SSD1306 LCD Display

Software:
The software for the board is written in Arduino using the Arduino IDE and the Teensyduino plugin. The code running on the board makes use of the following libraries:
MPU9250.h             from https://github.com/hideakitai/MPU9250
Adafruit_BMP280.h     from https://github.com/adafruit/Adafruit_BMP280_Library
TinyGPS++.h           from https://github.com/mikalhart/TinyGPSPlus
SoftwareSerial.h      from https://www.arduino.cc/en/Reference/SoftwareSerial
SD.h                  from https://www.arduino.cc/en/Reference/SD
Adafruit_GFX.h        from https://github.com/adafruit/Adafruit-GFX-Library
Adafruit_SSD1306.h    from https://github.com/adafruit/Adafruit_SSD1306
BasicLinearAlgebra.h  from https://github.com/tomstewart89/BasicLinearAlgebra
Fonts/Picopixel.h     from https://github.com/adafruit/Adafruit-GFX-Library

