// Libraries
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BasicLinearAlgebra.h>
#include <Fonts/Picopixel.h>


// BMP-280
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C initialization
#define localForecast 1018.6257 // local atmospheric pressure in Pa / 100
float startHeight; // starting altitude

// MPU9250
#define MAGDEC -4.22 // magnetic declination
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // IMU I2C address
MPU9250 IMU; // IMU object initialization
float ax, ay, az, gx, gy, gz, mx, my, mz; // accel, gyro, mag
float aN, aE, aU; // accel NEU frame of reference
float magOffset_x, magOffset_y, magOffset_z, magScale_x, magScale_y, magScale_z; // mag calibrations

// OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED button
#define inPin 30 // the number of the input pin for OLED button
int buttonStatus; // OLED button status (pressed, not pressed, etc)
#define MAPTOP 48.994636 // GPS map top latitude
#define MAPBOT 25.155229 // GPS map bottom latitude
#define MAPLEFT -124.707532 // GPS map left longitude
#define MAPRIGHT -66.996464 // GPS map right longitude

// OLED 3D Graphic
float tx, nx, p;
float ty, ny, py;
float rot, rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
int i; //0 to 360
int fl, scale; //focal length
int wireframe[12][2];

int originx = 96;
int originy = 32; //32

int front_depth = 20;
int back_depth = -20;

//Store cube vertices
int cube_vertex[8][3] = {
 { -10, -5, front_depth},
 {10, -5, front_depth},
 {10, 5, front_depth},
 { -10, 5, front_depth},
 { -10, -5, back_depth},
 {10, -5, back_depth},
 {10, 5, back_depth},
 { -10, 5, back_depth}
};

int fd = 0; //0=orthographic

// GPS
static const int RXPin = 0, TXPin = 1; // pin numbers
static const uint32_t GPSBaud = 19200; // baud rate
bool gpsFix = false; // gps fix bool
float latitude; // latitude variable
float longitude; // longitude variable
TinyGPSPlus gps; // initialize GPS object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

// Teensy
const int chipSelect = BUILTIN_SDCARD; // SD card

// Program constants
#define RAD2DEG 57.2958 // radians to degrees
#define DEG2RAD 0.0174533 // degrees to radians
#define g 9.80665 // gravity constant

// I2C
#define I2Cclock 400000
#define I2Cport Wire

// GUI variables
const char *mainMenuOptions[] = {"IMU/BMP", "skyMap", "Magnet", "Calibrate", "AHRS"};
const char *screens[] = {"mainMenu", "IMU/BMP", "skyMap", "Magnet", "Calibrate", "AHRS"};
String currentScreen;
int currentScreenIndex = 0;
int maxSelections;

// SEDS logo image map
const unsigned char SEDSLogo [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf8,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xf0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xe0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xc0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xc0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xfe, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xfc, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe7, 0xf8, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x00, 0x0f, 0x8f, 0xf8, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xfc, 0x00, 0x3f, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xc6, 0x0f, 0xf0, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xf0, 0x0f, 0xe0, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xf8, 0x1f, 0xe0, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0xc1, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xfc, 0x1f, 0xc0, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0xc1, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xfc, 0x3f, 0x80, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xfc, 0xcf, 0xff, 0xff, 0xfe, 0x7f, 0x80, 0x00,
  0x00, 0x01, 0xff, 0x00, 0xff, 0xc3, 0xff, 0x00, 0x00, 0xcf, 0xf8, 0x0f, 0xfe, 0x7f, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x7f, 0xc3, 0xfe, 0x00, 0x00, 0xcf, 0xf8, 0x03, 0xfe, 0x7f, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xc3, 0xfe, 0x00, 0x3f, 0xcf, 0xf8, 0x03, 0xfe, 0x7e, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xc3, 0xfe, 0x00, 0xff, 0xcf, 0xf8, 0x01, 0xfe, 0x7c, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xc3, 0xfe, 0x03, 0xff, 0xcf, 0xf8, 0x01, 0xfe, 0x7c, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xc3, 0xff, 0x00, 0x3f, 0xcf, 0xf8, 0x01, 0xfe, 0x78, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xc3, 0xff, 0xff, 0xc3, 0xcf, 0xf8, 0x01, 0xfe, 0x78, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xc3, 0xff, 0xff, 0xf9, 0xcf, 0xf8, 0x03, 0xfe, 0x70, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x7f, 0xc1, 0xff, 0xff, 0xfc, 0x8f, 0xf8, 0x03, 0xfe, 0xe0, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x81, 0xff, 0xc1, 0xff, 0xff, 0xfe, 0x0f, 0xf8, 0x0f, 0xfe, 0xe0, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfe, 0xc0, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xfc, 0x80, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0x82, 0x3f, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xfd, 0x80, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0x0f, 0x83, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xf9, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xfe, 0x3f, 0xf8, 0x01, 0xff, 0x0f, 0xff, 0xff, 0xf3, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xf8, 0xff, 0xff, 0x80, 0xff, 0x0f, 0xff, 0xff, 0xc6, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x01, 0xff, 0xfe, 0x00, 0xff, 0x0f, 0xf8, 0x00, 0x1c, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x0f, 0xff, 0xf8, 0x00, 0xff, 0x0f, 0xf8, 0x00, 0x7c, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0xff, 0x0f, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x00, 0x7f, 0xfc, 0x00, 0x01, 0xff, 0x0f, 0xf8, 0x00, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x01, 0xff, 0xf9, 0xfc, 0x0f, 0xff, 0x0f, 0xf8, 0x01, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x07, 0xff, 0xe1, 0xff, 0xff, 0xff, 0x0f, 0xf8, 0x01, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x1f, 0xff, 0x01, 0xff, 0xff, 0xfe, 0x0f, 0xf8, 0x01, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x3f, 0xfc, 0x01, 0xff, 0xff, 0xfe, 0x0f, 0xf8, 0x03, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x3f, 0xf0, 0x01, 0xff, 0xff, 0xfc, 0x0f, 0xf8, 0x03, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x3f, 0x80, 0x01, 0xff, 0xff, 0xf8, 0x0f, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0x3e, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x0f, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// USA image map
const unsigned char usamap [] PROGMEM = {
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x83, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3c,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x7c,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x7c,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0xfe,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xf0, 0x00, 0x03, 0xff, 0xff,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xf0, 0x00, 0x0f, 0xff, 0xf0,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xe8, 0x00, 0x0f, 0xff, 0xc0,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xf8, 0x00, 0x1f, 0xff, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xfc, 0x07, 0xff, 0xff, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xf8, 0x07, 0xff, 0xfe, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xf0, 0x3f, 0xff, 0xff, 0x00,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0xfc, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00,
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe8, 0x00, 0x00,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xec, 0x00, 0x00,
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xec, 0x00, 0x00,
  0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00,
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00,
  0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xf4, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0xff, 0xc0, 0x18, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00
};

// Purdue logo image map
const unsigned char purduelogo [] PROGMEM = {
  0xe0, 0x02, 0xe0, 0x00, 0xef, 0xfc, 0xef, 0xfc, 0xc7, 0x1c, 0xc7, 0x1c, 0xe7, 0x18, 0xe7, 0xfa,
  0xef, 0xf2, 0x8e, 0x06, 0x8e, 0x0e, 0xbf, 0x7e, 0xbe, 0x7e, 0x00, 0x7e, 0x00, 0xfe
};

unsigned long currentTime = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};              // vector to hold quaternion

// skyMap variable inialization
static const int MAX_SATELLITES = 40;
TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

long GPSRefreshPeriod = 0;
long lastGPSRefresh = 0;

const unsigned char UBLOX_INIT[] PROGMEM = {
  // Rate (pick one)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xFA,0x00,0x01,0x00,0x01,0x00,0x10,0x96, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

  // Disable specific NMEA sentences
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x5C, // GxZDA off
};

// Linear algebra 
using namespace BLA;

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

void setup() {
  Serial.begin(19200); // begin serial
  ss.begin(GPSBaud); // start gps
  for(unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    ss.write( pgm_read_byte(UBLOX_INIT+i) );
  }
  Wire.begin();
  pinMode(inPin, INPUT); // button pin

  // Show SEDS logo and credits
  OLEDIntro();

  // Initialize hardware
  initializeMPU9250();
  initializeBMP280();
  initializeGPS();

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  readMagCalValues();

  maxSelections = sizeof(mainMenuOptions) / 4; // for some reason it returns actual size*4 (bits?)
  for (int i = 1; i < maxSelections; i++) {
    screens[i] = mainMenuOptions[i - 1];
  }

  calibrateBMP280();
}

float compassHeading;
void loop() {
  // run time
  currentTime = millis();

  // read IMU, barometer
  readSensorData();

  // quaternion values
  q[0] = IMU.getQuaternionW();
  q[1] = IMU.getQuaternionX();
  q[2] = IMU.getQuaternionY();
  q[3] = IMU.getQuaternionZ();

  // calculate acceleration in NEU frame of reference
  calculateInertialAccel();

  // convert yaw to 360 degree heading
  compassHeading = IMU.getYaw();
  if (compassHeading < 0) compassHeading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
  if (compassHeading < 0) compassHeading += 360.0;                        // Allow for under|overflow
  if (compassHeading >= 360) compassHeading -= 360.0;

  // read GPS
  gpsRead();

  // OLED display
  displayMain();

  // check if GPS is operational
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

void displayMain() {
  currentScreen = screens[currentScreenIndex];
  if (currentScreen == "mainMenu") {
    mainMenu();
  } else if (currentScreen == "IMU/BMP") {
    OLEDraws();
  } else if (currentScreen == "skyMap") {
    displayGPSInfo();
  } else if (currentScreen == "Magnet") {
    showMagnetometer();
  } else if (currentScreen == "AHRS") {
    displayAHRS();
  } else if (currentScreen == "Calibrate") {
    calibrateMagnetometer();
  }

}

int currentSelection = 1;
const int maxSelectionsOnScreen = 4;
long lastTime = -1;
void mainMenu() {
  checkButton();
  if (buttonStatus == 1) {
    currentSelection++;
    if (currentSelection > maxSelections) {
      currentSelection = 1;
    }
  }
  if (buttonStatus == 2) {
    currentScreenIndex = currentSelection;
  }
  int buttonSpacing = 10;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.write("Main Menu");
  display.setTextSize(1);
  for (int i = 0; i < maxSelectionsOnScreen; i++) {
    display.setCursor(0, 16 + buttonSpacing * (i));
    int j = i + (currentSelection - maxSelectionsOnScreen);
    if (i > (maxSelections - 1) || j > (maxSelections - 1)) {
      break;
    }
    if (currentSelection > (maxSelectionsOnScreen)) {
      display.write(mainMenuOptions[j]);
    } else {
      display.write(mainMenuOptions[i]);
    }
  }

  // draw triangle next to current selection
  int arrowCenterY = 0;
  String currentSelectionString = mainMenuOptions[currentSelection - 1];
  int pixelsPerChar = 6;
  int charHeightPixels = 8;
  int arrowSpacer = 2;
  int stringLengthPixels = currentSelectionString.length() * pixelsPerChar;
  if (currentSelection > maxSelectionsOnScreen) {
    arrowCenterY = 16 + buttonSpacing * (maxSelectionsOnScreen - 1) + (charHeightPixels / 2);
  } else {
    arrowCenterY = 16 + buttonSpacing * (currentSelection - 1) + (charHeightPixels / 2);
  }
  display.drawLine(stringLengthPixels + arrowSpacer, arrowCenterY, stringLengthPixels + arrowSpacer + buttonSpacing / 4, arrowCenterY + buttonSpacing / 4, WHITE);
  display.drawLine(stringLengthPixels + arrowSpacer, arrowCenterY, stringLengthPixels + arrowSpacer + buttonSpacing / 4, arrowCenterY - buttonSpacing / 4, WHITE);

  // draw flashing triangle at bottom of selection screen'
  if (maxSelections > maxSelectionsOnScreen) {
    if (currentSelection != maxSelections) {
      arrowCenterY = 16 + buttonSpacing * maxSelectionsOnScreen; // triangle center y
      if (currentTime - lastTime > 500) { // make triangle flash
        display.fillTriangle(10, arrowCenterY, 16, arrowCenterY, 13, arrowCenterY + 5, WHITE); // fill
        lastTime = currentTime;
      } else {
        display.drawTriangle(10, arrowCenterY, 16, arrowCenterY, 13, arrowCenterY + 5, WHITE); // unfill
      }
    }
  }
  display.display();
}

const int holdTime = 500;
long pressTime = 0;
long timeHeld = 0;
long lastPress = 0;
const int minPressTime = 25;
int previous;
int checkButton() {
  buttonStatus = 0;
  int reading = digitalRead(inPin);
  //Serial.println(reading);
  Serial.println(reading);
  if (reading == HIGH && previous == HIGH) {
    // button pressed
    if (timeHeld == 0) {
      pressTime = millis();
      timeHeld++;
    } else {
      timeHeld = millis() - pressTime;
    }
    previous = reading;
  } else if (reading == LOW && previous == HIGH) {
    if (timeHeld < minPressTime) {
      // ghost press
    } else if (timeHeld > holdTime) {
      // long press
      buttonStatus = 2;
      currentScreenIndex = currentSelection;
    } else {
      // short press
      buttonStatus = 1;
    }
    timeHeld = 0;
  }
  previous = reading;
}

void readSensorData() {
  IMU.update();

  ax = IMU.getAccX();
  ay = IMU.getAccY();
  az = IMU.getAccZ();

  gx = IMU.getGyroX();
  gy = IMU.getGyroY();
  gz = IMU.getGyroZ();

  mx = IMU.getMagX();
  my = IMU.getMagY();
  mz = IMU.getMagZ();
}

static void gpsRead()
{
  while (ss.available())
    gps.encode(ss.read());
  if (totalGPGSVMessages.isUpdated() && currentScreen == "skyMap")// add screen
  {
    for (int i = 0; i < 4; ++i)
    {
      int no = atoi(satNumber[i].value());
      // Serial.print(F("SatNumber is ")); Serial.println(no);
      if (no >= 1 && no <= MAX_SATELLITES)
      {
        sats[no - 1].elevation = atoi(elevation[i].value());
        sats[no - 1].azimuth = atoi(azimuth[i].value());
        sats[no - 1].snr = atoi(snr[i].value());
        sats[no - 1].active = true;
      }
    }
  }
  if (gps.location.isValid()) {
    GPSRefreshPeriod = millis() - lastGPSRefresh;
    lastGPSRefresh = millis();
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsFix = true;
  } else {
    gpsFix = false;
  }
}

void initializeGPS() {
  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i = 0; i < 4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }
}

void OLEDIntro() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.drawBitmap(0, 0, SEDSLogo, 128, 64, WHITE);
  display.display();

  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.write("2021 v1 Avionics");
  display.setCursor(0, 8);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.write("Developed by the");
  display.setCursor(0, 16);
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.write("Hybrids");
  display.setCursor(0, 31);
  display.write("Avionics");
  display.setCursor(0, 46);
  display.write("Team");
  display.setCursor(50, 53);
  display.setTextSize(1);
  display.write("@ Purdue Univ");
  display.display();
  delay(10);
}

int skyMapRadius = 18;
int skyMapCenterX = 94;
int skyMapCenterY = 44;
void skyMap() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.write("GPSInfo");
  display.setCursor(0, 8);
  display.write("skyMap");
  display.setTextSize(1);
  display.drawBitmap(43, 0, purduelogo, 15, 15, WHITE);
  display.setCursor(0, 20);
  display.write("Sats:");
  display.setCursor(0, 28);
  display.write("HDOP:");
  display.setCursor(0, 36);
  display.write("Age:");
  display.setCursor(0, 44);
  display.write("h:");
  display.setCursor(30, 44);
  display.write("m");
  display.setCursor(0, 52);
  display.write("v:");
  display.setCursor(30, 52);
  display.write("kph");
  display.drawLine(skyMapCenterX - skyMapRadius, skyMapCenterY, skyMapCenterX + skyMapRadius, skyMapCenterY, SSD1306_WHITE);
  display.drawLine(skyMapCenterX, skyMapCenterY - skyMapRadius, skyMapCenterX, skyMapCenterY + skyMapRadius, SSD1306_WHITE);
  display.drawCircle(skyMapCenterX, skyMapCenterY, skyMapRadius, WHITE);
  display.setCursor(skyMapCenterX - skyMapRadius - 8, skyMapCenterY - 3);
  display.write("W");
  display.setCursor(skyMapCenterX - 2, skyMapCenterY - skyMapRadius - 8);
  display.write("N");
  if (gpsFix) {
    display.setCursor(60, 0);
    OLEDwritefloat(abs(latitude), 3, 3, 8);
    display.setCursor(105, 0);
    display.drawCircle(98, 1, 1, WHITE);
    display.write("N");
    display.setCursor(60, 8);
    OLEDwritefloat(abs(longitude), 3, 3, 8);
    display.setCursor(105, 8);
    display.drawCircle(98, 8, 1, WHITE);
    display.write("W");
    display.setCursor(25, 20);
    OLEDwritefloat(gps.satellites.value(), 2, 0, 2);
    display.setCursor(30, 28);
    OLEDwritefloat(gps.hdop.hdop(), 2, 0, 2);
    display.setCursor(50, 25);
    OLEDwritefloat(1/(GPSRefreshPeriod / 1000), 2, 0, 2);
    display.setCursor(25, 36);
    OLEDwritefloat(gps.location.age(), 2, 0, 2);
    display.setCursor(10, 44);
    OLEDwritefloat(gps.altitude.meters(), 3, 0, 3);
    display.setCursor(10, 52);
    OLEDwritefloat(gps.speed.kmph(), 2, 0, 2);
    for (int i = 0; i < MAX_SATELLITES; ++i)
      if (sats[i].active)
      {
        drawSatellite(sats[i].azimuth, sats[i].elevation, i + 1);
      }
  } else {
    display.setCursor(60, 0);
    display.write("No GPS Fix");
    display.setCursor(30, 20);
    display.write("--");
    display.setCursor(30, 28);
    display.write("--");
    display.setCursor(25, 36);
    display.write("--");
    display.setCursor(12, 44);
    display.write("--");
    display.setCursor(12, 52);
    display.write("--");
  }

  display.display();
}

int screenIndexGPS = 0;
void displayGPSInfo() {
  checkButton();
  const int numScreens = 2;
  if (buttonStatus == 1) {
    screenIndexGPS++;
    if (screenIndexGPS > (numScreens - 1)) {
      screenIndexGPS = 0;
    }
  }
  if (buttonStatus == 2) {
    currentScreenIndex = 0;
  }
  if (screenIndexGPS == 0) {
    skyMap();
  } else if (screenIndexGPS == 1) {
    displayOLEDGeoLoc();
  }
}

void draw_vertices(void)
{
 display.drawPixel (rotxxx, rotyyy,WHITE);
}

void draw_wireframe(void)
{
 display.drawLine(wireframe[0][0], wireframe[0][1], wireframe[1][0], wireframe[1][1],WHITE);
 display.drawLine(wireframe[1][0], wireframe[1][1], wireframe[2][0], wireframe[2][1],WHITE);
 display.drawLine(wireframe[2][0], wireframe[2][1], wireframe[3][0], wireframe[3][1],WHITE);
 display.drawLine(wireframe[3][0], wireframe[3][1], wireframe[0][0], wireframe[0][1],WHITE);

//cross face above
 display.drawLine(wireframe[1][0], wireframe[1][1], wireframe[3][0], wireframe[3][1],WHITE);
 display.drawLine(wireframe[0][0], wireframe[0][1], wireframe[2][0], wireframe[2][1],WHITE);

 display.drawLine(wireframe[4][0], wireframe[4][1], wireframe[5][0], wireframe[5][1],WHITE);
 display.drawLine(wireframe[5][0], wireframe[5][1], wireframe[6][0], wireframe[6][1],WHITE);
 display.drawLine(wireframe[6][0], wireframe[6][1], wireframe[7][0], wireframe[7][1],WHITE);
 display.drawLine(wireframe[7][0], wireframe[7][1], wireframe[4][0], wireframe[4][1],WHITE);

 display.drawLine(wireframe[0][0], wireframe[0][1], wireframe[4][0], wireframe[4][1],WHITE);
 display.drawLine(wireframe[1][0], wireframe[1][1], wireframe[5][0], wireframe[5][1],WHITE);
 display.drawLine(wireframe[2][0], wireframe[2][1], wireframe[6][0], wireframe[6][1],WHITE);
 display.drawLine(wireframe[3][0], wireframe[3][1], wireframe[7][0], wireframe[7][1],WHITE);
}

float compassRadius = 16;
int centerCompassX = 26;
int centerCompassY = 36;
int centerPitchX = 26;
int centerPitchY = 36;
void displayAHRS() {
  checkButton();
  //Serial.println(buttonStatus);
  if (buttonStatus == 2) {
    currentScreenIndex = 0;
  }
  display.clearDisplay();
  float x = round(compassRadius * cos((compassHeading - 90) * DEG2RAD));
  float y = round(compassRadius * sin((compassHeading - 90) * DEG2RAD));
  display.drawLine(centerCompassX, centerCompassY, x + centerCompassX, y + centerCompassY, WHITE);
  display.setTextSize(1);
  display.setCursor(centerCompassX - 3, centerCompassY - compassRadius - 12);
  display.write("N");
  display.setCursor(centerCompassX + 8, centerCompassY - compassRadius - 12);
  OLEDwritefloat(abs(compassHeading), 3, 0, 3);
  display.setCursor(centerCompassX - 3, centerCompassY + compassRadius + 5);
  display.write("S");
  display.setCursor(centerCompassX + compassRadius + 5, centerCompassY);
  display.write("E");
  display.setCursor(centerCompassX - compassRadius - 10, centerCompassY);
  display.write("W");
  display.drawCircle(centerCompassX, centerCompassY, compassRadius + 3, WHITE);

   for (int i = 0; i < 8; i++) {
//rotateY
    rot = IMU.getEulerZ() * 0.0174532; //0.0174532 = one degree
    rotz = cube_vertex[i][2] * cos(rot) - cube_vertex[i][0] * sin(rot);
    rotx = cube_vertex[i][2] * sin(rot) + cube_vertex[i][0] * cos(rot);
    roty = cube_vertex[i][1];
//rotateX
    rot = 45 * 0.0174532; //0.0174532 = one degree
    rotyy = roty * cos(rot) - rotz * sin(rot);
    rotzz = roty * sin(rot) + rotz * cos(rot);
    rotxx = rotx;
////rotateZ
rotyyy = rotyy;
    rotzzz = rotzz;
    rotxxx = rotxx;
//    rot = -1*(IMU.getEulerX()) * 0.0174532; //0.0174532 = one degree
//    rotxxx = rotxx * cos(rot) - rotyy * sin(rot);
//    rotyyy = rotxx * sin(rot) + rotyy * cos(rot);
//    rotzzz = rotzz;


//orthographic projection
    rotxxx = rotxxx + originx;
    rotyyy = rotyyy + originy;

//store new vertices values for wireframe drawing
    wireframe[i][0] = rotxxx;
    wireframe[i][1] = rotyyy;
    wireframe[i][2] = rotzzz;

    draw_vertices();
   }
    
   draw_wireframe();
  
//  float maxG = 0.5;
//  int vectorLengthMax = 20;
//  float vectorLengthX = (aE/maxG)*vectorLengthMax;
//  float vectorLengthY = (aN/maxG)*vectorLengthMax;
//  Serial.println(vectorLengthX);
//  Serial.println(vectorLengthY);
//  display.drawPixel(100,32,WHITE);
//  display.drawLine(100,32,100+vectorLengthX,32+vectorLengthY, WHITE);

  display.setCursor(64,0);
  display.write("P: ");
  display.setCursor(70,0);
  OLEDwritefloat(IMU.getEulerX(), 4, 0, 4);
  display.setCursor(64,10);
  display.write("R: ");
  display.setCursor(75,10);
  OLEDwritefloat(IMU.getEulerY(), 4, 0, 4);
  display.setCursor(64,20);
  display.write("Y: ");
  display.setCursor(75,20);
  OLEDwritefloat(IMU.getEulerZ(), 4, 0, 4);
  
  display.display();
}

bool endCalibration = false;
void calibrateMagnetometer() {
  int centerY = 39;
  int radius = 18;
  int graphGap = 5;
  int XYcenterX = graphGap + radius;
  int XZcenterX = 3 * radius + 2 * graphGap;
  int YZcenterX = 5 * radius + 3 * graphGap;
  int sensorMaxValue = 1000;

  // accel/gyro calibration
  bool startCalibration = false;
  int selection = 2;
  while(startCalibration == false) {
    checkButton();
    if (buttonStatus == 2) {
      if(selection == 1) {
        startCalibration == true;
        break;
      } else {
        currentScreenIndex = 0;
        return;
      }
    }
    if(buttonStatus == 1) {
      selection++;
      if(selection > 2) {
        selection = 1;
      }
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.write("Gyro/Accel Calibrate");
    display.setCursor(0,15);
    display.setFont(&Picopixel);
    display.setTextSize(1);
    display.write("Set device on a flat surface with the Z-axis normal to the ground");
    display.setFont();
    display.setCursor(10,35);
    display.write("Start calibration?");
    if(selection == 1) {
    display.fillRect(50, 44, 9, 9, WHITE);
    display.setCursor(52,45);
    display.setTextColor(BLACK);
    display.write("Y");
    display.setTextColor(WHITE);
    display.setCursor(69,45);
    display.write("N");
    }
    display.setCursor(61,45);
    display.write("/");
    if(selection == 2) {
    display.fillRect(67, 44, 9, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(69,45);
    display.write("N");
    display.setTextColor(WHITE);
    display.setCursor(52,45);
    display.write("Y");
    }
    display.display();
    delay(20);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,20);
  display.write("Do not");
  display.setCursor(0,40);
  display.write("move");
  display.setCursor(0,60);
  display.write("device");
  display.setTextSize(1);
  display.display();
  delay(2500);
  IMU.calibrateAccelGyro();

  // magnetometer calibration
  startCalibration = false;
  selection = 2;
  while(startCalibration == false) {
    checkButton();
    if (buttonStatus == 2) {
      if(selection == 1) {
        startCalibration == true;
        break;
      } else {
        currentScreenIndex = 0;
        return;
      }
    }
    if(buttonStatus == 1) {
      selection++;
      if(selection > 2) {
        selection = 1;
      }
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.write("Magnet Calibrate");
    display.setCursor(0,15);
    display.setFont(&Picopixel);
    display.setTextSize(1);
    display.write("After starting, wave device in a figure-8 until all three plots are populated with enough data");
    display.setFont();
    display.setCursor(10,45);
    display.write("Start calibration?");
    if(selection == 1) {
    display.fillRect(50, 54, 9, 9, WHITE);
    display.setCursor(52,55);
    display.setTextColor(BLACK);
    display.write("Y");
    display.setTextColor(WHITE);
    display.setCursor(69,55);
    display.write("N");
    }
    display.setCursor(61,55);
    display.write("/");
    if(selection == 2) {
    display.fillRect(67, 54, 9, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(69,55);
    display.write("N");
    display.setTextColor(WHITE);
    display.setCursor(52,55);
    display.write("Y");
    }
    display.display();
    delay(20);
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.write("Rotate in figure-8,  press button to exit");
  
  // xy axis
  display.drawLine(XYcenterX, centerY - radius, XYcenterX, centerY + radius, WHITE);
  display.drawLine(XYcenterX - radius, centerY, XYcenterX + radius, centerY, WHITE);
  // xz axis
  display.drawLine(XZcenterX, centerY - radius, XZcenterX, centerY + radius, WHITE);
  display.drawLine(XZcenterX - radius, centerY, XZcenterX + radius, centerY, WHITE);
  // yz axis
  display.drawLine(YZcenterX, centerY - radius, YZcenterX, centerY + radius, WHITE);
  display.drawLine(YZcenterX - radius, centerY, YZcenterX + radius, centerY, WHITE);
  display.display();
  // get initial min/max values
  readSensorData();
  float minX = mx;
  float maxX = mx;
  float minY = my;
  float maxY = my;
  float minZ = mz;
  float maxZ = mz;

  IMU.setMagBias(0, 0, 0);
  IMU.setMagScale(1, 1, 1);

  while (endCalibration == false) {
    // check button
    checkButton();
    if (buttonStatus == 1) {
      endCalibration = true;
    }
    readSensorData();
    if (mx < minX) {
      minX = mx;
    }
    if (mx > maxX) {
      maxX = mx;
    }
    if (my < minY) {
      minY = my;
    }
    if (my > maxY) {
      maxY = my;
    }
    if (mz < minZ) {
      minZ = mz;
    }
    if (mz > maxZ) {
      maxZ = mz;
    }
    int x = radius * (mx / sensorMaxValue);
    int y = radius * (my / sensorMaxValue);
    int z = radius * (mz / sensorMaxValue);
    display.drawPixel(x + XYcenterX, centerY + y, WHITE);
    display.drawPixel(x + XZcenterX, centerY + z, WHITE);
    display.drawPixel(y + YZcenterX, centerY + z, WHITE);
    display.display();
  }
  // hard iron offset
  magOffset_x = (maxX + minX) / 2;
  magOffset_y = (maxY + minY) / 2;
  magOffset_z = (maxZ + minZ) / 2;

  // soft iron distortion
  float avg_delta_x = (maxX - minX) / 2;
  float avg_delta_y = (maxY - minY) / 2;
  float avg_delta_z = (maxZ - minZ) / 2;

  float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

  magScale_x = avg_delta / avg_delta_x;
  magScale_y = avg_delta / avg_delta_y;
  magScale_z = avg_delta / avg_delta_z;

  IMU.setMagBias(magOffset_x, magOffset_y, magOffset_z);
  IMU.setMagScale(magScale_x, magScale_y, magScale_z);

  SD.remove("magCal.txt");
  File dataFile = SD.open("magCal.txt", FILE_WRITE);
  String dataStringOffsets = String(magOffset_x) + " " + String(magOffset_y) + " " + String(magOffset_z);
  String dataStringScales = String(magScale_x) + " " + String(magScale_y) + " " + String(magScale_z);
  if (dataFile) {
    dataFile.println(dataStringOffsets + " " + dataStringScales + " ");
    dataFile.close();
  }
  currentScreenIndex = 0;
}

bool firstRun = true;
void showMagnetometer() {
  checkButton();
  if (buttonStatus == 2) {
    currentScreenIndex = 0;
  }
  int centerY = 39;
  int radius = 18;
  int graphGap = 5;
  int XYcenterX = graphGap + radius;
  int XZcenterX = 3 * radius + 2 * graphGap;
  int YZcenterX = 5 * radius + 3 * graphGap;
  int sensorMaxValue = 1000;
  if (firstRun == true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.write("Mag Plots");
    // xy axis
    display.drawLine(XYcenterX, centerY - radius, XYcenterX, centerY + radius, WHITE);
    display.drawLine(XYcenterX - radius, centerY, XYcenterX + radius, centerY, WHITE);
    // xz axis
    display.drawLine(XZcenterX, centerY - radius, XZcenterX, centerY + radius, WHITE);
    display.drawLine(XZcenterX - radius, centerY, XZcenterX + radius, centerY, WHITE);
    // yz axis
    display.drawLine(YZcenterX, centerY - radius, YZcenterX, centerY + radius, WHITE);
    display.drawLine(YZcenterX - radius, centerY, YZcenterX + radius, centerY, WHITE);
    display.display();
    firstRun = false;
  }

  readSensorData();
  int x = radius * (mx / sensorMaxValue);
  int y = radius * (my / sensorMaxValue);
  int z = radius * (mz / sensorMaxValue);
  display.drawPixel(x + XYcenterX, centerY + y, WHITE);
  display.drawPixel(x + XZcenterX, centerY + z, WHITE);
  display.drawPixel(y + YZcenterX, centerY + z, WHITE);
  display.display();
}

void readMagCalValues() {
  char temp[] = {};
  File dataFile = SD.open("magCal.txt", FILE_READ);
  if (dataFile) {
    int i = 0;
    int n = 0;
    while (dataFile.available()) {
      Serial.println("file found");
      const char currentChar = dataFile.read();
      const char charSpacer = ' ';
      if (currentChar != charSpacer || n > 5) {
        temp[i++] = currentChar;
      } else {
        memset(temp, 0, sizeof temp);
        switch (n++) {
          case 0:
            magOffset_x = atof(temp);
            break;
          case 1:
            magOffset_y = atof(temp);
          case 2:
            magOffset_z = atof(temp);
          case 3:
            magScale_x = atof(temp);
          case 4:
            magScale_y = atof(temp);
          case 5:
            magScale_z = atof(temp);
        }
        i = 0;
      }
    }
    dataFile.close();
  } else {
    Serial.println("Using default values");
    magOffset_x = 0;
    magOffset_y = 0;
    magOffset_z = 0;
    magScale_x = 1;
    magScale_y = 1;
    magScale_z = 1;
  }
  IMU.setMagBias(magOffset_x, magOffset_y, magOffset_z);
  IMU.setMagScale(magScale_x, magScale_y, magScale_z);
}

void drawSatellite(float azimuth, float elevation, int num) {
  azimuth = azimuth - 90;
  float r = (1 - (elevation / 90)) * skyMapRadius;
  float x = round(r * cos(azimuth * DEG2RAD));
  float y = round(r * sin(azimuth * DEG2RAD));
  display.drawCircle(x + skyMapCenterX, y + skyMapCenterY, 2, WHITE);
}

void OLEDraws() {
  checkButton();
  if (buttonStatus == 2) {
    currentScreenIndex = 0;
  }
  display.clearDisplay();
  display.setTextSize(1);

  readSensorData();
  // display 9250 accel
  display.setCursor(0, 0);
  display.write("9250");
  display.setCursor(0, 8);
  display.write("acc");
  display.setCursor(25, 20);
  display.write("g");
  display.setCursor(25, 30);
  display.write("g");
  display.setCursor(25, 40);
  display.write("g");
  display.setCursor(0, 20);
  OLEDwritefloat(abs(ax), 3, 2, 4);
  display.setCursor(0, 30);
  OLEDwritefloat(abs(ay), 3, 2, 4);
  display.setCursor(0, 40);
  OLEDwritefloat(abs(az), 3, 2, 4);

  // display 9250 gyro
  display.setCursor(35, 0);
  display.write("9250");
  display.setCursor(35, 8);
  display.write("gyro");
  display.setCursor(60, 20);
  display.write("d/s");
  display.setCursor(60, 30);
  display.write("d/s");
  display.setCursor(60, 40);
  display.write("d/s");
  display.setCursor(35, 20);
  OLEDwritefloat(abs(gx), 3, 2, 4);
  display.setCursor(35, 30);
  OLEDwritefloat(abs(gy), 3, 2, 4);
  display.setCursor(35, 40);
  OLEDwritefloat(abs(gz), 3, 2, 4);

  // display BMP280 data
  display.setCursor(85, 0);
  display.write("BMP280");
  display.setCursor(85, 8);
  display.write("T,P,h");
  display.setCursor(116, 20);
  display.write("C");
  display.drawCircle(112, 20, 2, WHITE);
  display.setCursor(110, 30);
  display.write("kPa");
  display.setCursor(122, 40);
  display.write("m");
  display.setCursor(85, 20);
  OLEDwritefloat(abs(bmp.readTemperature()), 3, 2, 4);
  display.setCursor(85, 30);
  OLEDwritefloat(abs(bmp.readPressure() / 1000.0f), 3, 2, 4);
  display.setCursor(85, 40);
  OLEDwritefloat(abs(bmp.readAltitude(localForecast) - startHeight), 3, 2, 6);
  //Serial.println(startHeight);
  display.display();
}

void displayOLEDGeoLoc() {
  float scaleVrt = (latitude - MAPBOT) / (MAPTOP - MAPBOT);
  float pixelVert = round(49 - scaleVrt * 49);

  float scaleHrz = (longitude - MAPLEFT) / (MAPLEFT - MAPRIGHT);
  float pixelHrz = abs(round(scaleHrz * 128));

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.write("GPS");
  display.setTextSize(1);
  if (gpsFix) {
    display.setCursor(60, 0);
    OLEDwritefloat(abs(latitude), 3, 3, 8);
    display.setCursor(105, 0);
    display.drawCircle(98, 1, 1, WHITE);
    display.write("N");
    display.setCursor(60, 8);
    OLEDwritefloat(abs(longitude), 3, 3, 8);
    display.setCursor(105, 8);
    display.drawCircle(98, 8, 1, WHITE);
    display.write("W");
  } else {
    display.setCursor(60, 0);
    display.setTextSize(1);
    display.write("No GPS Fix");
  }
  display.drawBitmap(0, 16, usamap, 128, 64 - 15, WHITE);
  int circleSize = updateLocCircleSize();
  display.drawCircle(pixelHrz, pixelVert + 15, circleSize, BLACK);
  display.drawBitmap(43, 0, purduelogo, 15, 15, WHITE);
  display.display();
}

long lastLoop = 0;
float timeSince, circleSize;
int oscillationTime = 1000;
int minCircleSize = 2;
int maxCircleSize = 4;
float updateLocCircleSize() {
  timeSince = millis() - lastLoop;
  circleSize = round(minCircleSize + maxCircleSize * abs(sin(2 * PI * ((timeSince) / oscillationTime))));
  if (timeSince > oscillationTime) {
    lastLoop = millis();
  }
  return circleSize;
}

void OLEDwritefloat(float f_val, int numbers, int decimals, int SigFigs) {
  static char outstr[15];
  dtostrf(f_val, numbers, decimals, outstr);
  display.write(outstr, SigFigs);
}

void initializeMPU9250() {
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if (!IMU.setup(0x68, setting)) { // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  IMU.setMagneticDeclination(MAGDEC);
  IMU.calibrateAccelGyro();
}

void initializeBMP280() {
  if (!bmp.begin()) {
    Serial.println("BMP280 connection failed.");
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);
    display.write("Error");
    display.setCursor(0, 20);
    display.setTextSize(2);
    display.write("BMP280");
    display.setCursor(0, 40);
    display.write("Not Found");
    display.display();
    //while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
}

void calibrateBMP280() {
  int calLoops = 1000;
  for (int i = 0; i < calLoops; i++) {
    startHeight += bmp.readAltitude(localForecast);
  };
  startHeight /= calLoops;
}

void calculateInertialAccel() {
  float s = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  float a11 = 1 - 2 * s * (q[2] * q[2] + q[3] * q[3]);
  float a12 = 2 * s * (q[1] * q[2] - q[0] * q[3]);
  float a13 = 2 * s * (q[1] * q[3] + q[0] * q[2]);
  float a21 = 2 * s * (q[1] * q[2] + q[0] * q[3]);
  float a22 = 1 - 2 * s * (q[1] * q[1] + q[3] * q[3]);
  float a23 = 2 * s * (q[2] * q[3] - q[0] * q[1]);
  float a31 = 2 * s * (q[1] * q[3] - q[0] * q[2]);
  float a32 = 2 * s * (q[2] * q[3] + q[0] * q[1]);
  float a33 = 1 - 2 * s * (q[1] * q[1] + q[2] * q[2]);

  aN = -a11 * ax + a12 * ay + a13 * az;
  aE = -a21 * ax + a22 * ay + a23 * az;
  aU = -a31 * ax + a32 * ay + a33 * az;
}
