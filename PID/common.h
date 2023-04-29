#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#ifdef DEBUG_DISPLAY
#include <U8g2lib.h>
#include <U8x8lib.h> 
#include "Adafruit_TCS34725.h"
#endif

#define TOF_NUMBER 5
#define TOF_START 0

#define MUXADDR 0x70
#define OLED_CLK 28
#define OLED_DATA 30

#ifdef FAKE_SERIAL
#define PI_SERIAL Serial
#else
#define PI_SERIAL Serial2
#endif

#ifdef DEBUG_DISPLAY
U8X8_SSD1306_128X64_NONAME_SW_I2C oled(OLED_CLK, OLED_DATA);
#endif

/* might need sensor id of 55, but I doubt it  */
Adafruit_BNO055 bno;

VL53L0X tof;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);


Motor motorL(MPORT2);
Motor motorR(MPORT1, true, true);

sensors_event_t accelerometerData, gyroData, orientationData, linearAccelData;

const float CM_TO_ENCODERS = 100.f/4.6f;
const float ENCODERS_TO_CM = 1.f/CM_TO_ENCODERS;

const int minspeed = 200;
const double KP_TURN = 1.2;
const double KI_TURN = 0.00003;
const double KD_TURN = 0.243;
const int DRIVE_BOOST = 20;
const int TURN_BOOST = 100;


const double DRIVE_STRAIGHT_KP = 3.0;
const double KP_FORWARD = 1.2;
const double KI_FORWARD = 0.003;
const double KD_FORWARD = 0.01;
const double SAMPLERATE_DELAY_MS = 10.0;
const double TIMES_PER_SECOND = 1000.0 / SAMPLERATE_DELAY_MS;
volatile double global_angle = 0;

const int SPEED = 100;

const double TOF_DISTANCE = 58.64;

enum type { Color,
            Distance };
enum direction { n,
                 e,
                 s,
                 w };
volatile uint8_t cur_direction = n;
double xPos = 0, yPos = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;

inline void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(MUXADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
