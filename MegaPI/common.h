#ifndef _COMMON_H_
#define _COMMON_H_

#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_AS726x.h"
#include "Motors.h"
// #ifdef DEBUG_DISPLAY
#include <U8g2lib.h>
#include <U8x8lib.h> 
#include "Adafruit_TCS34725.h"
// #endif

#define TOF_NUMBER 6
#define TOF_START 0

#define MUXADDR 0x70
#define OLED_CLK 28
#define OLED_DATA 30

#ifdef FAKE_SERIAL
#define PI_SERIAL Serial
#else
#define PI_SERIAL Serial2
#endif

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define BNO_X orientationData.orientation.x
#define BNO_Y orientationData.orientation.y
#define BNO_Z orientationData.orientation.z
#define FRONT_LEFT A14
#define FRONT_RIGHT A13
#define BACK_RIGHT A10
#define BACK_LEFT A8
#define UPDATE_BNO() bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#define DIGITAL_READ(x) digitalRead(x) && digitalRead(x)

U8X8_SSD1306_128X64_NONAME_SW_I2C oled(OLED_CLK, OLED_DATA);

#ifdef DEBUG_DISPLAY
#define oled_println(...) oled.println(__VA_ARGS__)
#define oled_print(...) oled.print(__VA_ARGS__)
#define oled_clear() oled.clear(); oled.clearDisplay(); oled.setCursor(0, 0)
#else
#define oled_println(...)
#define oled_print(...)
#define oled_clear()
#endif

typedef unsigned int uint;

Adafruit_BNO055 bno;

VL53L0X tof;

#ifdef TCS
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
#endif
#ifdef AMS
Adafruit_AS726x ams;
#endif

uint16_t amsValues[AS726x_NUM_CHANNELS];

Motor motorL(MPORT2);
Motor motorR(MPORT1, true, true);

sensors_event_t accelerometerData, gyroData, orientationData, linearAccelData;

const float CM_TO_ENCODERS = 360.f/(7.7f*PI);
const float ENCODERS_TO_CM = 1.f/CM_TO_ENCODERS;

const int minspeed = 200;
const double KP_TURN = 1.5;
const double KI_TURN = 0.00003;
const double KD_TURN = 0.243;
const int DRIVE_BOOST = 40;
const int TURN_BOOST = 105;
const int ALIGN_TURN_BOOST = 70;


const double DRIVE_STRAIGHT_KP = 3.0;
const double KP_FORWARD = 1.2;
const double KI_FORWARD = 0.003;
const double KD_FORWARD = 0.01;
const double SAMPLERATE_DELAY_MS = 10.0;
const double TIMES_PER_SECOND = 1000.0 / SAMPLERATE_DELAY_MS;
volatile int32_t global_angle = 0;
volatile bool restart = false;

const int SPEED = 105;
const int ALIGN_SPEED = 85;

enum type { Color,
            Distance };
enum direction { n = 0,
                 e = 1,
                 s = 2,
                 w = 3 };
volatile uint8_t cur_direction = n;
double xPos = 0, yPos = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;

const int wall_tresh = 175;
const float tile_dist = 31.1;
const float forward_offset = 1.5 * CM_TO_ENCODERS;

const double full_vic_percent_end = 0.9;
const double full_vic_percent_begin = 0.25;
const double strip_vic_percent = 0.1;

volatile bool left_dropped = false;
volatile bool right_dropped = false;

bool black_tile_detected = false;

// double detection
volatile char seen_l[4] = { 0 };
volatile char seen_r[4] = { 0 };

// move count for debug
volatile int move_count = 0;

// timer for resetting victim
volatile int32_t tvictim = 10000000;
volatile int32_t ttest = 10000000;

// color calibration

int color_to_value[6] = {AS726x_VIOLET, AS726x_BLUE, AS726x_GREEN, AS726x_YELLOW, AS726x_ORANGE, AS726x_RED};


enum { VIOLET, BLUE, GREEN, YELLOW, ORANGE, RED};

int blue[6] = { 0 };
int black[6] = { 0 };
int silver[6] = { 0 };

inline void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(MUXADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

char dir_to_char(uint8_t cur_dir)
{
  if(cur_dir > 3)
    return 'n';

  const char char_map[4] = {'n', 'e', 's', 'w'};
  return char_map[cur_dir];
}

void oled_display_walls(bool walls[4]) {
  const char char_map[4] = { 'n', 'e', 's', 'w' };
  String data = "    ";

  for (int i = 0; i < 4; i++) {
    if (walls[i])
      data[i] = char_map[i];
  }

  oled_println(data.c_str());
}

int closestToDirection(double num) {
  for (int i = 0; i <= 360; i += 90) {
    if (abs(num - i) <= 45) {
      return (i == 360 ? 0 : i);
    }
  }
}

#endif