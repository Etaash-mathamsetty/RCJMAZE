#include <Adafruit_BNO055.h>
#include "Motors.h"
#include "VL53L0X.h"
#include <Wire.h>
#include "utils.h"

#define FAKE_ROBOT
#define PI_SERIAL Serial2
#define MUXADDR 0x70
#define TOF_NUMBER 2
#define TOF_START 0
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t accelerometerData, gyroData, orientationData, linearAccelData;

VL53L0X tof;
Motor motorL(MPORT2);
Motor motorR(MPORT1, true, true);

const double KP_TURN = 1.2;
const double KI_TURN = 0.00003;
const double KD_TURN = 0.243;
const int TURN_BOOST = 60;


const double DRIVE_STRAIGHT_KP = 3.0;
const double KP_FORWARD = 1.2;
const double KI_FORWARD = 0.003;
const double KD_FORWARD = 0.01;
const double SAMPLERATE_DELAY_MS = 10.0;
const double TIMES_PER_SECOND = 1000.0 / SAMPLERATE_DELAY_MS;

const int SPEED = 100;

volatile double velocity = 0.0;
volatile double position = 0.0;
volatile double average_acceleration;
volatile unsigned long tStart = millis();
volatile int count = 0;

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

void setup() {
  //PI_SERIAL.begin(9600);
  Serial.begin(9600);
  utils::setMotors(&motorR, &motorL);
  Wire.begin();
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  Serial.println("starting the code!");

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {
    tcaselect(i);
    tof.init();
    //tof.setTimeout(500);
    tof.startContinuous();
  }
}

inline void pi_send_tag(const char* tag) {
  PI_SERIAL.print(tag);
  PI_SERIAL.print("::");
}

inline void display_tag(const char* tag) {
  Serial.print(tag);
  Serial.print(": ");
}

void display_data(const sensors_event_t& data) {

  switch (data.type) {
    case SENSOR_TYPE_ACCELEROMETER:
      {
        display_tag("accel");
        Serial.print("X: ");
        Serial.print(data.acceleration.x);
        Serial.print(" Y: ");
        Serial.print(data.acceleration.y);
        Serial.print(" Z: ");
        Serial.println(data.acceleration.z);
        break;
      }
    case SENSOR_TYPE_GYROSCOPE:
      {
        display_tag("gyro");
        Serial.print("X: ");
        Serial.print(data.gyro.x);
        Serial.print(" Y: ");
        Serial.print(data.gyro.y);
        Serial.print(" Z: ");
        Serial.println(data.gyro.z);
        break;
      }
    case SENSOR_TYPE_ORIENTATION:
      {
        display_tag("orientation");
        Serial.print("X: ");
        Serial.print(data.orientation.x);
        Serial.print(" Y: ");
        Serial.print(data.orientation.y);
        Serial.print(" Z: ");
        Serial.println(data.orientation.z);
      }
    default:
      break;
  }
}
//forward_status[0] = is forward command runnning
//forward_status[1] = is black tile ? (or command failed)
void pi_send_data(bool forward, bool black_tile) {
  double arr[2] = { forward, black_tile };
  pi_send_tag("forward_status");

  PI_SERIAL.print(arr[0]);
  PI_SERIAL.print(',');
  PI_SERIAL.println(arr[1]);
}

void pi_send_data(bool walls[4]) {
  pi_send_tag("NW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (n - cur_direction), sizeof(walls)/sizeof(walls[0]))]);
  pi_send_tag("EW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (e - cur_direction), sizeof(walls)/sizeof(walls[0]))]);
  pi_send_tag("SW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (s - cur_direction), sizeof(walls)/sizeof(walls[0]))]);
  pi_send_tag("WW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (w - cur_direction), sizeof(walls)/sizeof(walls[0]))]);
}

void pi_send_data(const sensors_event_t& data) {

  switch (data.type) {
    case SENSOR_TYPE_ACCELEROMETER:
      {
        pi_send_tag("accel");
        PI_SERIAL.print(data.acceleration.x);
        PI_SERIAL.print(",");
        PI_SERIAL.print(data.acceleration.y);
        PI_SERIAL.print(",");
        PI_SERIAL.println(data.acceleration.z);
      }
    case SENSOR_TYPE_GYROSCOPE:
      {
        pi_send_tag("gyro");
        PI_SERIAL.print(data.gyro.x);
        PI_SERIAL.print(",");
        PI_SERIAL.print(data.gyro.y);
        PI_SERIAL.print(",");
        PI_SERIAL.println(data.gyro.z);
      }
    case SENSOR_TYPE_ORIENTATION:
      {
        pi_send_tag("orientation");
        PI_SERIAL.print(data.orientation.x);
        PI_SERIAL.print(",");
        PI_SERIAL.print(data.orientation.y);
        PI_SERIAL.print(",");
        PI_SERIAL.println(data.orientation.z);
      }
    default:
      break;
  }
}

byte get_tof_vals(int threshold) {

  byte wall = 0;
  int reading;

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {

    tcaselect(i);
    reading = tof.readRangeContinuousMillimeters();
    wall <<= 1;

    if (reading < threshold) {
      wall |= 1;
      Serial.println("Wall");
    } else {
      Serial.println("No Wall");
    }
    Serial.println(reading);
  }

  return wall;
}

void send_tof_vals(byte tof_val) {
  pi_send_tag("tof");
  PI_SERIAL.write(tof_val);
  PI_SERIAL.println();
}

void pi_read_data() {
  const char* commands_array[] = { "ge", "gw", "gn", "gs" };
  const int num_commands = sizeof(commands_array) / sizeof(commands_array[0]);
  static int cur_command = 0;

#ifndef FAKE_ROBOT
  while (!PI_SERIAL.available())
    ;
  String data = PI_SERIAL.readString();
#endif

#ifdef FAKE_ROBOT
  String data = commands_array[cur_command];
  cur_command++;
  cur_command %= num_commands;
#endif

  data.trim();
  data.toLowerCase();
  data += '\n';
  String cur_cmd = "";
  Serial.println(data);
  for (char c : data) {
    if (c == 'g' || c == 'f' || c == 't') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'g' || cur_cmd[0] == 'f') {
          Serial.println("FORWARD");
          drive(1000, 100, 1);

        } else {
          Serial.println("ERR: Invalid Parameter");
        }
      }
      cur_cmd.remove(0);
      cur_cmd += c;
    }
    if (c == 'e' || c == 'w' || c == 's' || c == 'n') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'f' || cur_cmd[0] == 'g') {
          Serial.print("turn to ");
          Serial.println(c);
          Serial.println("FORWARD");
          turn(c);
          pi_send_data({ false, false, false, false });
          drive(1000, 100, 1);
        } else if (cur_cmd[0] == 't') {
          Serial.print("turn to ");
          Serial.println(c);
          turn(c);
          pi_send_data({ false, false, false, false });
        } else {
          Serial.println("ERR: Invalid Command");
        }
        cur_cmd.remove(0);
        continue;
      } else {
        Serial.println("ERR: Invalid Command");
        continue;
      }
    }
    if (c == '\n' || c == '\0') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'g' || cur_cmd[0] == 'f') {
          Serial.println("FORWARD");
          drive(1000, 100, 1);
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
      }
      cur_cmd.remove(0);
    }
  }
}

void left(int relative_angle, int speed) {
  motorL.addBoost(TURN_BOOST);
  motorR.addBoost(TURN_BOOST);
  int angle = 360 - relative_angle;
  double p, i = 0, d;
  double PID;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double last_error = abs((orientationData.orientation.x - angle) / angle);
  double orientation = orientationData.orientation.x + 360;

  while (orientation > angle) {

    p = abs((orientation - angle) / relative_angle);
    //i = i + p;
    //d = p - last_error;
    PID = KP_TURN * p;
    //last_error = p;

    utils::forward((PID * speed), (PID * -speed));

    if (PID <= 0.01)
      break;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orientation = orientationData.orientation.x;

    if (orientationData.orientation.x < 1.0) {
      orientation += 360;
    }
  }
  motorL.addBoost(0);
  motorR.addBoost(0);
  utils::stopMotors();
}

void right(int angle, int speed) {

  motorL.addBoost(TURN_BOOST);
  motorR.addBoost(TURN_BOOST);

  double p, i = 0, d;
  double PID;
  double start = millis();
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double last_error = abs((orientationData.orientation.x - angle) / angle);

  while (orientationData.orientation.x < angle) {

    p = abs((orientationData.orientation.x - angle) / angle);
    //i = (i + p) * (start - millis());
    //d = (p - last_error) / (start - millis());

    PID = KP_TURN * p;

    utils::forward((PID * -speed), (PID * speed));
    Serial.println(PID);

    /* prevents error value from being too low */
    if (PID <= 0.01)
      break;
    //utils::forward(100,-100);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  }
  utils::stopMotors();
  motorL.addBoost(0);
  motorR.addBoost(0);
}

void turn(char char_end_direction) {
  uint8_t end_direction;
  switch (tolower(char_end_direction)) {
    case 'n': end_direction = (uint8_t)n; break;
    case 'e': end_direction = (uint8_t)e; break;
    case 's': end_direction = (uint8_t)s; break;
    case 'w': end_direction = (uint8_t)w; break;
    default: Serial.println("invalid"); break;
  }

  switch (cur_direction - end_direction) {
    case -3:
    case 1: left(90, SPEED); break;
    case -1:
    case 3: right(90, SPEED); break;
    case 2:
    case -2: left(180, SPEED); break;
    default: Serial.println("invalid");
    case 0: break;
  }
  cur_direction = end_direction;
}

void drive(int encoders, int speed, int tolerance) {

  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  int angle, tofR, tofL; 
  utils::resetTicks();
  double p, d, i = 0;
  double p_turn, d_turn, last_difference = 0;
  double PID;
  double last_dist = abs(motorR.getTicks() / abs(encoders));
  double startX = xPos;

  tcaselect(2); 
  tofL = tof.readRangeContinuousMillimeters(); 
  tcaselect(3); 
  tofR = tof.readRangeContinuousMillimeters(); 



  pi_send_data(true, false);

  while (abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders)) {
    motorL.addBoost(TURN_BOOST);
    motorR.addBoost(TURN_BOOST);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    int minspeed = 50;
    p = speed * (double)(abs(encoders) - abs(motorR.getTicks())) / abs(encoders);
    //i = i + p;
    //d = p - last_dist;
    PID = p * KP_FORWARD;
    Serial.println(PID);

    if (orientationData.orientation.x > 180) {
      p_turn = -(orientationData.orientation.x - 360) - (startX - xPos);
    } else {
      p_turn = -orientationData.orientation.x - (startX - xPos);
    }
    
    


    if (abs(p_turn * DRIVE_STRAIGHT_KP) <= 0.01 && PID <= 0.01)
      break;
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    //    Serial.println(speed * (double)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    utils::forward(PID - p_turn * DRIVE_STRAIGHT_KP, PID + p_turn * DRIVE_STRAIGHT_KP);
    angle = orientationData.orientation.x;
  } 
  //correct horizontal error when inside of hallway 
  if(tofR < 175 && tofL < 175){ 
            
      while(tofR - tofL > tolerance){ 
         right(60); 
         utils::forward(100, 100); 
         delay(150); 
         left(60); 
         utils::forward(-100, -100);  
         delay(130); 

     } 
     while(tofL - tofR > tolerance){ 
         left(60); 
         utils::forward(100, 100); 
         delay(150); 
         right(60); 
         utils::forward(-100, -100);  
         delay(130); 

     }
  }

  utils::stopMotors();
  pi_send_data(false, false);
  motorL.addBoost(0);
  motorR.addBoost(0);
} 


void acceleration_position() {
  unsigned long tStart = micros();
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  if ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
    xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  }
}

void loop() {
  //acceleration_position();
  //pi_read_data();
  /*
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  */
  /*
  display_data(accelerometerData);
  pi_send_data(accelerometerData);
  display_data(gyroData);
  pi_send_data(gyroData);
  pi_read_data();
  Serial.println();*/
  /*
  byte test = get_tof_vals(100);
  Serial.print("Tof: ");
  Serial.println(test, BIN);*/
  //send_tof_vals(test);

  //Serial.println("hi");
  //Serial.println(get_tof_vals(100));
  //Serial.println("hi");
  pi_read_data();

  //drive(1000, 100, 1);
  //left(180,100);
  //delay(1000);

  //left(90,200);
  //delay(1000);

  /*straightDrive(2000,80,3);
  delay(2000);*/
}
