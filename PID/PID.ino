#include <Adafruit_BNO055.h>
#include "Motors.h"
#include "VL53L0X.h"
#include <Wire.h>
#include "utils.h"

#define FAKE_ROBOT
#define PI_SERIAL Serial2
#define MUXADDR 0x70
#define TOF_NUMBER 3
#define TOF_START 0

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t accelerometerData, gyroData, orientationData;

VL53L0X tof;
Motor motorL(MPORT2);
Motor motorR(MPORT1, true, true);

const float KP_TURN = 3.0;
const float KI_TURN = 0.003;
const float KD_TURN = 0.243;

const float DRIVE_STRAIGHT_KP = 3.0;
const float KP_FORWARD = 3.0;
const float KI_FORWARD = 0.003;
const float KD_FORWARD = 0.01;
const double SAMPLERATE_DELAY_MS = 10.0;
const double TIMES_PER_SECOND = 1000.0 / SAMPLERATE_DELAY_MS;

volatile double velocity = 0.0;
volatile double position = 0.0;
volatile double average_acceleration;
volatile unsigned long tStart = millis();
volatile int count = 0;



void setup() {
  //PI_SERIAL.begin(9600);
  Serial.begin(9600);
  utils::setMotors(&motorR, &motorL);
  Wire.begin();
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  Serial.println("starting the code!");

  for (int i = TOF_START; i < TOF_NUMBER; i++) {
    tcaselect(i);
    tof.setTimeout(500);
    tof.init();
    tof.startContinuous();
  }
}

inline void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(MUXADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
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

  for (int i = TOF_START; i < TOF_NUMBER; i++) {

    tcaselect(i);
    reading = tof.readRangeContinuousMillimeters();
    wall <<= 1;

    if (reading < threshold) {
      wall |= 0b1;
      Serial.println("Wall");
    } else {
      Serial.println("No Wall");
    }
    Serial.println(reading);
  }

  return wall;
}

void send_tof_vals(byte tof_val){
  pi_send_tag("tof");
  PI_SERIAL.write(tof_val);
  PI_SERIAL.println();
}

void pi_read_data() {
  while (!PI_SERIAL.available());

  String data = PI_SERIAL.readString();
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
        } else if (cur_cmd[0] == 't') {
          Serial.print("turn to ");
          Serial.println(c);
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
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
      }
      cur_cmd.remove(0);
    }
  }
}

void left(int relative_angle, int speed){

  int angle = 360 - relative_angle;
  float p, i = 0, d;
  float PID;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float last_error = abs((orientationData.orientation.x - angle) / angle);
  float orientation = orientationData.orientation.x + 360;

  while (orientation > angle){

    p = abs((orientation - angle) / relative_angle);
    i = i + p;
    d = p - last_error;
    PID = KP_TURN * p + KI_TURN * i + KD_TURN * d;
    last_error = p;

    utils::forward((PID * speed), (PID * -speed));
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orientation = orientationData.orientation.x;

    if (orientationData.orientation.x < 1.0){
      orientation += 360;
    }
  }
  utils::stopMotors();
}

void right(int angle, int speed){

  float p, i = 0, d;
  float PID;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float last_error = abs((orientationData.orientation.x - angle) / angle);
    
  while (orientationData.orientation.x < angle){

    p = abs((orientationData.orientation.x - angle) / angle);
    i = i + p;
    d = p - last_error;
    PID = KP_TURN * p + KI_TURN * i + KD_TURN * d;
    last_error = p;

    utils::forward((PID * -speed), (PID * speed));
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  }
  utils::stopMotors();
}

void straightDrive(int encoders, int speed, int tolerance) {

  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  int angle;
  utils::resetTicks();
  float p, d, i = 0;
  float p_turn, d_turn, last_difference = 0;
  float PID;
  float last_dist = abs(motorR.getTicks()/abs(encoders));

  while (abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders)) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    int minspeed = 50;
    p = speed * (float)(abs(encoders) - abs(motorR.getTicks())) / abs(encoders);
    i = i + p;
    d = p - last_dist;
    PID = p * KP_FORWARD + i * KI_FORWARD + d * KD_FORWARD;

    if (orientationData.orientation.x > 180){
      p_turn = -(orientationData.orientation.x - 360);
    }
    else {
      p_turn = -orientationData.orientation.x;
    }
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    //    Serial.println(speed * (float)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    utils::forward(PID - p_turn * DRIVE_STRAIGHT_KP, PID + p_turn * DRIVE_STRAIGHT_KP);
    angle = orientationData.orientation.x;
    Serial.println(orientationData.orientation.x);
  }
  utils::stopMotors();

  /*
  while (angle > tolerance && angle < 180) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    angle = orientationData.orientation.x;
    utils::forward(100, -100);
  }
  while (angle < (360 - tolerance) && angle > 180) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    angle = orientationData.orientation.x;
    utils::forward(-100, 100);
  }*/
  utils::stopMotors();
}

void loop() {

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
  

  //right(90,100);
  //delay(1000);

  //left(90,100);
  //delay(1000);

  /*straightDrive(2000,80,3);
  delay(2000);*/

  if((millis() - tStart) % 1000 == 0){
    Serial.println(millis() - tStart);
    count++;
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    
    //average acceleration cm/ms^2

    average_acceleration = (average_acceleration + accelerometerData.acceleration.x) / (float)(count);    
    position = position + 0.5 * average_acceleration * (float) count * (float) count;
    Serial.print("Orientation: ");
    Serial.print(orientationData.orientation.x);
    Serial.print("\tAcceleration: ");
    Serial.print(accelerometerData.acceleration.x);
    Serial.print("\tAverage Acceleration: ");
    Serial.print(average_acceleration);
    Serial.print("\tPosition: ");
    Serial.println(position);
  }


}
