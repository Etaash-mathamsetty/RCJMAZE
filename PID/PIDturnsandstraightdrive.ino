#include "Motors.h"
#include "utils.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
static char L = 'L';
static char R = 'R';
const float Kp = 1;
const float Ki = 0;
const float Kd = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

sensors_event_t orientationData;


Motor motor1(MPORT1);
Motor motor2(MPORT2);


void setup() {
  // put your setup code here, to run once:
  utils::setMotors(&motor1, &motor2);
  Serial.begin(9600);
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  Serial.println("Success");
}
void right(int angle, int speed) {
  float orientation = 0;
  float p, i, d = 0;
  float PID;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x + angle);
  orientation = orientationData.orientation.x > angle + (goal - 360) ? orientationData.orientation.x - 360 : orientationData.orientation.x;

  if (goal >= 360) {

    goal -= 360;
    float last_error = abs((orientation - angle) / angle);
    while (orientation < goal) {
      p = abs((orientation - angle) / angle);
      i = p - last_error;
      d = d + p;
      PID = Kp * p + Ki * i + Kd * d;

      Serial.println(orientation);
      // Serial.println(goal);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x > angle + goal ? orientationData.orientation.x - 360 : orientationData.orientation.x;
      // Serial.println(orient);
      utils::forward((PID * -speed) - 70, (PID * speed) + 70);
      last_error = p;
    }
  }

  else {
    float last_error = abs((orientationData.orientation.x - angle) / angle);
    while ((int)orientationData.orientation.x < goal) {
      p = abs((orientationData.orientation.x - angle) / angle);
      i = p - last_error;
      d = d + p;
      PID = Kp * p + Ki * i + Kd * d;

      Serial.println(p);
      // Serial.println(orientationData.orientation.x);
      //  Serial.println(goal);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      utils::forward((PID * -speed) - 70, (PID * speed) + 70);
      last_error = p;
    }
  }
  utils::stopMotors();
}
void left(int angle, int speed) {
  float orientation = 0;
  float p, i, d = 0;
  float PID;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x - angle);

  orientation = orientationData.orientation.x < goal + 360 - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;

  if (goal < 0) {

    goal += 360;
    float last_error = abs((orientation - angle) / angle);
    Serial.println(orientationData.orientation.x);
    Serial.println(goal);
    while (orientation > goal) {
      p = abs((orientation - angle) / angle);
      i = p - last_error;
      d = d + p;
      PID = Kp * p + Ki * i + Kd * d;


      Serial.println(orientationData.orientation.x);
      Serial.println(p);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x < goal - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;
      utils::forward((PID * speed) + 70, (PID * -speed) - 70);
      last_error = p;
    }
  }

  else {
    float last_error = abs((orientationData.orientation.x - angle) / angle);
    while ((int)orientationData.orientation.x > goal) {
      p = abs((orientationData.orientation.x - angle) / angle);
      i = p - last_error;
      d = d + p;
      PID = Kp * p + Ki * i + Kd * d;

      Serial.println(orientationData.orientation.x);
      Serial.println(p);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      utils::forward((PID * speed) + 70, (PID * -speed) - 70);
      last_error = p;
    }
  }
  utils::stopMotors();
}
void turn(int angle, int speed, char direction) {
  if (direction == 'R')
    right(angle, speed);

  else if (direction == 'L')
    left(angle, speed);
  else
    Serial.println("bro what?");
}
void straightDrive(int encoders, int speed, int tolerance) {

  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  int angle;
  utils::resetTicks();

  while (abs(motor1.getTicks()) < abs(encoders) && abs(motor2.getTicks()) < abs(encoders)) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    int minspeed = 50;
    float p = speed * (float)(abs(encoders) - abs(motor1.getTicks())) / abs(encoders);
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    //    Serial.println(speed * (float)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    utils::forward(p + minspeed, p + minspeed);
    angle = orientationData.orientation.x;
    Serial.println(orientationData.orientation.x);
  }
  utils::stopMotors();
  while (angle > tolerance && angle < 180) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    angle = orientationData.orientation.x;
    utils::forward(100, -100);
  }
  while (angle < (360 - tolerance) && angle > 180) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    angle = orientationData.orientation.x;
    utils::forward(-100, 100);
  }
  utils::stopMotors();
  return;
}
void loop() {
  // put your main code here, to run repeatedly:
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  turn(90, 80, R);
  straightDrive(1000, 80, 5);
  turn(179, 80, L);
  straightDrive(1000, 80, 5);
  delay(1000);
}