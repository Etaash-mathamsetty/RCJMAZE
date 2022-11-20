#include "Motors.h"
#include "utils.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

enum Direction {turnL,turnR};

const float KP_TURN = 3.0;
const float KI_TURN = 0.003;
const float KD_TURN = 0.243;

const float KP_FORWARD = 3.0;
const float KI_FORWARD = 0.003;
const float KD_FORWARD = 0.01;
//const float KI_TURN = 0;
//const float KD_TURN = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

sensors_event_t orientationData;


Motor motor1(MPORT1, true, true);
Motor motor2(MPORT2);


void setup() {
  // put your setup code here, to run once:
  utils::setMotors(&motor1, &motor2);
  Serial.begin(9600);
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  Serial.println("Success");
}

void turn(int angle, int speed, int direction) {

  if (direction == turnR)
    right(angle, speed);
  else if (direction == turnL)
    left(angle, speed);
  else
    Serial.println("bro what?");
}

void straightDrive(int encoders, int speed, int tolerance) {

  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  int angle;
  utils::resetTicks();
  float p, d, i = 0;
  float p_turn, d_turn, last_difference = 0;
  float PID;
  float last_dist = abs(motor1.getTicks()/abs(encoders));

  while (abs(motor1.getTicks()) < abs(encoders) && abs(motor2.getTicks()) < abs(encoders)) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    int minspeed = 50;
    p = speed * (float)(abs(encoders) - abs(motor1.getTicks())) / abs(encoders);
    i = i + p;
    d = p - last_dist;
    PID = p * KP_FORWARD + i * KI_FORWARD + d * KD_FORWARD;

    if (orientationData.orientation.x > 180){
      p_turn = orientationData.orientation.x - 360;
    }
    else {
      p_turn = orientationData.orientation.x;
    }
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    //    Serial.println(speed * (float)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    utils::forward(PID + p_turn, PID - p_turn);
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
  }
  utils::stopMotors();*/
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

void loop() {
  // put your main code here, to run repeatedly:
  //bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  //left(90, 100);
  //straightDrive(1000, 80, 5);
  //turn(180, 80, turnL);
  straightDrive(500, 80, 5);
  //delay(1000);
  //right(90, 100);
  delay(1000);
}