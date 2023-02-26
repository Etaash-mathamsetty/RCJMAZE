#include <Adafruit_BNO055.h>
#include "Motors.h"
#include "VL53L0X.h"
#include <Wire.h>
#include "utils.h"

//#define FAKE_ROBOT
#define FAKE_SERIAL

#ifdef FAKE_SERIAL
#define PI_SERIAL Serial
#endif
#ifndef
#define PI_SERIAL Serial2
#endif

#define MUXADDR 0x70
#define TOF_NUMBER 2
#define TOF_START 0
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t accelerometerData, gyroData, orientationData, linearAccelData;

VL53L0X tof;
Motor motorL(MPORT2);
Motor motorR(MPORT1, true, true);

const float CM_TO_ENCODERS = 100/4.6;
const float ENCODERS_TO_CM = 4.6/100;

const int minspeed = 200;
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

    if (PID * speed < minspeed) {
      utils::forward((PID * speed), (PID * -speed));
    }
    else {
      utils::forward((minspeed), (-minspeed));
    }

    if (PID <= 0.01)
      break;

    Serial.print("PID: ");
    Serial.println(PID);

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

    if (PID * speed < minspeed) {
      utils::forward((PID * -speed), (PID * speed));
    }
    else {
      utils::forward((-minspeed), (minspeed));
    }
   //                                                                                        Serial.println(PID);

    /* prevents error value from being too low */
    if (PID <= 0.01)
      break;

    Serial.print("PID: ");
    Serial.println(PID);
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

void driveCM (float cm, int speed = 200, int tolerance = 1) {
  drive(cm * CM_TO_ENCODERS, speed, tolerance);
}

void drive(int encoders, int speed, int tolerance) {

  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  int angle = 60, tofR1, tofR2; 
  utils::resetTicks();
  double p, d, i = 0;
  double p_turn, d_turn, last_difference = 0;
  double PID;
  double last_dist = abs(motorR.getTicks() / abs(encoders));
  double startX = xPos;
  pi_send_data(true, false);

  while (abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders)) {
    motorL.addBoost(TURN_BOOST);
    motorR.addBoost(TURN_BOOST);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

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
/*  if(tofR < 175 && tofL < 175){ 
            
      while(tofR - tofL > tolerance){ 
         right(angle, SPEED); 
         utils::forward(SPEED, SPEED); 
         delay(150); 
         left(angle, SPEED); 
         utils::forward(-SPEED, -SPEED);  
         delay(130); 

     } 
     while(tofL - tofR > tolerance){ 
         left(angle, SPEED); 
         utils::forward(SPEED, SPEED); 
         delay(150); 
         right(angle, SPEED); 
         utils::forward(-SPEED, -SPEED);  
         delay(130); 

     }
  } 
  */ 
  
 // alignAngle(SPEED); 

  
  /*
  while(tofR1 > 100){
    tcaselect(0); 
    tofR1 = tof.readRangeContinuousMillimeters() - 50; 
    tcaselect(1); 
    tofR2 = tof.readRangeContinuousMillimeters() - 15; 
    shiftRight(); 
    
  } 
  
  align(tolerance);  
  */
  
  utils::stopMotors();
  pi_send_data(false, false);
  motorL.addBoost(0);
  motorR.addBoost(0);
} 
void shiftRight(){
    right(15, SPEED); 
    utils::forward(SPEED, SPEED);
    delay(25);
    left(15, SPEED);
    utils::forward(-SPEED, -SPEED);
    delay(24);
    return; 
} 
void shiftLeft(){
    left(15, SPEED); 
    utils::forward(SPEED, SPEED);
    delay(25);
    right(15  , SPEED);
    utils::forward(-SPEED, -SPEED);
    delay(24);
    return; 
} 

void alignCenterLR(int speed) {
  int tofR1, tofL1; 
  tcaselect(0);
  tofR1 = tof.readRangeContinuousMillimeters() - 50;
  tcaselect(1);
  tofL1 = tof.readRangeContinuousMillimeters() - 15;

  const int dist = tofR1 - tofL1;

  if (tofR1 < tofL1) {
    right(90, 100);
  } else {
    left(90, 100);
  }

  while (abs(tofR1 - tofL1) > 10) {
    utils::forward(speed, speed);
  }

  if (tofR1 < tofL1) {
    left(90, 100);
  } else {
    right(90, 100);
  }
}

void alignCenterFB(int speed) {
  int tofF1, tofB1; 
  tcaselect(0);
  tofF1 = tof.readRangeContinuousMillimeters() - 50;
  tcaselect(1);
  tofB1 = tof.readRangeContinuousMillimeters() - 15;

  const int dist = tofF1 - tofB1;

  if (tofF1 > tofB1) {
    speed = -speed;
  }

  while (abs(tofF1 - tofB1) > 10) {
    utils::forward(speed, speed);
  }
}

void alignAngle(int speed) {
  float tofR1, tofR2; 
  tcaselect(0);
  tofR1 = tof.readRangeContinuousMillimeters() - 50;
  tcaselect(1);
  tofR2 = tof.readRangeContinuousMillimeters() - 15;

  if (tofR1 >= 200 || tofR2 >= 200) {
    return;
  }

  float len = tofR1 - tofR2;
  if (len == 0) {
    len += 0.01;
  }
  const int width = TOF_DISTANCE;
  const int angle = 90 - (atan(width/len) * (180/3.1415)); 
  //Serial.println(len);  
  //Serial.println(atan(width/len) * (180/3.1415)  );
  //Serial.println(angle);
  Serial.print("Length: ");
  Serial.print(len);
  if(angle > 10 && angle < 170) {
  if (angle > 135) {
    // Serial.print(" Turn Right\t");
    // Serial.println(angle);
    right(180 - angle, speed);
  } else {
    // Serial.print(" Turn Left\t");
    // Serial.println(angle);
    left(angle, speed);
  }
  } else {
    Serial.println();
  }
   
  /*
  while (abs(tofR1 - tofR2) > tolerance) {
    Serial.println(abs(tofR1 - tofR2)); 
    tcaselect(0);
    tofR1 = tof.readRangeContinuousMillimeters() - 50;
    tcaselect(1);
    tofR2 = tof.readRangeContinuousMillimeters() - 15;

    if (tofR1 - tofR2 > 10) {
      left(10, SPEED);
    }
    else if (tofR2 - tofR1 > 10) {
      right(10, SPEED);
    }
  }*/
  
}

void acceleration_position() {
  unsigned long tStart = micros();
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  if ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
    xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  }

  Serial.print("xPos: ");
  Serial.print(xPos);
  Serial.print("yPos: ");
  Serial.println(yPos);
}

int tofCalibrated(int tof) {
  switch (tof) {
    case 0: {
        tcaselect(0);
        int x1 = tof.readRangeContinuousMillimeters();
        int tofR1 = -89.7 + (x1 * 1.9) - (0.0033 * (x1 * x1));
        return tofR1;
        //accurate (50, 150), horrible < 25
    }
    case 1: {
        tcaselect(1);
        int x2 = tof.readRangeContinuousMillimeters();
        int tofR2 = (1.09 * x2) - 21.3;
        return tofR2;
        //accurate (50, 150), still works < 25ish
    }
    case 2: {
        tcaselect(2);
        int x3 = tof.readRangeContinuousMillimeters();
        int tofL1 = (1.03 * x3) - 9.91;
        return tofL1;
        //accrate (50, 150), passable < 50 but not that good
    } 
    case 3: { 
        tcaselect(3); 
        int x4 = tof.readRangeContinuousMillimeters(); 
        int tofL2 = -0.848 + (0.671 * x4) + (0.00165 * x4 * x4); 
        return tofL2; 
        //decent accuracy 
      
    } 
    case 4: { 
        tcaselect(4); 
        int x5 = tof.readRangeContinuousMillimeters(); 
        int tofF = 3 + (0.657 * x5) + (0.00146 * x5 * x5);
        return tofF;   
        //pretty accurate
    } 
    default:
      break;
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
  //alignAngle(100); 
  //delay(500);
  // driveCM(30, 200, 0);
  // delay(1000);

}
