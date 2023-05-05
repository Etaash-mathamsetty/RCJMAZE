//#define FAKE_ROBOT
//#define FAKE_SERIAL
#define DEBUG_DISPLAY
//#define MOTORSOFF
#define TEST

#include "Motors.h"
#include "utils.h"
#include "common.h"

void setup() {
#ifndef FAKE_SERIAL
  PI_SERIAL.begin(115200);
#endif
  Serial.begin(9600);
  utils::setMotors(&motorR, &motorL);
  Wire.begin();
  Serial.println("starting the code!");
  //Wire.begin();
  //Wire.setClockStretchLimit(200000L);

  oled.begin();
  oled.setFlipMode(0);
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.setCursor(0, 0);
  oled.println("Starting...");

  bno.begin(OPERATION_MODE_IMUPLUS);
  oled.println("BNO init done!");

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {
    tcaselect(i);
    tof.init();
    //tof.setTimeout(500);
    //tof.startContinuous();
  } 
  oled.println("TOF init done!");
  
  tcaselect(6);
  if (!tcs.begin())
  {
    Serial.println("color sensor init fail!");
  }

  oled.println("TCS init done!");
  
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  analogWrite(2, 10);  
  Serial.println("TOF INIT SUCCEED!");
  #ifdef DEBUG_DISPLAY
    oled.println("Startup Done!");
    delay(1000);
    oled.setCursor(0, 0);
    oled.clearDisplay();    
  #endif
  analogWrite(2, 0); 
  // delay(500);
 
}

inline void pi_send_tag(const char* tag) {
  PI_SERIAL.print(tag);
  PI_SERIAL.print("::");
}

inline void display_tag(const char* tag) {
  Serial.print(tag);
  Serial.print(": ");
}

void display_bno_data(const sensors_event_t& data) {

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
//forward_status[1] = true succeed ? (or command failed, black tile, etc)
void pi_send_data(bool forward, bool black_tile) {
  double arr[2] = { forward, black_tile };
  pi_send_tag("forward_status");

  PI_SERIAL.print(arr[0]);
  PI_SERIAL.print(',');
  PI_SERIAL.println(arr[1]);
}

void pi_send_data(bool walls[4]) {
  pi_send_tag("NW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (n - cur_direction), 4)]);
  pi_send_tag("EW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (e - cur_direction), 4)]);
  pi_send_tag("SW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (s - cur_direction), 4)]);
  pi_send_tag("WW");
  PI_SERIAL.println(walls[utils::math::wrapAround((int) (w - cur_direction), 4)]);
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

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

bool* get_tof_vals(double threshold) {

  static bool arr[TOF_NUMBER + 1];
  double reading;

  memset(arr, 0, ARRAY_SIZE(arr));

  Serial.println("start");

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {

    reading = tofCalibrated(i);

    if (reading < threshold) {
      arr[i] = true;
      Serial.println("Wall");
    } else {
      Serial.println("No Wall");
    }
    Serial.println(reading);
  }

  Serial.println("end");
  //Serial.println("passed");

  return arr;
}

void send_tof_vals(byte tof_val) {
  pi_send_tag("tof");
  PI_SERIAL.write(tof_val);
  PI_SERIAL.println();
}

void driveCM(float,int,int);

/* there is a bug in this function somewhere 
the problem is that we need to read & execute every single command before continuing */
void pi_read_data() {
#ifdef FAKE_ROBOT
  const char* commands_array[] = { "ge", "gw", "gn", "gs" };
  const int num_commands = sizeof(commands_array) / sizeof(commands_array[0]);
  static int cur_command = 0;
#endif

#ifndef FAKE_ROBOT
  //while (!PI_SERIAL.available());

  String data = "";
  char ch = 0;
  while(PI_SERIAL.available() && ch != '\n') 
  {
    ch = PI_SERIAL.read();
    data += ch;
    delay(10);
  }

  //data += "\n";
  // String data = PI_SERIAL.readString();
#else
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
    if(c == 'r')
    {
      cur_cmd.remove(0);
      cur_cmd += c;
    }
    if (c == 'g' || c == 'f' || c == 't') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'g' || cur_cmd[0] == 'f') {
          Serial.println("FORWARD");
          oled.println("forward");
          driveCM(27, 110, 1);
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
          oled.print("turn to ");
          oled.println(c);
          oled.println("forward");
          turn(c);
          //pi_send_data({ false, false, false, false });
          driveCM(27, 110, 1);
        } else if (cur_cmd[0] == 't') {
          Serial.print("turn to ");
          Serial.println(c);
          oled.print("turn to ");
          oled.println(c);
          turn(c);
          oled.println("done");
          //pi_send_data({ false, false, false, false });
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
          oled.println("forward");
          driveCM(27, 110, 1);
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
        if (cur_cmd[0] == 'r')
        {
          bool* arr = get_tof_vals(150);

          //oled.println("test2");
          
          // Serial.print("Tof Vals: ");
          // Serial.println(vals);

          // //n e s w
          bool walls[4] = {arr[4], arr[0] && arr[1], arr[5], arr[2] && arr[3]};
          // not wrapped around and stuff 
          //oled_display_walls(walls);
          //  this is wrapped
          pi_send_data(walls);
        }
      }
      cur_cmd.remove(0);
    }
  }
}

int returnColor(bool only_black = false){
    uint16_t r, g, b, c = 0;  
    tcaselect(6);
    tcs.getRawData(&r, &g, &b, &c); 
    // oled.println(r);
    // delay(50);
    // oled.println(g);
    // delay(50);
    // oled.println(b);
    // delay(50);
    // oled.println(c);
    // delay(2000);
    if (c >= 500 /* && (r / (float)g) * 10 >= 10.5*/ && !only_black) {

   //   silver_persistance++;
      Serial.println("silver detected"); 
      oled.println("silver");
      //pi_send_tag("color");
      //PI_SERIAL.println("silver");
      return 0; //change later
    }
    if(c < 100){ 
      Serial.println("black detected"); 
      //oled.println("black");
      //pi_send_tag("color");
      //PI_SERIAL.println("black");  
      return 1;  
    }
    if(b > (r * 2.5) && !only_black){ 
      Serial.println("blue detected");
      oled.println("blue");  
      //pi_send_tag("color"); 
     //PI_SERIAL.println("blue"); 
      return 0; //change later
    }
  return 0;  
}

void right(int relative_angle, int speed) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // double orientation = orientationData.orientation.x;
  
  pi_send_tag("turn_status");
  PI_SERIAL.println(1.0);

  
  int offset = (int) (orientationData.orientation.x) % relative_angle;

  if (offset < relative_angle / 2) {
    relative_angle -= offset;
  }
  // Serial.print("With offset: ");
  // if (offset < relative_angle / 2) {

  //   Serial.println(relative_angle - offset);
  // } else {
  //   Serial.println(relative_angle + offset);
  // }

  raw_right(relative_angle, speed);

  pi_send_tag("turn_status");
  PI_SERIAL.println(0.0);
}

void left(int relative_angle, int speed) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double orientation = orientationData.orientation.x;

  if (abs(orientationData.orientation.x - global_angle) > 180)
    orientation = orientationData.orientation.x - 360;

  int offset = relative_angle - (int) (orientationData.orientation.x) % relative_angle;

  if (offset < relative_angle / 2) {
    relative_angle -= offset;
  }

  pi_send_tag("turn_status");
  PI_SERIAL.println(1.0);
  
  // raw_left(relative_angle + (orientation - global_angle), speed);

  // int offset = (orientation - relative_angle);
  // if (offset < 0) {
  //   offset += relative_angle;
  // }

  // offset %= relative_angle;

  // if (offset < relative_angle / 2) {
  //   relative_angle -= offset;
  // } else {
  //   relative_angle += offset;
  // }

  raw_left(relative_angle, speed);

  pi_send_tag("turn_status");
  PI_SERIAL.println(0.0);
}

void raw_right(int relative_angle, int speed) {

#ifndef MOTORSOFF
  motorL.addBoost(TURN_BOOST);
  motorR.addBoost(TURN_BOOST);

  double p, i = 0, d;
  double PID;
  bool cross_over = false;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  if (orientationData.orientation.x + relative_angle >= 360) {
    cross_over = true;
  }

  const double initial_angle = orientationData.orientation.x;

  double orientation = cross_over ? orientationData.orientation.x - relative_angle: orientationData.orientation.x;
  // double angle = cross_over ? global_angle : global_angle + relative_angle;
  //FIXME: Robot keeps dying when I use global angle
  double angle = orientation + relative_angle;
  double last_error = abs((orientationData.orientation.x - angle) / angle);

  while (abs(orientation - angle) > 1) {

    // Serial.print("Orientation Right: ");
    // Serial.print(orientation);
    // Serial.print("\t");
    // Serial.print(global_angle);
    // Serial.print("\t");
    // Serial.println(cross_over);

    p = abs((orientation - angle) / relative_angle);
    //i = i + p;
    //d = p - last_error;
    PID = KP_TURN * p;
    //last_error = p;
    utils::forward((PID * -speed), (PID * speed));

    if (PID <= 0.01)
      break;

    // Serial.print("PID: ");
    // Serial.println(PID);

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orientation = cross_over ? orientationData.orientation.x - relative_angle: orientationData.orientation.x;

    if (orientationData.orientation.x < initial_angle) {
      orientation += 360;
    }

    // if (orientationData.orientation.x < 1.0) {
    //   orientation += 360;
    // }
  }
  utils::resetBoost();
  utils::stopMotors();
  global_angle = utils::math::wrapAround(global_angle + relative_angle, 360);
#endif
}

void raw_left(int relative_angle, int speed) {
#ifndef MOTORSOFF
  motorL.addBoost(TURN_BOOST);
  motorR.addBoost(TURN_BOOST);

  double p, i = 0, d;
  double PID;
  bool cross_over = false;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  if (orientationData.orientation.x - relative_angle < 0) {
    cross_over = true;
  }

  const double initial_angle = orientationData.orientation.x;
  double orientation = cross_over ? orientationData.orientation.x + relative_angle: orientationData.orientation.x;
  // double angle = cross_over ? global_angle : global_angle - relative_angle;
  double angle = orientation - relative_angle;
  double last_error = abs((orientationData.orientation.x - angle) / angle);

  while (abs(orientation - angle) > 1) {
    // Serial.print("Orientation Left:  ");
    // Serial.print(orientation);
    // Serial.print("\t");
    // Serial.print(global_angle);
    // Serial.print("\t");
    // Serial.println(cross_over);

    p = abs((orientation - angle) / relative_angle);
    //i = i + p;
    //d = p - last_error;
    PID = KP_TURN * p;
    //last_error = p;
    utils::forward((PID * speed), (PID * -speed));

    if (PID <= 0.01)
      break;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orientation = cross_over ? orientationData.orientation.x + relative_angle: orientationData.orientation.x;

    if (orientationData.orientation.x > initial_angle) {
      orientation -= 360;
    }

    // if (orientationData.orientation.x < 1.0) {
    //   orientation += 360;
    // }
  }
  utils::resetBoost();
  utils::stopMotors();
  global_angle = utils::math::wrapAround(global_angle - relative_angle, 360);
#endif
}

void turn(char char_end_direction) {
  uint8_t end_direction = 0;
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
    case 0: pi_send_tag("turn_status"); PI_SERIAL.println(0.0); break;
  }
  cur_direction = end_direction;
}

void driveCM(float cm, int speed = 200, int tolerance = 10) {
  //alignAngle(100);
#if 1
  double horizontalError = abs((int)tofCalibrated(0) - (int)tofCalibrated(2)) / 2;
  double angle = abs(atan((cm * 10) / horizontalError) * (180/PI));
  if (horizontalError >= tolerance && angle >= 3 && tofCalibrated(0) < 150 && tofCalibrated(2) < 150) {

    if (tofCalibrated(0) > tofCalibrated(2)) {

      raw_right(90 - angle, SPEED);
      drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI/180))), speed);
      raw_left(90 - angle, SPEED);

    } else {
      raw_left(90 - angle, SPEED);
      drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI/180))), speed);
      raw_right(90 - angle, SPEED);
    }
  } 
  else {
    drive((cm * CM_TO_ENCODERS), speed); 
  }
#else
  drive((cm * CM_TO_ENCODERS), speed);

#endif

}


void drive(int encoders, int speed) {
#ifndef MOTORSOFF
  // bno.begin(OPERATION_MODE_IMUPLUS);
  double orientation_offset;
  motorL.addBoost(DRIVE_BOOST);
  motorR.addBoost(DRIVE_BOOST);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  orientation_offset = orientationData.orientation.x;

  int angle = 60, tofR1, tofR2; 
  utils::resetTicks();
  double p, d, i = 0;
  double p_turn, d_turn, last_difference = 0;
  double PID;
  double last_dist = abs(motorR.getTicks() / abs(encoders));
  double startX = xPos;
  double orientation;
  double orig_encoders = encoders;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  encoders = orig_encoders / cos(-orientationData.orientation.z * (2 * PI / 360));
  pi_send_data(true, true);

  while (abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders) && tofCalibrated(4) >= 50) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    encoders = orig_encoders / cos(abs(orientationData.orientation.z * (2 * PI / 360)));

    p = speed * (double) (abs(encoders) - abs(motorR.getTicks())) / abs(encoders);
    //i = i + p;
    //d = p - last_dist;
    PID = p * KP_FORWARD;
    //Serial.println(PID);

    orientation = (int) (orientationData.orientation.x - orientation_offset) % 360;

    if (orientation > 180) {
      p_turn = -(orientation - 360) - (startX - xPos);
    } else {
      p_turn = -orientation - (startX - xPos);
    }

    if(returnColor(true) == 1)
    {
      utils::stopMotors();
      unsigned int ticks = (motorR.getTicks() + motorL.getTicks()) / 2;
      utils::resetTicks();
      while(abs(motorR.getTicks()) < abs(ticks) && abs(motorL.getTicks()) < abs(ticks) && tofCalibrated(5) >= 40)
      {
        utils::forward(-speed);
      }
      pi_send_data(false, false);
      utils::resetBoost();
      utils::stopMotors();
      return;
    }
    
    //we are close enough to the target at this point, so quit the loop
    if (abs(p_turn * DRIVE_STRAIGHT_KP) <= 0.01 && PID <= 0.01)
      break;
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    //    Serial.println(speed * (double)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    utils::forward(PID - p_turn * DRIVE_STRAIGHT_KP + DRIVE_BOOST);
    angle = orientation;
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
    tofR1 = tof.readRangeSingleMillimeters() - 50; 
    tcaselect(1); 
    tofR2 = tof.readRangeSingleMillimeters() - 15; 
    shiftRight(); 
    
  } 
  
  align(tolerance);  
  */
  
  utils::stopMotors();
  pi_send_data(false, true);
  utils::resetBoost();
#else
  pi_send_data(true, true);
  delay(100);
  pi_send_data(false, true);
#endif
  
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
    right(15, SPEED);
    utils::forward(-SPEED, -SPEED);
    delay(24);
    return; 
} 

void alignCenterLR(int speed) {
  int tofR1, tofL1; 
  tofR1 = tofCalibrated(0);
  tofL1 = tofCalibrated(1);

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
  tofF1 = tofCalibrated(0);
  tofB1 = tofCalibrated(1);

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

  tofR1 = tofCalibrated(0); 
  tofR2 = tofCalibrated(1); 
         
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
    tofR1 = tof.readRangeSingleMillimeters() - 50;
    tcaselect(1);
    tofR2 = tof.readRangeSingleMillimeters() - 15;

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

unsigned int tofCalibrated(int select) 
{
  unsigned int dist = 0;
  unsigned int cal = 0;
  const unsigned int max_dist = 200;
  switch (select) 
  {
    case 0: 
    {
        tcaselect(0);
        dist = tof.readRangeSingleMillimeters();
        cal = -89.7 + (dist * 1.9) - (0.0033 * (dist * dist));
        cal = min(cal, max_dist);
        return cal;
        //accurate (50, 150), horrible < 25
    }
    case 1: 
    {
        tcaselect(1);
        dist = tof.readRangeSingleMillimeters();
        cal = (1.09 * dist) - 21.3;
        cal = min(cal, max_dist);        
        return cal;
        //accurate (50, 150), still works < 25ish
    }
    case 2: 
    {
        tcaselect(2);
        dist = tof.readRangeSingleMillimeters();
        cal = (1.03 * dist) - 9.91;
        cal = min(cal, max_dist);
        return cal;
        //accrate (50, 150), passable < 50 but not that good
    } 
    case 3: 
    { 
        tcaselect(3); 
        dist = tof.readRangeSingleMillimeters(); 
        cal = -0.848 + (0.671 * dist) + (0.00165 * dist * dist); 
        cal = min(cal, max_dist);
        return cal; 
        //decent accuracy 
      
    } 
    case 4: 
    { 
        tcaselect(4); 
        dist = tof.readRangeSingleMillimeters(); 
        cal = 3 + (0.657 * dist) + (0.00146 * dist * dist);
        cal = min(cal, max_dist);
        return cal;   
        //pretty accurate
    }
    case 5:
    {
        //TODO: calibrate 
        tcaselect(5);
        dist = tof.readRangeSingleMillimeters();
        cal = dist;
        cal = min(cal, max_dist);
        return cal;
    } 
    default:
      return -1;
  }
}

void oled_display_walls(bool walls[4])
{
#ifdef DEBUG_DISPLAY

  const char char_map[4] = {'n', 'e', 's', 'w'};
  String data = "    ";

  for(int i = 0; i < 4; i++)
  {
    if(walls[i])
      data[i] = char_map[i];
  }

  oled.println(data.c_str());
#endif
}

char dir_to_char(uint8_t cur_dir)
{
  switch(cur_dir)
  {
    case n:
      return 'n';
    case e:
      return 'e';
    case s:
      return 's';
    case w:
      return 'w';
    default:
      return 'n';
  }
  return 'n';
}

//#define TEST
#ifndef TEST

int clear_oled_counter = 0;

void loop() 
{

  bool* arr = get_tof_vals(150);

  // //n e s w
  bool walls[4] = {arr[4], arr[0] && arr[1], arr[5], arr[2] && arr[3]};
  // not wrapped around and stuff 
  oled_display_walls(walls);

  //acceleration_position();
  //pi_read_data();
  /*
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE); */
  /*
  display_data(accelerometerData);
  pi_send_data(accelerometerData);
  display_data(gyroData);
  pi_send_data(gyroData);
  pi_read_data();
  Serial.println();
  */
  /*
  byte test = get_tof_vals(100);
  Serial.print("Tof: ");
  Serial.println(test, BIN);*/
  //send_tof_vals(test);

  //Serial.println("hi");
  //Serial.println(get_tof_vals(100));
  //Serial.println("hi");
  //pi_read_data();

  //oled.println("test");
//   // alignAngle(100, 0, 1); 
//   //delay(500);
//   // driveCM(30, 200, 0);
//   // delay(1000);
//   pi_send_tag("dir");
//   PI_SERIAL.println(cur_direction);

  while(PI_SERIAL.available())
    pi_read_data();

  oled.print("dir: ");
  oled.println(dir_to_char(cur_direction));


  //checkpoint detection
  pi_send_tag("CP");
  PI_SERIAL.println("0");

#ifdef DEBUG_DISPLAY
  oled.setCursor(0, 0);
  clear_oled_counter++;
  if(clear_oled_counter > 50)
  {
    oled.clearDisplay();
    clear_oled_counter = 0;
  }
#endif

  delay(100);

}
#else


void loop()
{
  // right(90, 100);
  // right(90, 100);
  // delay(1000);  
  // left(90, 100);
  // left(90, 100);
  // left(90, 100);
  // delay(1000); 
  //utils::forward(255);
  //delay(1000);

  driveCM(27, 110);
  delay(100);
  //Serial.println(returnColor());
}

#endif
