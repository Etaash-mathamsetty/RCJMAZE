//#define FAKE_ROBOT
//#define FAKE_SERIAL
#define DEBUG_DISPLAY
// #define MOTORSOFF
#define TEST
// #define ALIGN_ANGLE
#define NO_PI //basic auto when no raspberry pi (brain stem mode)

//define: debug display, motorsoff, test, comment out all others if you want to calibrate tofs 

#include "Motors.h"
#include "utils.h"
#include "common.h"

using namespace utils;


void setup() {
//#ifndef FAKE_SERIAL
  PI_SERIAL.begin(115200);

  Serial.begin(9600);
  setMotors(&motorR, &motorL);
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
    if(!tof.init()){
      Serial.print("Bruh :( sensor "); 
      Serial.print(i); 
      Serial.println(" is broken"); 
    } else {
      Serial.print("Yay! sensor "); 
      Serial.print(i); 
      Serial.println(" is init");
    }
 
    //tof.setTimeout(500);
    //tof.startContinuous(); 
  }
  oled.println("TOF init done!");
  myservo.attach(servopin);  
  myservo2.attach(servopin2); 
  resetServo();
  myservo.write(175);
  delay(100);
  myservo.write(170);
  delay(100);
  myservo.write(175);
  oled.println("Servo reset");
  tcaselect(6);
  if (!tcs.begin())
  {
    Serial.println("color sensor init fail!");
  }

  oled.println("TCS init done!");
  
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  analogWrite(2, 50);  
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

//forward_status[0] = is forward command runnning
//forward_status[1] = true = no black tile
//forward_status[2] = true = failed for other reason, (wall in front for ex)
void pi_send_data(bool forward, bool black_tile, bool failed = false) {
  pi_send_tag("forward_status");

  PI_SERIAL.print((double)forward);
  PI_SERIAL.print(',');
  PI_SERIAL.print((double)black_tile);
  PI_SERIAL.print(',');
  PI_SERIAL.println((double)failed);
}

void pi_send_data(bool walls[4]) {
  // pi_send_tag("NW");
  // PI_SERIAL.println(walls[math::wrapAround((int) (n - cur_direction), 4)]);
  // pi_send_tag("EW");
  // PI_SERIAL.println(walls[math::wrapAround((int) (e - cur_direction), 4)]);
  // pi_send_tag("SW");
  // PI_SERIAL.println(walls[math::wrapAround((int) (s - cur_direction), 4)]);
  // pi_send_tag("WW");
  // PI_SERIAL.println(walls[math::wrapAround((int) (w - cur_direction), 4)]);

  pi_send_tag("W");
  PI_SERIAL.print(walls[math::wrapAround((int)n - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.print(walls[math::wrapAround((int)e - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.print(walls[math::wrapAround((int)s - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.println(walls[math::wrapAround((int)w - (int)cur_direction, 4)]);
  // oled.print(walls[math::wrapAround((int)n - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.print(walls[math::wrapAround((int)e - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.print(walls[math::wrapAround((int)s - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.println(walls[math::wrapAround((int)w - (int)cur_direction, 4)]);
}

bool* get_tof_vals(double threshold) {

  static bool arr[TOF_NUMBER + 1];
  uint32_t reading = 0;

  memset(arr, 0, ARRAY_SIZE(arr));

  //Serial.println("start");

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {
    reading = tofCalibrated(i);

    if (reading < threshold - ((i >= 4) ? 0 : 55)) {
      arr[i] = true;
      Serial.print("Wall: ");
    } else {
      Serial.print("No Wall: ");
    }
    Serial.println(reading);
  }

  //Serial.println("end");
  //Serial.println("passed");

  return arr;
}

void driveCM(float,int,int);
void left(int, int, bool);
void right(int, int, bool);

void pi_read_vision() {
  String data = "";
  char ch = 0;
  int num = 0;

  while(PI_SERIAL.available() && ch != '\n') 
  {
    ch = PI_SERIAL.read();
    data += ch;
    delay(5);
  }
  data.trim();
  data.toLowerCase();
  data += '\n';
  String cur_cmd = "";
  Serial.println(data);

  for (char c : data) {
    if (c == 'd') {
      cur_cmd.remove(0);
      cur_cmd += c;
    } else if (c == 'l') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        if(num > 0)
          left(90, 100, false);
        kitDrop(num, 'r');
        if(num > 0)
          right(90, 100, false);
        cur_cmd.remove(0);

        if(num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c == 'q') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        if(num > 0)
          right(90, 100, false);
        kitDrop(num, 'r');
        if(num > 0)
          left(90, 100, false);
        cur_cmd.remove(0);

        if(num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    }
    else if (c >= '0' && c <= '9') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd')
      {
        num = c - '0';
      }
    }
  }

  //get_tof_vals(wall_tresh);
}

int returnColor(bool only_black = false);

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
  int num = 0;
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
    if (c == 'g' || c == 'f' || c == 't' || c == 'd' || c == 'r') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'g' || cur_cmd[0] == 'f') {
          Serial.println("FORWARD");
          oled.println("forward");
          driveCM(30, 110, 1);
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
      }
      cur_cmd.remove(0);
      cur_cmd += c;
    }
    else if (c >= '0' && c <= '9') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd')
      {
        num = c - '0';
      }
    }
    else if (c == 'l') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        if(num > 0)
          left(90, 100, false);
        kitDrop(num, 'r');
        if(num > 0)
          right(90, 100, false);
        cur_cmd.remove(0);

        if(num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c == 'q') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        if(num > 0)
          right(90, 100, false);
        kitDrop(num, 'r');
        if(num > 0)
          left(90, 100, false);
        cur_cmd.remove(0);

        if(num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    }
    else if (c == 'e' || c == 'w' || c == 's' || c == 'n') {
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
          driveCM(30, 110, 1);
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
    else if (c == '\n' || c == '\0') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'g' || cur_cmd[0] == 'f') {
          Serial.println("FORWARD");
          oled.println("forward");
          driveCM(30, 110, 1);
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
        if (cur_cmd[0] == 'r')
        {
          bool* arr = get_tof_vals(wall_tresh);

          //oled.println("test2");
          
          // Serial.print("Tof Vals: ");
          // Serial.println(vals);

          // //n e s w
          bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };
          // not wrapped around and stuff 
          //oled_display_walls(walls);
          //  this is wrapped
          pi_send_data(walls);

          //checkpoint detection
          //pi_send_tag("CP");
          //PI_SERIAL.println(float(returnColor() == 2));

          pi_send_tag("CP");
          PI_SERIAL.println("0.0");

          //Serial.println("sending wall data");
        }
      }
      cur_cmd = "";
    }
  }
}

// FIXME: Blue being detected as black
int returnColor(bool only_black = false){
    uint16_t r, g, b, c = 0;
    int silver_detect = 0;
    int black_detect = 0;
    int blue_detect = 0;
    tcaselect(6);
    tcs.getRawData(&r, &g, &b, &c);
    const int persistance_count = 14;
#if 0
    Serial.print("red:");
    Serial.print(r);
    Serial.print(",");
    Serial.print("green:");
    Serial.print(g);
    Serial.print(",");
    Serial.print("blue:");
    Serial.print(b);
    Serial.print(",");
    Serial.print("clear:");
    Serial.print(c); 
    Serial.print(",");
    Serial.print("r/c:");
    Serial.print((float)r/c * 100.0);
    Serial.print(",");
    Serial.print("g/c:");
    Serial.print((float)g/c * 100.0);
    Serial.print(",");
    Serial.print("r/g:");
    Serial.print((float)r/g * 100.0);
    Serial.print(",");
    Serial.print("b/r:");
    Serial.println((double)b/r * 100.0);
#endif
    // oled.println(r);
    // delay(50);
    // oled.println(g);
    // delay(50);
    // oled.println(b);
    // delay(50);
    // oled.println(c);
    // delay(2000);

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    for (int i = 0; i <= persistance_count; i++) {
      if (c < 320  && c >= 180 && (r / (double)g) * 100.0 >= 116) {
   //   silver_persistance++;
      // Serial.println("silver detected"); 
      // oled.println("silver");
      //pi_send_tag("color");
      //PI_SERIAL.println("silver");
        silver_detect++; //change later
      }
      if(c < 110 && (double) b/r < 1 && abs(orientationData.orientation.z) < 3.5){ 
      // Serial.println("black detected"); 
      //oled.println("black");
      //pi_send_tag("color");
      //PI_SERIAL.println("black");  
        black_detect++;  
      }
      if((double) b/r >= 1.5) {
        blue_detect++;
      }
    }
    if (blue_detect >= persistance_count && !only_black) {
      stopMotors();
      delay(5000);
      return 3;
    } else if (black_detect >= persistance_count) {
      return 1;
    } else if (silver_detect >= persistance_count && !only_black) {
      return 2;
    } else {
      return 0;
    }

    // if(b > (r * 2.5) && !only_black){ 
    //   // Serial.println("blue detected");
    //   // oled.println("blue");  
    //   //pi_send_tag("color"); 
    //  //PI_SERIAL.println("blue"); 
    //   return 0; //change later
    // }
  return 0;  
}


void backup_align(int speed, int time) {

  while (tofCalibrated(5) > 40) {
    utils::forward(-speed);
  }
  utils::resetTicks();
  utils::stopMotors();
  delay(100);

  const int tstart = millis();
  while(millis() - tstart < time) {
    utils::forward(-speed);
  }
  utils::stopMotors();

  bno.begin(OPERATION_MODE_IMUPLUS);

  // while(abs(motorR.getTicks() > 0) {
  //   utils::forward(speed);
  // }
  // utils::stopMotors();
}

void right(int relative_angle, int speed, bool turn_status = true) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // double orientation = orientationData.orientation.x;
  
  if(turn_status)
  {
    pi_send_tag("turn_status");
    PI_SERIAL.println(1.0);
  }

  
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


 
  if(tofCalibrated(5) <= wall_tresh - 30)
  {
    backup_align(SPEED, 1200);
    // while(tofCalibrated(5) >= 70)
    // {
    //   forward(-speed);
    // }
    while(tofCalibrated(5) <= 40)
    {
      forward(speed);
    }
    stopMotors();
  }

  if(turn_status)
  {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
}

void left(int relative_angle, int speed, bool turn_status = true) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double orientation = orientationData.orientation.x;

  if (abs(orientationData.orientation.x - global_angle) > 180)
    orientation = orientationData.orientation.x - 360;

  int offset = relative_angle - (int) (orientationData.orientation.x) % relative_angle;

  if (offset < relative_angle / 2) {
    relative_angle -= offset;
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(1.0);
  }

  
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

  if(tofCalibrated(5) <= wall_tresh - 30)
  {
    backup_align(SPEED, 1200);
    // while(tofCalibrated(5) >= 70)
    // {
    //   forward(-speed);
    // }
    while(tofCalibrated(5) <= 40)
    {
      forward(speed);
    }
    stopMotors();
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
}

void raw_right(double relative_angle, int speed) {

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

  double tstart = millis();

  while (abs(orientation - angle) > 1) {

    // Serial.print("Orientation Right: ");
    // Serial.print(orientation);rightright
    // Serial.print("\t");
    // Serial.print(global_angle);
    // Serial.print("\t");
    // Serial.println(cross_over);

    p = abs((orientation - angle) / relative_angle);
    //i = i + p;
    //d = p - last_error;
    PID = KP_TURN * p;
    //last_error = p;
    // while(PI_SERIAL.available())
    // {
    //   stopMotors();
    //   pi_read_vision();
    //   oled.println("detected");
    // }

    if (millis() - tstart < 3000) {
      forward((PID * -speed), (PID * speed));
    } else {
      forward((PID * -speed) - TURN_BOOST, (PID * speed) + TURN_BOOST);
    }

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
  resetBoost();
  stopMotors();
  global_angle = math::wrapAround(global_angle + relative_angle, 360);
#endif
}

void raw_left(double relative_angle, int speed) {
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

  double tstart = millis();

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
    last_error = p;
    // while(PI_SERIAL.available())
    // {
    //   stopMotors();
    //   pi_read_vision();
    //   oled.println("detected");
    // }

    if (millis() - tstart < 3000) {
      forward((PID * speed), (PID * -speed));
    } else {
      forward((PID * speed) + TURN_BOOST, (PID * -speed) - TURN_BOOST);
    }

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
  resetBoost();
  stopMotors();
  global_angle = math::wrapAround(global_angle - relative_angle, 360);
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

  switch ((int) cur_direction - (int)end_direction) {
    case -3:
    case 1: left(90, SPEED); break;
    case -1:
    case 3: right(90, SPEED); break;
    case 2:
    case -2: left(90, SPEED, false); left(90, SPEED); break;
    default: Serial.println("invalid");
    case 0: pi_send_tag("turn_status"); PI_SERIAL.println(0.0); break;
  }

  cur_direction = end_direction;
}

void alignAngle(bool, int x = 5);

void driveCM(float cm, int speed = 200, int tolerance = 10) {
  //kitDrop(1);

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();  
    alignAngle(true);
  }
  
  pi_send_data(true, true);
#if 1
  const float mult_factor = 1.0;
  unsigned int right = (tofCalibrated(2) + tofCalibrated(3))/2;
  unsigned int left = (tofCalibrated(0) + tofCalibrated(1))/2;
  const float half_chassis = 75;
  const double target_dist_from_wall = (300.0 - half_chassis * 2) / 2.0;

  double horizontalError = abs((int)left - (int)right) / 2;
  double angle = abs(atan((cm * 10.0) / horizontalError) * (180.0/PI));
  //angle = max(angle, 90 - 30);
  oled.println(angle * mult_factor);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  bool optimal_alignment = horizontalError >= tolerance && abs(orientationData.orientation.z) < 12;
  if (optimal_alignment && left <= wall_tresh && right <= wall_tresh) {

    if (left < right) {

      raw_right(90 - min(90, angle * mult_factor), SPEED);

      if(tofCalibrated(4) > wall_tresh)
        drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI/180))), speed);
      else
      {
        Serial.println("achievement unlocked! How did we get here?");
        oled.clearDisplay();
        oled.println("achievement unlocked!");
        oled.println("How did we get here?");
        while(tofCalibrated(4) >= 70)
        {
          forward(speed * 0.7);
        }
        stopMotors();

        raw_left(90 - min(90, angle * mult_factor), SPEED);
        pi_send_data(false, true, true);
        return;
      }

      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      if (abs(orientationData.orientation.z) < 12)
        raw_left(90 - min(90, angle * mult_factor), SPEED);
      // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      // raw_left(orientationData.orientation.x, SPEED);

    } else {
      raw_left(90 - min(90, angle * mult_factor), SPEED);

      if(tofCalibrated(4) > wall_tresh)
        drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI/180))), speed);
      else {
        Serial.println("achievement unlocked! How did we get here?");
        oled.clearDisplay();
        oled.println("achievement unlocked!");
        oled.println("How did we get here?");
        while(tofCalibrated(4) >= 70)
        {
          forward(speed * 0.7);

        }
        
        stopMotors();
        raw_right(90 - min(90, angle * mult_factor), SPEED);
        
        pi_send_data(false, true, true);
        return;
      }

      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      if (abs(orientationData.orientation.z) < 12)
        raw_right(90 - min(90, angle * mult_factor), SPEED);
      // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      // raw_right(360-orientationData.orientation.x, SPEED);
    }
  } else if (optimal_alignment && left <= wall_tresh && right >= wall_tresh && abs(left - target_dist_from_wall) > 20.0) {
    oled.println("single left wall");

    double angle = 90.0;

    if (left != target_dist_from_wall)
      angle = atan((cm * 10.0)/(abs(left - target_dist_from_wall))) * (180/PI);

    if(left - target_dist_from_wall > 0.0)
    {
      raw_left(90.0 - angle, SPEED);
    }
    else if(left - target_dist_from_wall < 0.0)
    {
      raw_right(90.0 - angle, SPEED);
    }
    oled.print("Angle: ");
    oled.println(90.0 - angle);
    // oled.print(atan((cm * 10)/(150 - (half_chassis + left))));

    // if(tofCalibrated(4) > wall_tresh)
    drive(cm / sin(angle * (PI/180)) * CM_TO_ENCODERS, speed);
    // else {
    //   Serial.println("achievement unlocked! How did we get here?");
    //   oled.clearDisplay();
    //   oled.println("achievement unlocked!");
    //   oled.println("How did we get here?");
    //   while(tofCalibrated(4) >= 70)
    //   {
    //     forward(SPEED * 0.7);
    //   }
    //   stopMotors();

    //   raw_left(90 - atan((cm * 10)/(150 - (half_chassis + left))), SPEED);
    //   pi_send_data(false, true, true);
    //   return;
    // }
    if(left - target_dist_from_wall > 0.0)
    {
      raw_right(90.0 - angle, SPEED);
    }
    else if (left - target_dist_from_wall < 0.0)
    {
      raw_left(90.0 - angle, SPEED);
    }

  } else if (optimal_alignment && left >= wall_tresh && right <= wall_tresh && abs(right - target_dist_from_wall) > 20.0) {
    oled.println("single right wall");
    oled.print("Angle: ");

    double angle = 90.0;
    
    if (right != target_dist_from_wall) 
      angle = atan((cm * 10.0)/(abs(right - target_dist_from_wall))) * (180/PI);

    oled.println(90.0 - angle);

    if(right - target_dist_from_wall > 0.0)
    {
      raw_right(90.0 - angle, SPEED);
    }
    else if (right - target_dist_from_wall < 0.0)
    {
      raw_left(90.0 - angle, SPEED);
    }

    // if(tofCalibrated(4) > wall_tresh)
    drive(cm / sin(angle * (PI/180)) * CM_TO_ENCODERS, speed);
    // else
    // {
    //   Serial.println("achievement unlocked! How did we get here?");
    //   oled.clearDisplay();
    //   oled.println("achievement unlocked!");
    //   oled.println("How did we get here?");
    //   while(tofCalibrated(4) >= 70)
    //   {
    //     forward(SPEED * 0.7);
    //   }
    //   stopMotors();

    //   raw_right(90 - atan((cm * 10)/(150 - (half_chassis + right)) * (180/PI)), SPEED);
    //   pi_send_data(false, true, true);
    //   return;
    // }
    
    if(right - target_dist_from_wall > 0.0)
    {
      raw_left(90.0 - angle, SPEED);
    }
    else if (right - target_dist_from_wall < 0.0)
    {
      raw_right(90.0 - angle, SPEED);
    }
  }
  // else if (left <= wall_tresh - 30 && (150 - (80 + left)) != 0) {
  //   double angle = atan(cm / (150 - (80 + left)));
  //   angle *= (180/PI);
  //   raw_right(angle, speed);
  //   drive((cm * CM_TO_ENCODERS), speed); 
  //   raw_left(angle, speed);
  // }
  // else if (right <= wall_tresh - 30 && (150 - (80 + right)) != 0) {
  //   double angle = atan(cm / (150 - (80 + right)));
  //   angle *= (180/PI);
  //   raw_left(angle, speed);
  //   drive((cm * CM_TO_ENCODERS), speed); 
  //   raw_right(angle, speed);
  // }
  // else if (left <= wall_tresh && left - 60 != 0) {
  //   double angle = atan((cm * 10) / (left - 60));
  //   if(angle > 0)
  //     angle = min(angle, 10);
  //   else
  //     angle = max(angle, -10);
  //   if(angle < 0)
  //     raw_left(-angle * (180/PI), speed);
  //   else  
  //     raw_right(angle * (180/PI), speed);
  //   if(tofCalibrated(4) > wall_tresh)
  //     drive((cm * CM_TO_ENCODERS), speed); 
  //   else
  //   {

  //     Serial.println("achievement unlocked! How did we get here?");
  //     oled.clearDisplay();
  //     oled.println("achievement unlocked!");
  //     oled.println("How did we get here?");
      
  //     while(tofCalibrated(4) >= 60)
  //     {
  //       forward(speed * 0.7);
  //     }

  //     stopMotors();

  //     pi_send_data(false, true, true);
  //     if(angle < 0)
  //       raw_right(-angle * (180/PI), speed);
  //     else  
  //       raw_left(angle * (180/PI), speed);
  //     return;
  //   }
  //   if(angle < 0)
  //     raw_right(-angle * (180/PI), speed);
  //   else  
  //     raw_left(angle * (180/PI), speed);
  // }
  // else if (right <= wall_tresh && right - 60 != 0) {
  //   double angle = atan((cm * 10) / (right - 60));
  //   if(angle > 0)
  //     angle = min(angle, 10);
  //   else
  //     angle = max(angle, -10);
  //   if(angle < 0)
  //     raw_right(-angle * (180/PI), speed);
  //   else  
  //     raw_left(angle * (180/PI), speed);
  //   if(tofCalibrated(4) > wall_tresh)
  //     drive((cm * CM_TO_ENCODERS), speed); 
  //   else
  //   {
  //     Serial.println("achievement unlocked! How did we get here?");
  //     oled.clearDisplay();
  //     oled.println("achievement unlocked!");
  //     oled.println("How did we get here?");
      
  //     while(tofCalibrated(4) >= 60)
  //     {
  //       forward(speed * 0.7);
  //     }

  //     stopMotors();

  //     pi_send_data(false, true, true);
  //     if(angle < 0)
  //       raw_left(-angle * (180/PI), speed);
  //     else  
  //       raw_right(angle * (180/PI), speed);
  //     return;
  //   }
  //   if(angle < 0)
  //     raw_left(-angle * (180/PI), speed);
  //   else  
  //     raw_right(angle * (180/PI), speed);
  // }
  else {
    drive((cm * CM_TO_ENCODERS), speed); 
  }
#else
  drive((cm * CM_TO_ENCODERS), speed);

#endif

  if(tofCalibrated(4) <= wall_tresh)
  {
    while(tofCalibrated(4) >= 70)
    {
      forward(speed * 0.7);

    }
    stopMotors();
  }

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    alignAngle(true);
  }

  //pause for blue if detected
  returnColor();

  //WARN: drive function can no longer be used on it's own (when communicating with stereo pi) !!!!!
  pi_send_data(false, true);
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
  resetTicks();
  double p, d, i = 0;
  double p_turn, d_turn, last_difference = 0;
  double PID;
  double last_dist = abs(motorR.getTicks() / abs(encoders));
  double startX = xPos;
  double orientation;
  double orig_encoders = encoders;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double start_pitch = orientationData.orientation.z;
  bool ramp_detect = false;
  bool down_ramp_detect = false;
  double start_ramp = INT_MAX;
  uint32_t tstart = millis();
  uint32_t tramp = UINT32_MAX;
  uint32_t tramp_up = UINT32_MAX;
  const uint32_t ramp_up_elapsed = 800;
  const uint32_t ramp_elapsed = 1200;
  // encoders = orig_encoders / cos(-orientationData.orientation.z * (2 * PI / 360));


  while ((abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders) && (tofCalibrated(4) >= 75)) || ramp_detect || down_ramp_detect) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // encoders = orig_encoders / cos(abs(orientationData.orientation.z * (2 * PI / 360)));

    if (abs(start_pitch) < 4 && orientationData.orientation.z - start_pitch < -12 && !ramp_detect) {
      tramp_up = millis();
      ramp_detect = true;
    }

    if ((int64_t) millis() - (int64_t) tramp_up > 150 && orientationData.orientation.z - start_pitch < -12) {
      // start_ramp = millis();
      oled.println("ramp detected!");
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      while (orientationData.orientation.z - start_pitch < -3) {
        utils::forward(-SPEED * 0.75);
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      }
      utils::resetTicks();
      while (abs(motorR.getTicks()) < 7 * CM_TO_ENCODERS) {
        utils::forward(-SPEED * 0.75);
      }
      utils::stopMotors();
      alignAngle(false);
      alignAngle(false);
      alignAngle(false);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      while(orientationData.orientation.z >= -3.5) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        utils::forward(SPEED);
      }
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      const double angle_start = orientationData.orientation.z;
      while(abs(orientationData.orientation.z - angle_start) < 4) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        utils::forward(SPEED);
      }
      utils::stopMotors();
      utils::resetTicks();
      while (abs(motorR.getTicks()) < 10 * CM_TO_ENCODERS) {
        utils::forward(SPEED * 0.7);
      }
      utils::stopMotors();
      return;
    }

    if (abs(start_pitch) < 4 && orientationData.orientation.z - start_pitch > 3.5 && !down_ramp_detect) {
      tramp = millis();
      down_ramp_detect = true;
      oled.println("down ramp detected 1");
    }

    if ((int64_t) millis() - (int64_t) tramp > ramp_elapsed) {
        
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      while (abs(orientationData.orientation.z) < 5 && tofCalibrated(5) > 40) {
        oled.println("backward 1");
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        utils::forward(-SPEED * 0.75);
      }
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      while (orientationData.orientation.z > 0 && tofCalibrated(5) > 40) {
        oled.println("backward 2");
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        utils::forward(-SPEED * 0.75);
      }
      utils::resetTicks();
      while(abs(motorR.getTicks()) < 10 * CM_TO_ENCODERS) {
        utils::forward(-SPEED * 0.75);
      }
      utils::stopMotors();
      alignAngle(false);
      alignAngle(false);
      alignAngle(false);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      while(orientationData.orientation.z <= 3.5) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        utils::forward(SPEED);
      }
      while(orientationData.orientation.z > 3.5) {
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        utils::forward(SPEED * 0.65);
      }
      utils::stopMotors();
      utils::resetTicks();
      while (abs(motorR.getTicks()) < 10 * CM_TO_ENCODERS) {
        utils::forward(SPEED * 0.65);
      }
      utils::stopMotors();
      return;
    }

    // if (millis() - start_ramp > 75 && ramp_detect) {
    //   encoders = motorR.getTicks() + (30 * CM_TO_ENCODERS - motorR.getTicks()) / cos(abs(orientationData.orientation.z * (PI / 180))) + 5 * CM_TO_ENCODERS;
    //   speed *= 0.7;
    //   ramp_detect = false;
    // }

    // if (orientationData.orientation.z > 12 && abs(start_pitch) > -3) {
    //   speed *= 0.7;
    //   down_ramp_detect = true;
    // }

    // if (orientationData.orientation.z <= 12 && down_ramp_detect) {
    //   speed *= (1.0/0.7);
    // }

    // if (orientationData.orientation.z <= 7 && !ramp_detect) {
    //   encoders = orig_encoders;
    // }

    // oled.print("encoders: ");
    // oled.println(encoders);
    // oled.clearDisplay();

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
      while(/* motorR.getTicks() > 0 && */ motorL.getTicks() > 0 && tofCalibrated(5) >= 40)
      {
        forward(-speed);
      }
      stopMotors();
      pi_send_data(false, false);
      resetBoost();
      return;
    }

    
    //we are close enough to the target at this point, so quit the loop
    if ( /* abs(p_turn * DRIVE_STRAIGHT_KP) <= 0.01 && */ PID <= 0.01)
      break;
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    // Serial.println(speed * (double)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    if (abs(orientationData.orientation.z) < 5) {
      while(PI_SERIAL.available()) {
        int right_ticks = motorR.getTicks();
        int left_ticks = motorL.getTicks();
        stopMotors();
        pi_read_vision();
        oled.println("detected");
        motorR.getTicks() = right_ticks;
        motorL.getTicks() = left_ticks;
      }
    }


    if (!down_ramp_detect) {
      forward((millis() - tstart  < 5000) ? PID : 210);
    } else {
      forward(PID * 0.25 * ((ramp_elapsed - (millis() - tramp)) / ramp_elapsed));
    }
    angle = orientation;
  } 
  //correct horizontal error when inside of hallway 
/*  if(tofR < 175 && tofL < 175){ 
            
      while(tofR - tofL > tolerance){ 
         right(angle, SPEED); 
         forward(SPEED, SPEED); 
         delay(150); 
         left(angle, SPEED); 
         forward(-SPEED, -SPEED);  
         delay(130); 

     } 
     while(tofL - tofR > tolerance){ 
         left(angle, SPEED); 
         forward(SPEED, SPEED); 
         delay(150); 
         right(angle, SPEED); 
         forward(-SPEED, -SPEED);  
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
  
  stopMotors();
  resetBoost();
#else
  pi_send_data(true, true);
  delay(100);
  pi_send_data(false, true);
#endif
  
}

int closestToDirection(double num) {
  for (int i = 0; i <= 360; i+=90) {
    if (abs(num - i) <= 45) {
      return i;
    }
  }
}

void alignAngle(bool reset, int tolerance = 5) {
  int tofR1, tofR2; 
  int tofR3, tofR4;
  int lnum = 1, rnum = 0;
  const float kP = 0.7;
  addBoost(TURN_BOOST - 5);


  tofR1 = tofCalibrated(0); 
  tofR2 = tofCalibrated(1); 
  tofR3 = tofCalibrated(2); 
  tofR4 = tofCalibrated(3);
  Serial.print(tofR1);
  Serial.print(" ");
  Serial.print(tofR2);
  Serial.print(" ");
  Serial.print(tofR3);
  Serial.print(" ");
  Serial.println(tofR4);
       
  if (tofR1 >= wall_tresh || tofR2 >= wall_tresh) {
    lnum = 2;
    rnum = 3;
  }

  if ((tofR3 >= wall_tresh || tofR4 >= wall_tresh) && (tofR1 >= wall_tresh || tofR2 >= wall_tresh)) {
    Serial.println("too high values");
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    double new_angle = closestToDirection(orientationData.orientation.x);

    if (orientationData.orientation.x  - new_angle < 0 /* && abs(orientationData.orientation.x) */) {
      raw_right(abs(orientationData.orientation.x - new_angle), SPEED - 20);
    } else {
      raw_left(abs(orientationData.orientation.x - new_angle), SPEED - 20);
    } 
    return;
  } 

  float len = abs((int)tofCalibrated(lnum) - (int)tofCalibrated(rnum));
  Serial.print("Length: ");
  Serial.println(len);

  if(len <= tolerance) {
    //Serial.println("return");
    return;
  }

  //const int width = TOF_DISTANCE;
  //const int angle = atan(width/len) * (180/PI);

  while(abs(len) >= tolerance) {
    forward(len * kP, -len * kP);
    //Serial.print("Speed: ");
    //Serial.println(len * kP);
    if (abs(len * kP) <= 6) {
      break;
    }
    len = (int)tofCalibrated(lnum) - (int)tofCalibrated(rnum);
  }

  resetBoost();
  stopMotors();


  if (reset) {
    bno.begin(OPERATION_MODE_IMUPLUS);
    oled.println("BNO has reset!");
    delay(200);
  }
}

unsigned int _tofRawValue(int select) {
  tcaselect(select);
  return tof.readRangeSingleMillimeters();
}

unsigned int _tofCalibrated(int select) 
{
  unsigned int dist = 0;
  unsigned int cal = 0;
  const unsigned int max_dist = 250;
  switch (select) 
  {
    case 0: 
    {
        tcaselect(0);
        cal = tof.readRangeSingleMillimeters();
        cal = min(cal, max_dist);
        return cal; 
        //wowzers very cool 
    }
    case 1: 
    {
        tcaselect(1);
        cal = tof.readRangeSingleMillimeters() - 40;
        cal = min(cal, max_dist);        
        return cal; 
        //pretty good 6/14 
    }
    case 2: 
    {
        tcaselect(2);
        dist = tof.readRangeSingleMillimeters();
        if(dist <= 80){
        cal = (1.39 *dist) - 57.3; 
        }
        else
        {
          cal = (0.925 * dist ) - 22.5; 
        }
        
        cal = min(cal, max_dist);
        return cal;
        //calibrated 6/14 
    } 
    case 3: 
    { 
        tcaselect(3);
        cal = tof.readRangeSingleMillimeters();
        cal = min(cal, max_dist);
        return cal;
        //not in need of calibration 6/14 
      
    } 
    case 4: 
    { 
        tcaselect(4); 
        cal = tof.readRangeSingleMillimeters();
        cal = min(cal, max_dist);
        return cal;   
        //not in need of calibration 6/14 
    }
    case 5:
    {
        //TODO: calibrate (low priority) 
        tcaselect(5);
        cal = tof.readRangeSingleMillimeters();;
        cal = min(cal, max_dist);
        return cal; 
        //6/14 essentially fine... 
    } 
    default:
      return -1;
  }
}

unsigned int tofCalibrated(int select)
{
  uint32_t dist = 0;
  for(int n = 0; n < 2; n++)
  {
    dist += _tofCalibrated(select);
  }
  dist /= 2;
  //oled.println(dist);
  return dist;
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

  bool* arr = get_tof_vals(wall_tresh);

  // //n e s w
  bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };
  // not wrapped around and stuff 
  oled_display_walls(walls);
  //acceleration_position();
  //pi_read_data();
  
  while(PI_SERIAL.available())
  {
    //pi_read_vision();
    pi_read_data();
  }

  oled.print("dir: ");
  oled.println(dir_to_char(cur_direction));

#ifdef DEBUG_DISPLAY
  oled.setCursor(0, 0);
  clear_oled_counter++;
  if(clear_oled_counter > 5)
  {
    oled.clearDisplay();
    oled.clear();
    clear_oled_counter = 0;
  }
#endif

  delay(100);

}
#else


void loop()
{


  #ifndef NO_PI
  #ifndef ALIGN_ANGLE

  int clear_oled_counter = 0;

  for (int i = 0; i <= 1; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(_tofCalibrated(i));
    Serial.print(", ");
    
  }
  Serial.println();


  int r,g,b,c;
  tcaselect(6);
  tcs.getRawData(&r, &g, &b, &c);
  oled.println();
  oled.print(r);
  oled.print(" ");
  oled.print(g);
  oled.print(" ");
  oled.print(b);
  oled.println(" ");
  oled.print(c);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // Serial.print("Orientation X:");
  // Serial.println(orientationData.orientation.x);


  oled.setCursor(0, 0);
  clear_oled_counter++;
  if(clear_oled_counter > 5)
  {
    oled.clearDisplay();
    clear_oled_counter = 0;
  }
  delay(200);
  #else
    alignAngle(true);
    delay(1000);
  #endif


  // kitDrop(2, 'r'); 
  // delay(1000);
  // kitDrop(2, 'l'); 
  // delay(1000);

  
  

  #else

  static int clear_oled_counter = 0;

  bool* arr = get_tof_vals(wall_tresh);

  // // //n e s w
  bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };
   // not wrapped around and stuff 
  oled_display_walls(walls);

  if(!walls[0])
  {
    driveCM(32, 110);
  }
  else if(!walls[1])
  {
    right(90, SPEED);
  }
  else if(!walls[3])
  {
    left(90, SPEED);
  }
  else if(!walls[2])
  {
    right(180, SPEED);
  }

  oled.setCursor(0, 0);
  clear_oled_counter++;
  if(clear_oled_counter > 5)
  {
    oled.clearDisplay();
    clear_oled_counter = 0;
  }

  #endif

  
}

#endif
