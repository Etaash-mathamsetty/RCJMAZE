//#define FAKE_ROBOT
//#define FAKE_SERIAL
#define DEBUG_DISPLAY
//#define MOTORSOFF
// #define TEST
// #define NO_PI //basic auto when no raspberry pi (brain stem mode)

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
  utils::myservo.attach(utils::servopin); 
  utils::resetServo();
  utils::myservo.write(180);
  delay(100);
  utils::myservo.write(170);
  delay(100);
  utils::myservo.write(180);
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
  // PI_SERIAL.println(walls[utils::math::wrapAround((int) (n - cur_direction), 4)]);
  // pi_send_tag("EW");
  // PI_SERIAL.println(walls[utils::math::wrapAround((int) (e - cur_direction), 4)]);
  // pi_send_tag("SW");
  // PI_SERIAL.println(walls[utils::math::wrapAround((int) (s - cur_direction), 4)]);
  // pi_send_tag("WW");
  // PI_SERIAL.println(walls[utils::math::wrapAround((int) (w - cur_direction), 4)]);

  pi_send_tag("W");
  PI_SERIAL.print(walls[utils::math::wrapAround((int)n - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.print(walls[utils::math::wrapAround((int)e - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.print(walls[utils::math::wrapAround((int)s - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.println(walls[utils::math::wrapAround((int)w - (int)cur_direction, 4)]);
  // oled.print(walls[utils::math::wrapAround((int)n - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.print(walls[utils::math::wrapAround((int)e - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.print(walls[utils::math::wrapAround((int)s - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.println(walls[utils::math::wrapAround((int)w - (int)cur_direction, 4)]);
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
        utils::kitDrop(num);
        if(num > 0)
          right(90, 100, false);
        cur_cmd.remove(0);

        if(num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c == 'r') {
      if(cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        if(num > 0)
          right(90, 100, false);
        utils::kitDrop(num);
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
    if (c == 'g' || c == 'f' || c == 't' /* || c == 'd' */ || c == 'r') {
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
    // else if(c >= '0' && c <= '9')
    // {
    //   if(cur_cmd.length() > 0)
    //   {
    //     if(cur_cmd[0] == 'd')
    //     {
    //       utils::kitDrop(c - '0');
    //     }
    //     else
    //     {
    //       Serial.println("invalid command");
    //     }
    //     cur_cmd.remove(0);
    //   }
    // }
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
    else if (c == '\n' || c == '\0') {
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
          bool* arr = get_tof_vals(wall_tresh);

          //oled.println("test2");
          
          // Serial.print("Tof Vals: ");
          // Serial.println(vals);

          // //n e s w
          bool walls[4] = {arr[4], arr[0] || arr[1], arr[5], arr[2] || arr[3]};
          // not wrapped around and stuff 
          //oled_display_walls(walls);
          //  this is wrapped
          pi_send_data(walls);

          //checkpoint detection
          pi_send_tag("CP");
          PI_SERIAL.println(float(returnColor() == 2));

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
#if 1
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
      utils::stopMotors();
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

  if(tofCalibrated(5) <= wall_tresh - 50)
  {
    while(tofCalibrated(5) >= 70)
    {
      utils::forward(-speed);
    }
    utils::stopMotors();
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

  if(tofCalibrated(5) <= wall_tresh - 50)
  {
    while(tofCalibrated(5) >= 70)
    {
      utils::forward(-speed);
    }
    utils::stopMotors();
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
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

  double tstart = millis();

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
    // while(PI_SERIAL.available())
    // {
    //   utils::stopMotors();
    //   pi_read_vision();
    //   oled.println("detected");
    // }

    if (millis() - tstart < 3000) {
      utils::forward((PID * -speed), (PID * speed));
    } else {
      utils::forward((PID * -speed) - TURN_BOOST, (PID * speed) + TURN_BOOST);
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
    //   utils::stopMotors();
    //   pi_read_vision();
    //   oled.println("detected");
    // }

    if (millis() - tstart < 3000) {
      utils::forward((PID * speed), (PID * -speed));
    } else {
      utils::forward((PID * speed) + TURN_BOOST, (PID * -speed) - TURN_BOOST);
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

void alignAngle(int, bool, int x = 5);

void driveCM(float cm, int speed = 200, int tolerance = 10) {
  //utils::kitDrop(1);

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();  
    //alignAngle(90, false);
  }
  
  pi_send_data(true, true);
#if 1
  const float mult_factor = 1.0;
  unsigned int left = (tofCalibrated(0) + tofCalibrated(1))/2;
  unsigned int right = (tofCalibrated(2) + tofCalibrated(3))/2;

  double horizontalError = abs((int)left - (int)right) / 2;
  double angle = abs(atan((cm * 10.0) / horizontalError) * (180.0/PI));
  //angle = max(angle, 90 - 30);
  oled.println(angle * mult_factor);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (horizontalError >= tolerance && abs(orientationData.orientation.z) < 12 && left <= wall_tresh && right <= wall_tresh) {

    if (left > right) {

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
          utils::forward(speed * 0.7);

        }
        
        utils::stopMotors();

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
      else
      {
        Serial.println("achievement unlocked! How did we get here?");
        oled.clearDisplay();
        oled.println("achievement unlocked!");
        oled.println("How did we get here?");
        while(tofCalibrated(4) >= 70)
        {
          utils::forward(speed * 0.7);

        }
        
        utils::stopMotors();
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
  //       utils::forward(speed * 0.7);
  //     }

  //     utils::stopMotors();

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
  //       utils::forward(speed * 0.7);
  //     }

  //     utils::stopMotors();

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

  if(tofCalibrated(4) <= wall_tresh - 50)
  {
    while(tofCalibrated(4) >= 70)
    {
      utils::forward(speed * 0.7);

    }
    utils::stopMotors();
  }

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    //alignAngle(90, false);
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
  utils::resetTicks();
  double p, d, i = 0;
  double p_turn, d_turn, last_difference = 0;
  double PID;
  double last_dist = abs(motorR.getTicks() / abs(encoders));
  double startX = xPos;
  double orientation;
  double orig_encoders = encoders;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double start_pitch = orientationData.orientation.z;
  bool ramp_detect = true;
  bool down_ramp_detect = false;
  double start_ramp = INT_MAX;
  double tstart = millis();
  // encoders = orig_encoders / cos(-orientationData.orientation.z * (2 * PI / 360));


  while (abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders) && (tofCalibrated(4) >= 75)) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // encoders = orig_encoders / cos(abs(orientationData.orientation.z * (2 * PI / 360)));


    if (orientationData.orientation.z > 12) {
      start_ramp = millis();
      oled.println("ramp detected!");
    }

    if (millis() - start_ramp > 75 && ramp_detect) {
      encoders = motorR.getTicks() + (encoders - motorR.getTicks()) / cos(abs(orientationData.orientation.z * (PI / 180))) + 5 * CM_TO_ENCODERS;
      ramp_detect = false;
    }

    if (orientationData.orientation.z < -12 && abs(start_pitch) < 3) {
      speed *= 0.7;
      down_ramp_detect = true;
    }

    if (orientationData.orientation.z >= -12 && down_ramp_detect) {
      speed *= (1.0/0.7);
    }

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
        utils::forward(-speed);
      }
      utils::stopMotors();
      pi_send_data(false, false);
      utils::resetBoost();
      return;
    }

    
    //we are close enough to the target at this point, so quit the loop
    if ( /* abs(p_turn * DRIVE_STRAIGHT_KP) <= 0.01 && */ PID <= 0.01)
      break;
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    // Serial.println(speed * (double)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    if (abs(orientationData.orientation.z) < 3.5) {
      while(PI_SERIAL.available()) {
        int right_ticks = motorR.getTicks();
        int left_ticks = motorL.getTicks();
        utils::stopMotors();
        pi_read_vision();
        oled.println("detected");
        motorR.getTicks() = right_ticks;
        motorL.getTicks() = left_ticks;
      }
    }



    utils::forward(/* ramp_detect && millis() - tstart  < 5000 ? 255 : PID /* - p_turn * DRIVE_STRAIGHT_KP */ PID + DRIVE_BOOST);
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
  utils::resetBoost();
#else
  pi_send_data(true, true);
  delay(100);
  pi_send_data(false, true);
#endif
  
}

int closestToDirection(int num) {
  for (int i = 0; i <= 360; i+=90) {
    if (abs(num - i) <= 45) {
      return i;
    }
  }
}

void alignAngle(int speed, bool reset, int tolerance = 5) {
  int tofR1, tofR2; 
  int tofR3, tofR4;
  bool tofAlign = false;
  int lnum = 0, rnum = 1;

  tofR1 = tofCalibrated(0); 
  tofR2 = tofCalibrated(1); 
  tofR3 = tofCalibrated(2); 
  tofR4 = tofCalibrated(3); 

         
  if (tofR1 >= 160 || tofR2 >= 160) {
    lnum = 3;
    rnum = 2;
  }

  if (tofR3 >= 160 || tofR4 >= 160 && tofR1 >= 160 || tofR2 >= 160) {
    // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // int new_angle = closestToDirection((int) orientationData.orientation.x);

    // if ((int) orientationData.orientation.x  - new_angle < 0) {
    //   raw_right(abs(orientationData.orientation.x - new_angle), speed);
    // } else {
    //   raw_left(abs(orientationData.orientation.x - new_angle), speed);
    // }
    
    return;
  } else {
    // tofAlign = true;
  }

  float len = abs((int)tofCalibrated(lnum) - (int)tofCalibrated(rnum));

  // oled.println((int) tofCalibrated(0) - (int)tofCalibrated(1));
  // delay(2000);

  if(len <= tolerance)
    return;

  const int width = TOF_DISTANCE;
  const int angle = atan(width/len) * (180/PI); 

  if ( (int) tofCalibrated(lnum) - (int)tofCalibrated(rnum) < 0) {

    while(len >= tolerance) {
      utils::forward(-speed, speed);
      len = abs((int)tofCalibrated(lnum) - (int)tofCalibrated(rnum));
    }
  } else {
    
    while(len >= tolerance) {
      utils::forward(speed, -speed);
      len = abs((int)tofCalibrated(lnum) - (int)tofCalibrated(rnum));
    }
  }
  // if (reset) {
  //   bno.begin(OPERATION_MODE_IMUPLUS);
  // }

  if (tofAlign) {
    // bno.begin(OPERATION_MODE_IMUPLUS);
    // delay(500);
  }

  utils::stopMotors();

  //Serial.println(len);  
  //Serial.println(atan(width/len) * (180/3.1415)  );
  //Serial.println(angle);
  // Serial.print("Length: ");
  // Serial.print(len);
  // if (angle > 0) {
  //   // Serial.print(" Turn Right\t");
  //   // Serial.println(angle);
  //   raw_right(180 - angle, speed);
  // } else {
  //   // Serial.print(" Turn Left\t");
  //   // Serial.println(angle);
  //   raw_left(180 - angle, speed);
  // }
   
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
        cal = (1.09 * dist);
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
        cal = dist - 40;
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
        //TODO: calibrate (low priority)
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
  bool walls[4] = {arr[4], arr[0] || arr[1], arr[5], arr[2] || arr[3]};
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


  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // Serial.println(orientationData.orientation.z);

  #ifndef NO_PI

  // driveCM(27, 110);
  // delay(1000);
  // alignAngle(110, false);
  // utils::myservo.write(180);
  // delay(100);
  //turn('e');
  //delay(100);
  //Serial.println(returnColor());
  // utils::kitDrop(1);
  // delay(1000);

  for (int i = 0; i <= TOF_NUMBER; i++) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(tofCalibrated(i));
  }
  Serial.println();

  // static int dir = 0;
  // const char char_map[] = {'w', 'e', 'n', 's', 'n', 'w', 'e', 's', 'w', 'e'};

  // dir++;
  // dir %= ARRAY_SIZE(char_map);

  // turn(char_map[dir]);
  // bool* arr =  get_tof_vals(wall_tresh);
  // bool walls[4] = {arr[4], arr[0] || arr[1], arr[5], arr[2] || arr[3]};


  // pi_send_data(walls);
  // //String data = "    ";


  // oled.println("n, e, s, w");
  // oled.print(walls[utils::math::wrapAround((int)n - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.print(walls[utils::math::wrapAround((int)e - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.print(walls[utils::math::wrapAround((int)s - (int)cur_direction, 4)]);
  // oled.print(",");
  // oled.println(walls[utils::math::wrapAround((int)w - (int)cur_direction, 4)]);
  // oled.println(dir_to_char((int) cur_direction));
  // for(int i = 0; i < 4; i++)
  // {
  //   {
  //     if(walls[utils::math::wrapAround((int)i - (int)cur_direction, 4)])
  //     {
  //       data[i] = char_map[utils::mpi_rath::wrapAround((int)i - (int)cur_direction, 4)];
  //     }
  //   }
  // }

  //oled.println(data.c_str());

  delay(1000);


  

  //returnColor();


  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // Serial.print("return color: ");
  // Serial.println(returnColor());

  // utils::kitDrop(1);
  // delay(200);
  
  // Serial.println(returnColor());
  // oled.print(returnColor());
  // delay(100);
  // oled.setCursor(0,0);
  // Serial.print("Front:");
  // Serial.print(tofCalibrated(4));
  // Serial.print(",");
  // Serial.print("Right:");
  // Serial.print((tofCalibrated(0) + tofCalibrated(1)) / 2);
  // Serial.print(",");
  // Serial.print("Back:");
  // Serial.print(tofCalibrated(5));
  // Serial.print(",");
  // Serial.print("Left:");
  // Serial.println((tofCalibrated(2) + tofCalibrated(3)) / 2);
  // delay(1000);

  // utils::kitDrop(1);
  // delay(100);

  #else
  bool* arr = get_tof_vals(wall_tresh);

  // //n e s w
  bool walls[4] = {arr[4], arr[0] || arr[1], arr[5], arr[2] || arr[3]};
  // not wrapped around and stuff 
  oled_display_walls(walls);

  if(!walls[0])
  {
    driveCM(27, 110);
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



  delay(1000);

  #endif

  oled.clearDisplay();
  oled.setCursor(0,0);

  //Serial.println(returnColor());
}

#endif
