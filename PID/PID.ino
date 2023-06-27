//#define FAKE_ROBOT
//#define FAKE_SERIAL
#define DEBUG_DISPLAY
// #define MOTORSOFF
// #define TEST
// #define ALIGN_ANGLE
// #define NO_PI //basic auto when no raspberry pi (brain stem mode)
// #define NO_LIMIT
// #define NO_PID
// #define TCS
#define AMS

//define: debug display, motorsoff, test, comment out all others if you want to calibrate tofs

#include "Motors.h"
#include "utils.h"
#include "common.h"

using namespace utils;

bool restart = false;

void setup() {
  if (restart) {
    oled_clear();
    oled_println("Reinit...");
    delay(200);
    //reinit all variables here:
    resetBoost();
    stopMotors();
    resetTicks();
    resetServo();
    cur_direction = n;
    global_angle = 0;
    black_tile_detected = false;
  }

  restart = false;
  PI_SERIAL.begin(115200);
  Serial.begin(9600);
  setMotors(&motorR, &motorL);
  Wire.begin();
  Serial.println("starting the code!");
  //Wire.begin();
  //Wire.setClockStretchLimit(200000L);
#ifdef DEBUG_DISPLAY
  oled.begin();
  oled.setFlipMode(0);
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.setCursor(0, 0);
  oled_println("Starting...");
#endif

  bno.begin(OPERATION_MODE_IMUPLUS);
  global_angle = 0;
  oled_println("BNO init done!");

#ifndef NO_LIMIT
  pinMode(FRONT_RIGHT, INPUT_PULLUP);
  pinMode(FRONT_LEFT, INPUT_PULLUP);
  pinMode(BACK_RIGHT, INPUT_PULLUP);
  pinMode(BACK_LEFT, INPUT_PULLUP);

  if (digitalRead(FRONT_LEFT)) {
    Serial.println("front left limit disconnected");
    oled_println("front left disconnect");
  }

  if (digitalRead(FRONT_RIGHT)) {
    Serial.println("front right limit disconnected");
    oled_println("front right disconnect");
  }

  if (digitalRead(BACK_LEFT)) {
    Serial.println("back left limit disconnected");
    oled_println("back left disconnect");
  }

  if (digitalRead(BACK_RIGHT)) {
    Serial.println("back right limit disconnected");
    oled_println("back right disconnect");
  }

  if (!(digitalRead(FRONT_LEFT) || digitalRead(FRONT_RIGHT) || digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT))) {
    Serial.println("limit init successful");
    oled_println("limit init!");
  }
#endif

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {
    tcaselect(i);
    if (!tof.init()) {
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
  oled_println("TOF init done!");
  myservo.attach(servopin);
  myservo2.attach(servopin2);
  resetServo();
  myservo.write(175);
  delay(50);
  myservo.write(170);
  delay(50);
  myservo.write(175);
  delay(50);
  myservo.detach();
  myservo2.detach();
  oled_println("Servo reset");

#ifdef AMS
  while (!ams.begin()) {

    // Serial.println("could not connect to sensor! Please check your wiring.");
    oled_println("AMS init fail!!!!!");
    delay(100);
  }
  ams.drvOn();
  ams.setIntegrationTime(1);
#endif
#ifdef TCS
  tcaselect(6);
  if (!tcs.begin()) {
    Serial.println("color sensor init fail!");
    oled_println("TCS init fail!");
  }
#endif

  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  analogWrite(2, 50);
  Serial.println("TOF INIT SUCCEED!");
  oled_println("Startup Done!");
  delay(1000);
  oled_clear();

  analogWrite(2, 0);
  // delay(500);
}

void readColors() {
  //ams.drvOn();
  ams.startMeasurement();  //begin a measurement

  //wait till data is available
  bool rdy = false;
  while (!rdy) {
    // delay(1);
    rdy = ams.dataReady();
  }
  //ams.drvOff(); //uncomment this if you want to use the driver LED for readings

  //read the values!
  ams.readRawValues(amsValues);
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
  // oled_print(walls[math::wrapAround((int)n - (int)cur_direction, 4)]);
  // oled_print(",");
  // oled_print(walls[math::wrapAround((int)e - (int)cur_direction, 4)]);
  // oled_print(",");
  // oled_print(walls[math::wrapAround((int)s - (int)cur_direction, 4)]);
  // oled_print(",");
  // oled_println(walls[math::wrapAround((int)w - (int)cur_direction, 4)]);
}

bool* get_tof_vals(double threshold) {

  static bool arr[TOF_NUMBER + 1];
  uint32_t reading = 0;

  memset(arr, 0, ARRAY_SIZE(arr));

  //Serial.println("start");

  for (int i = TOF_START; i <= TOF_NUMBER; i++) {
    reading = tofCalibrated(i);

    if (reading < threshold - ((i >= 4) ? 50 : 0)) {
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

void driveCM(float, int, int y = 10);
void left(int, int, bool);
void right(int, int, bool);

void pi_read_vision() {
  String data = "";
  char ch = 0;
  int num = 0;

  while (PI_SERIAL.available() && ch != '\n') {
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
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        kitDrop(num, 'l');

        cur_cmd.remove(0);

        if (num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c == 'r') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        kitDrop(num, 'r');

        cur_cmd.remove(0);

        if (num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c >= '0' && c <= '9') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        num = c - '0';
      }
    } else if (c == 'q') {
      Serial.println("Restarting...");
      delay(200);
      restart = true;
      return;
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
  while (PI_SERIAL.available() && ch != '\n') {
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
    if (c == 'g' || c == 'f' || c == 't' || c == 'd' || c == 'q') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'g' || cur_cmd[0] == 'f') {
          Serial.println("FORWARD");
          oled_println("forward");
          driveCM(tile_dist, 110);
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
      }
      cur_cmd.remove(0);
      cur_cmd += c;
    }
    if (c >= '0' && c <= '9') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        num = c - '0';
      }
    }
    if (c == 'l') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        kitDrop(num, 'l');

        cur_cmd.remove(0);

        if (num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c == 'r') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        kitDrop(num, 'r');
        cur_cmd.remove(0);

        if (num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      } else {
        bool* arr = get_tof_vals(wall_tresh);

        //oled_println("test2");

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

        if (returnColor() == 2) {
          pi_send_tag("CP");
          PI_SERIAL.println("1.0");
          oled_clear();
          stopMotors();
          delay(4000);
        } else {
          pi_send_tag("CP");
          PI_SERIAL.println("0.0");
        }

        //Serial.println("sending wall data");
      }
    }
    if (c == 'e' || c == 'w' || c == 's' || c == 'n') {
      if (cur_cmd.length() > 0) {
        if (cur_cmd[0] == 'f' || cur_cmd[0] == 'g') {
          Serial.print("turn to ");
          Serial.println(c);
          Serial.println("FORWARD");
          oled_print("turn to ");
          oled_println(c);
          oled_println("forward");
          turn(c);
          //pi_send_data({ false, false, false, false });
          driveCM(tile_dist, 110);
        } else if (cur_cmd[0] == 't') {
          Serial.print("turn to ");
          Serial.println(c);
          oled_print("turn to ");
          oled_println(c);
          turn(c);
          oled_println("done");
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
          oled_println("forward");
          driveCM(tile_dist, 110);
        } else {
          Serial.println("ERR: Invalid Parameter");
        }
        if (cur_cmd[0] == 'q') {
          Serial.println("restarting");
          delay(200);
          restart = true;
          return;
        }
      }
      cur_cmd = "";
    }
  }
}

// FIXME: Blue being detected as black
int returnColor(bool only_black = false) {
  uint16_t r, g, b, c = 0;
  int silver_detect = 0;
  int black_detect = 0;
  int blue_detect = 0;

  const int persistance_count = 0;
#ifdef TEST

#ifdef TCS
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print("clear:");
  Serial.print(c);
  Serial.print(",");
  Serial.print("r/g:");
  Serial.print((float)r / g * 100.0);
  Serial.print(",");
  Serial.print("b/r:");
  Serial.print((double)b / r * 100.0);
  Serial.print(',');
  Serial.print("sub:");
  Serial.print((double)r / g * 100.0 - (double)b / r * 100.0);
#endif

#endif

#ifdef TCS
  UPDATE_BNO();
  for (int i = 0; i <= persistance_count; i++) {

    if (c >= 40 && abs((r / (double)g) * 100.0 - ((double)b / r) * 100.0) <= 10 && abs(BNO_Z) < 12) {
      //   silver_persistance++;
      // Serial.println("silver detected");
      // oled_println("silver");
      //pi_send_tag("color");
      //PI_SERIAL.println("silver");
      silver_detect++;  //change later
    }
    if (c < 20 && (double)b / r < 1.0 && abs(BNO_Z) < 12) {
      // Serial.println("black detected");
      //oled_println("black");
      //pi_send_tag("color");
      //PI_SERIAL.println("black");
      black_detect++;
    }
    if (c <= 50 && (double)b / r >= 1.5 && abs(BNO_Z) < 12) {
      blue_detect++;
    }
  }

  if (blue_detect >= persistance_count && !only_black) {
    Serial.println(" blue detected");
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println(" blue detected");
    stopMotors();
    delay(5000);
    return 3;
  } else if (black_detect >= persistance_count) {
    Serial.println(" black detected");
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println(" black detected");
    return 1;
  } else if (silver_detect >= persistance_count && !only_black) {
    Serial.println(" silver detected");
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println(" silver detected");
    return 2;
  } else {
    Serial.println(" white detected");
    return 0;
  }
#endif
#ifdef AMS
  UPDATE_BNO();

  readColors();
#ifdef TEST
  Serial.print("Violet:");
  Serial.print(amsValues[AS726x_VIOLET]);
  Serial.print(",Blue:");
  Serial.print(amsValues[AS726x_BLUE]);
  Serial.print(",Green:");
  Serial.print(amsValues[AS726x_GREEN]);
  Serial.print(",Yellow:");
  Serial.print(amsValues[AS726x_YELLOW]);
  Serial.print(",Orange:");
  Serial.print(amsValues[AS726x_ORANGE]);
  Serial.print(",Red:");
  Serial.print(amsValues[AS726x_RED]);
  Serial.println();
#endif

  bool violet_greatest = true;
  int dark_count = 0;
  int bright_count = 0;

  for (int i = 0; i <= AS726x_RED; i++) {
    if (amsValues[AS726x_VIOLET] < amsValues[i]) {
      violet_greatest = false;
    }

    if (amsValues[i] <= 20) {
      dark_count++;
    }

    if (amsValues[i] >= 150) {
      bright_count++;
    }
  }

  if (dark_count >= 5 && abs(BNO_Z) < 12) {
    // stopMotors();
    oled_clear();
    oled_println(" black detected");
    // delay(5000);
    Serial.println(" black detected");
    return 1;
  } else if (bright_count >= 3 && abs(BNO_Z) < 12 && !only_black) {
    // stopMotors();
    oled_clear();
    oled_println(" silver detected");
    // delay(5000);
    Serial.println(" silver detected");
    return 2;
  } else if (violet_greatest && !only_black && amsValues[AS726x_VIOLET] > 40 && amsValues[AS726x_RED] <= 18 && amsValues[AS726x_BLUE] >= 18 && abs(BNO_Z) < 12) {
    stopMotors();
    Serial.println(" blue detected");
    oled_clear();
    oled_println(" blue detected");
    delay(5000);
    return 3;
  }

#endif

  // if(b > (r * 2.5) && !only_black){
  //   // Serial.println("blue detected");
  //   // oled_println("blue");
  //   //pi_send_tag("color");
  //  //PI_SERIAL.println("blue");
  //   return 0; //change later
  // }
  return 0;
}


void backup_align(int speed, int time) {

  while (tofCalibrated(5) >= 60 || digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT)) {
    forward(-speed);
  }

  stopMotors();
  delay(100);

  int32_t tstart = millis();

#ifndef NO_LIMIT

  while ((int32_t)millis() - tstart < time) {
    if (!digitalRead(BACK_LEFT)) {
      motorL.run(-SPEED * 0.7);
    } else {
      motorL.stop();
    }

    if (!digitalRead(BACK_RIGHT)) {
      motorR.run(-SPEED * 0.7);
    } else {
      motorR.stop();
    }
  }
#else
  while ((int32_t)millis() - tstart < time) {
    forward(-SPEED * 0.7);
  }


#endif

  stopMotors();
  bno.begin(OPERATION_MODE_IMUPLUS);
  global_angle = 0;
  delay(50);
}

void right(int relative_angle, int speed, bool turn_status = true) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // double orientation = orientationData.orientation.x;

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(1.0);
  }


  int offset = (int)(orientationData.orientation.x) % relative_angle;

  if (offset < relative_angle / 2) {
    relative_angle -= offset;
  }
  // Serial.print("With offset: ");
  // if (offset < relative_angle / 2) {

  //   Serial.println(relative_angle - offset);
  // } else {
  //   Serial.println(relative_angle + offset);
  // }

  raw_right(relative_angle, speed, false);



  if (tofCalibrated(5) <= wall_tresh - 30) {
    backup_align(SPEED, 600);
    // while(tofCalibrated(5) >= 70)
    // {
    //   forward(-speed);
    // }
    while (tofCalibrated(5) <= 40) {
      forward(speed);
    }
    stopMotors();
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
}

void left(int relative_angle, int speed, bool turn_status = true) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  int offset = relative_angle - (int)(orientationData.orientation.x) % relative_angle;

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

  raw_left(relative_angle, speed, false);

  if (tofCalibrated(5) <= wall_tresh - 30) {
    backup_align(SPEED, 600);
    // while(tofCalibrated(5) >= 70)
    // {
    //   forward(-speed);
    // }
    while (tofCalibrated(5) <= 40) {
      forward(speed);
    }
    stopMotors();
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
}

void raw_right(double relative_angle, int speed, bool alignment) {

  if (abs(relative_angle) < 1) {
    return;
  }

#ifndef MOTORSOFF
  if (alignment) {
    motorL.addBoost(ALIGN_TURN_BOOST);
    motorR.addBoost(ALIGN_TURN_BOOST);
    speed = ALIGN_SPEED;
  } else {
    motorL.addBoost(TURN_BOOST);
    motorR.addBoost(TURN_BOOST);
  }

  double p, i = 0, d;
  double PID;
  bool cross_over = false;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  if (orientationData.orientation.x + relative_angle >= 360) {
    cross_over = true;
  }

  const double initial_angle = orientationData.orientation.x;

  double orientation = cross_over ? orientationData.orientation.x - relative_angle : orientationData.orientation.x;
  // double angle = cross_over ? global_angle : global_angle + relative_angle;
  //FIXME: Robot keeps dying when I use global angle
  double angle = orientation + relative_angle;
  double last_error = abs((orientationData.orientation.x - angle) / angle);

  double tstart = millis();

#ifndef NO_PID
  while (abs(orientation - angle) > 1) {
#else
  while (orientation < angle) {
#endif

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
    //   oled_println("detected");
    // }

    if (digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 4 * CM_TO_ENCODERS) {
        forward(SPEED * 0.75);
      }
    } else if (digitalRead(FRONT_LEFT) || digitalRead(FRONT_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 4 * CM_TO_ENCODERS) {
        forward(-SPEED * 0.75);
      }
    }

#ifndef NO_PID

    if (millis() - tstart < 3000) {
      forward((PID * -speed), (PID * speed));
    } else {
      if (alignment) {
        addBoost(ALIGN_TURN_BOOST + 70);
      } else {
        addBoost(TURN_BOOST + 70);
      }
      forward((PID * -speed), (PID * speed));
    }
#else
    addBoost(0);
    if (millis() - tstart < 3000) {
      forward(-180, 180);
    } else {
      forward(-110, 110);
    }
#endif

    if (PID <= 0.01)
      break;

    // Serial.print("PID: ");
    // Serial.println(PID);

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orientation = cross_over ? orientationData.orientation.x - relative_angle : orientationData.orientation.x;

    if (orientationData.orientation.x < initial_angle) {
      orientation += 360;
    }

    // if (orientationData.orientation.x < 1.0) {
    //   orientation += 360;
    // }
  }
  resetBoost();
  stopMotors();
#endif
}

void raw_left(double relative_angle, int speed, bool alignment) {
#ifndef MOTORSOFF

  if (abs(relative_angle) < 1) {
    return;
  }

  if (!alignment) {
    motorL.addBoost(TURN_BOOST);
    motorR.addBoost(TURN_BOOST);
  } else {
    motorL.addBoost(ALIGN_TURN_BOOST);
    motorR.addBoost(ALIGN_TURN_BOOST);
    speed = ALIGN_SPEED;
  }


  double p, i = 0, d;
  double PID;
  bool cross_over = false;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  if (orientationData.orientation.x - relative_angle < 0) {
    cross_over = true;
  }

  const double initial_angle = orientationData.orientation.x;
  double orientation = cross_over ? orientationData.orientation.x + relative_angle : orientationData.orientation.x;
  double angle = orientation - relative_angle;
  double last_error = abs((orientationData.orientation.x - angle) / angle);

  double tstart = millis();

#ifndef NO_PID
  while (abs(orientation - angle) > 1) {
#else
  while (orientation > angle) {
#endif
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
    //   oled_println("detected");
    // }

    if (digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 4 * CM_TO_ENCODERS) {
        forward(SPEED * 0.75);
      }
    } else if (digitalRead(FRONT_LEFT) || digitalRead(FRONT_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 4 * CM_TO_ENCODERS) {
        forward(-SPEED * 0.75);
      }
    }

#ifndef NO_PID
    if (millis() - tstart < 3000) {
      forward((PID * speed), (PID * -speed));
    } else {
      if (alignment) {
        addBoost(ALIGN_TURN_BOOST + 70);
      } else {
        addBoost(TURN_BOOST + 70);
      }
      forward((PID * speed), (PID * -speed));
    }
#else
    addBoost(0);
    if (millis() - tstart < 3000) {
      forward(180, -180);
    } else {
      forward(110, -110);
    }
#endif

    if (PID <= 0.01)
      break;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    orientation = cross_over ? orientationData.orientation.x + relative_angle : orientationData.orientation.x;

    if (orientationData.orientation.x > initial_angle) {
      orientation -= 360;
    }

    // if (orientationData.orientation.x < 1.0) {
    //   orientation += 360;
    // }
  }
  resetBoost();
  stopMotors();
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

  switch ((int)cur_direction - (int)end_direction) {
    case -3:
    case 1:
      left(90, SPEED);
      global_angle = math::wrapAround(global_angle - 90, 360);
      break;
    case -1:
    case 3:
      right(90, SPEED);
      global_angle = math::wrapAround(global_angle + 90, 360);
      break;
    case 2:
    case -2:
      left(90, SPEED, false);
      global_angle = math::wrapAround(global_angle - 90, 360);
      left(90, SPEED);
      global_angle = math::wrapAround(global_angle - 90, 360);
      break;
    default: Serial.println("invalid");
    case 0:
      pi_send_tag("turn_status");
      PI_SERIAL.println(0.0);
      break;
  }

  cur_direction = end_direction;
}

void alignAngle(bool, int tolerance = 10, int start_yaw = INT32_MAX);

void driveCM(float cm, int speed = 200, int tolerance = 10) {
  //kitDrop(1);
  double start_yaw;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    alignAngle(true);
    UPDATE_BNO();
    start_yaw = BNO_X;
  }

  pi_send_data(true, true);
#if 1
  const float mult_factor = 1.0;
  uint right = (tofCalibrated(2) + tofCalibrated(3)) / 2;
  uint left = (tofCalibrated(0) + tofCalibrated(1)) / 2;
  const float half_chassis = 75;
  const double target_dist_from_wall = (300.0 - half_chassis * 2) / 2.0;

  double horizontalError = abs((int)left - (int)right) / 2;

  if (horizontalError == 0) {
    horizontalError = 0.0001;
  }

  double angle = abs(atan((cm * 10.0) / horizontalError) * (180.0 / PI));

  if (abs(angle) < 1) {
    return;
  }

  if (abs(angle) == 180.0) {
    angle -= 0.0001;
  }

  //angle = max(angle, 90 - 30);
  oled_println(angle * mult_factor);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  bool optimal_alignment = horizontalError >= tolerance && abs(orientationData.orientation.z) < 12;
  if (optimal_alignment && left <= wall_tresh && right <= wall_tresh && abs(left - right) > 25) {

    if (left < right) {

      raw_right(90 - min(90, angle * mult_factor), SPEED, true);

      if (tofCalibrated(4) > wall_tresh - 50)
        drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI / 180))), speed);
      else {
        Serial.println("achievement unlocked! How did we get here?");
        oled_clear();
        oled_println("achievement unlocked!");
        oled_println("How did we get here?");
        while (tofCalibrated(4) >= 90) {
          forward(speed * 0.7);
        }
        stopMotors();

        raw_left(90 - min(90, angle * mult_factor), SPEED, true);
        pi_send_data(false, true, true);
        return;
      }

      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      if (abs(orientationData.orientation.z) < 12)
        raw_left(90 - min(90, angle * mult_factor), SPEED, true);
      // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      // raw_left(orientationData.orientation.x, SPEED);

    } else {
      raw_left(90 - min(90, angle * mult_factor), SPEED, true);

      if (tofCalibrated(4) > wall_tresh - 50)
        drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI / 180))), speed);
      else {
        Serial.println("achievement unlocked! How did we get here?");
        oled_clear();
        oled_println("achievement unlocked!");
        oled_println("How did we get here?");
        while (tofCalibrated(4) >= 90) {
          forward(speed * 0.7);
        }

        stopMotors();
        raw_right(90 - min(90, angle * mult_factor), SPEED, true);

        pi_send_data(false, true, true);
        return;
      }

      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      if (abs(orientationData.orientation.z) < 12)
        raw_right(90 - min(90, angle * mult_factor), SPEED, true);
      // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      // raw_right(360-orientationData.orientation.x, SPEED);
    }
  } else if (optimal_alignment && left <= wall_tresh && right >= wall_tresh && abs(left - target_dist_from_wall) > 30.0) {
    oled_println("single left wall");

    double angle = 90.0;

    if (left != target_dist_from_wall)
      angle = atan((cm * 10.0) / (abs(left - target_dist_from_wall))) * (180 / PI);

    if (left - target_dist_from_wall > 0.0) {
      raw_left(90.0 - angle, SPEED, true);
    } else if (left - target_dist_from_wall < 0.0) {
      raw_right(90.0 - angle, SPEED, true);
    }
    oled_print("Angle: ");
    oled_println(90.0 - angle);
    // oled_print(atan((cm * 10)/(150 - (half_chassis + left))));

    if (tofCalibrated(4) > wall_tresh - 50)
      drive(cm / sin(angle * (PI / 180)) * CM_TO_ENCODERS, speed);
    else {
      Serial.println("achievement unlocked! How did we get here?");
      oled_clear();
      oled_println("achievement unlocked!");
      oled_println("How did we get here?");
      while (tofCalibrated(4) >= 70) {
        forward(SPEED * 0.7);
      }
      stopMotors();

      if (left - target_dist_from_wall > 0.0) {
        raw_right(90.0 - angle, SPEED, true);
      } else if (left - target_dist_from_wall < 0.0) {
        raw_left(90.0 - angle, SPEED, true);
      }
      pi_send_data(false, true, true);
      return;
    }
    if (left - target_dist_from_wall > 0.0) {
      raw_right(90.0 - angle, SPEED, true);
    } else if (left - target_dist_from_wall < 0.0) {
      raw_left(90.0 - angle, SPEED, true);
    }

  } else if (optimal_alignment && left >= wall_tresh && right <= wall_tresh && abs(right - target_dist_from_wall) > 30.0) {
    oled_println("single right wall");
    oled_print("Angle: ");

    double angle = 90.0;

    if (right != target_dist_from_wall)
      angle = atan((cm * 10.0) / (abs(right - target_dist_from_wall))) * (180 / PI);

    oled_println(90.0 - angle);

    if (right - target_dist_from_wall > 0.0) {
      raw_right(90.0 - angle, SPEED, true);
    } else if (right - target_dist_from_wall < 0.0) {
      raw_left(90.0 - angle, SPEED, true);
    }

    if (tofCalibrated(4) > wall_tresh - 50)
      drive(cm / sin(angle * (PI / 180)) * CM_TO_ENCODERS, speed);
    else {
      Serial.println("achievement unlocked! How did we get here?");
      oled_clear();
      oled_println("achievement unlocked!");
      oled_println("How did we get here?");
      while (tofCalibrated(4) >= 70) {
        forward(SPEED * 0.7);
      }
      stopMotors();

      if (right - target_dist_from_wall > 0.0) {
        raw_left(90.0 - angle, SPEED, true);
      } else if (right - target_dist_from_wall < 0.0) {
        raw_right(90.0 - angle, SPEED, true);
      }
      pi_send_data(false, true, true);
      return;
    }

    if (right - target_dist_from_wall > 0.0) {
      raw_left(90.0 - angle, SPEED, true);
    } else if (right - target_dist_from_wall < 0.0) {
      raw_right(90.0 - angle, SPEED, true);
    }
  }
  else
#endif
  {
    if (tofCalibrated(4) > wall_tresh - 50)
      drive((cm * CM_TO_ENCODERS), speed);
    else {
      Serial.println("achievement unlocked! How did we get here?");
      oled_clear();
      oled_println("achievement unlocked!");
      oled_println("How did we get here?");
      while (tofCalibrated(4) >= 70) {
        forward(SPEED * 0.7);
      }
      stopMotors();

      pi_send_data(false, true, true);
      return;
    }
  }

  if (tofCalibrated(4) <= wall_tresh) {
    while (tofCalibrated(4) >= 90) {
      forward(speed * 0.7);
    }
    stopMotors();
  }

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    alignAngle(true, 10, start_yaw);
  }

  //pause for blue if detected
  returnColor();

  //WARN: drive function can no longer be used on it's own (when communicating with stereo pi) !!!!!
  pi_send_data(false, !black_tile_detected);
  black_tile_detected = false;
}

bool handle_up_ramp(double start_pitch, int32_t end_encoders) {
  int32_t ticks = 7 * CM_TO_ENCODERS;
  int32_t back_up = 0;
  int32_t delta_x = 0;
  int32_t delta_theta = 0;
  int32_t delta_time = 10;
  const double BNO_KP = 3;

  UPDATE_BNO();
  //delay(20);
  double new_angle = closestToDirection(BNO_X);

  double distance = 0;
  double height = 0;
  auto old_ticks = motorR.getTicks();
  resetTicks();
  // stopMotors();
  // oled.clearDisplay();
  // oled.setCursor(0,0);
  // oled_println("start forward ramp");
  // delay(1000);


  while (abs(motorR.getTicks()) < abs(ticks)) {
    forward(100);
  }

  // stopMotors();
  // oled.clearDisplay();
  // oled.setCursor(0,0);
  // oled_println("end forward ramp");
  // delay(1000);
  // bool* arr = get_tof_vals(wall_tresh);

  // n e s w
  if ((tofCalibrated(1) >= wall_tresh || tofCalibrated(0) >= wall_tresh) && (tofCalibrated(2) >= wall_tresh || tofCalibrated(3) >= wall_tresh)) {
    // stopMotors();
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println("no walls either side");
    // delay(5000);
    motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;
    motorL.getTicks() = -old_ticks - ticks + 4 * CM_TO_ENCODERS;
    return false;
  }

  UPDATE_BNO();
  if (abs(BNO_Z - start_pitch) <= 3 || BNO_Z - start_pitch > 3) {
    // not a ramp
    // int32_t dist = abs(motorL.getTicks()) - end_encoders;
    // motorR.resetTicks();
    // while (abs(motorR.getTicks()) < dist) {
    //   forward(-100);
    // }
    // back_up = motorR.getTicks();

    motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;
    motorL.getTicks() = -old_ticks - ticks + 4 * CM_TO_ENCODERS;

    // stopMotors();
    // oled.clearDisplay();
    // oled.setCursor(0,0);
    // oled_println("not enough pitch");
    // delay(5000);
    return false;
  } else {
    double old_x = motorR.getTicks();
    const float wall_kp = 0.10f;

    while (abs(BNO_Z - start_pitch) >= 4 && tofCalibrated(4) >= 90) {
      UPDATE_BNO();
      double reading;

      if (BNO_X - new_angle > 180) {
        reading = BNO_X - 360;
      } else if (BNO_X - new_angle < -180) {
        reading = BNO_X + 360;
      } else {
        reading = BNO_X;
      }

      double bno_error = (reading - new_angle) * (BNO_KP + abs(reading - new_angle) / 270.0);

      // if (bno_error > 80) {
      //   bno_error = 80;
      // } else if (bno_error < -80) {
      //   bno_error = -80;
      // }

      bno_error = min(bno_error, 50);
      bno_error = max(bno_error, -50);

      int32_t right = (_tofCalibrated(0) + _tofCalibrated(1)) / 2;
      int32_t left = (_tofCalibrated(2) + _tofCalibrated(3)) / 2;
      float err = 0.0f;

      if (left <= wall_tresh && right <= wall_tresh)
        err = (right - left) * wall_kp;
      else if (left <= wall_tresh)
        err = (75 - left) * wall_kp;
      else if (right <= wall_tresh)
        err = (right - 75) * wall_kp;

      forward(120.0 + bno_error + err, 120.0 - bno_error - err);

      // calculate distance on a ramp
      double delta_x = abs(motorR.getTicks()) - abs(old_x);
      double delta_theta = abs(BNO_Z - start_pitch);
      height += delta_x * sin(delta_theta * (PI / 180.0));
      distance += delta_x * cos(delta_theta * (PI / 180.0));
      old_x = motorR.getTicks();
    }


    if ((distance / (30.0 * CM_TO_ENCODERS)) - 0.4 <= 0.65) {
      motorR.getTicks() = - old_ticks - ticks - 4 * CM_TO_ENCODERS;
      motorL.resetTicks();


      // while (abs(motorL.getTicks()) < ticks) {
      //   forward(SPEED * -0.75);
      // }
      oled_clear();
      oled_print("Fake Ramp: ");
      oled_println((distance / (30.0 * CM_TO_ENCODERS)) - 0.4);
      oled_print("Distance: ");
      oled_println(distance);
      oled_print("Height: ");
      oled_println(height);
      stopMotors();
      delay(5000);
      return true;
    }

    stopMotors();
    delay(100);

    UPDATE_BNO();

    double BNO_STATIC_KP = 30;
    while (abs(BNO_X - new_angle) > 1) {
      double reading;
      UPDATE_BNO();
      if (BNO_X - new_angle > 180.0) {
        reading = BNO_X - 360;
      } else if (BNO_X - new_angle < -180.0) {
        reading = BNO_X + 360;
      } else {
        reading = BNO_X;
      }

      double bno_error = (reading - new_angle) * (BNO_STATIC_KP + abs(reading - new_angle) / 270.0);
      forward(bno_error, -bno_error);
    }

    stopMotors();
    alignAngle(true);
    oled_clear();
    oled_print("Ramps: ");
    oled_println((distance / (30.0 * CM_TO_ENCODERS)) - 0.4);
    oled_print(height / (30.0 * CM_TO_ENCODERS));
    delay(5000);
    pi_send_tag("ramp");
    PI_SERIAL.print(1.0);
    PI_SERIAL.print(",");
    PI_SERIAL.print(round((distance / (30.0 * CM_TO_ENCODERS)) - 0.4));
    PI_SERIAL.print(",");
    height = round(height / (30.0 * CM_TO_ENCODERS));
    if (height == 0) {
      height = 1;
    }
    PI_SERIAL.println(height);
    alignAngle(true);
    return true;
  }
}

bool handle_down_ramp(double start_pitch, double end_encoders) {
  int32_t ticks = 7 * CM_TO_ENCODERS;
  int32_t back_up = 0;
  auto old_ticks = motorR.getTicks();
  int32_t delta_time = 10;
  double distance = 0;
  double height = 0;
  UPDATE_BNO();
  double new_angle = 0;

  motorR.resetTicks();
  while (abs(motorR.getTicks()) < abs(ticks)) {
    forward(90);
  }
  UPDATE_BNO();

  // bool* arr = get_tof_vals(wall_tresh);

  //   // n e s w
  // bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };


  if ((tofCalibrated(1) >= wall_tresh || tofCalibrated(0) >= wall_tresh) && (tofCalibrated(2) >= wall_tresh || tofCalibrated(3) >= wall_tresh)) {
    // stopMotors();
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println("no walls either side");
    // delay(5000);
    motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;
    motorL.getTicks() = -old_ticks - ticks + 4 * CM_TO_ENCODERS;
    return false;
  }

  if (abs(BNO_Z - start_pitch) <= 3 || BNO_Z - start_pitch < -3) {
    //not a ramp

    // stopMotors();
    // oled.clearDisplay();
    // oled.setCursor(0,0);
    // oled_println("not enough pitch");
    // delay(5000);

    motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;
    motorL.getTicks() = -old_ticks - ticks + 4 * CM_TO_ENCODERS;
    return false;
  } else {
    double old_x = motorR.getTicks();
    const float wall_kp = 0.15f;
    const double BNO_KP = 5;
    while (abs(BNO_Z - start_pitch) >= 4 && tofCalibrated(4) >= 90) {
      UPDATE_BNO();
      double reading = abs(BNO_X - new_angle > 180) ? BNO_X - 360 : BNO_X;
      double bno_error = (reading - new_angle) * BNO_KP;

      bno_error = min(bno_error, 30);
      bno_error = max(bno_error, -30);

      int32_t right = (_tofCalibrated(0) + _tofCalibrated(1)) / 2;
      int32_t left = (_tofCalibrated(2) + _tofCalibrated(3)) / 2;
      float err = 0.0f;

      if (left <= wall_tresh && right <= wall_tresh)
        err = (right - left) * wall_kp;
      //at the end of the ramp there will only be 1 wall
      else if (left <= wall_tresh)
        err = (75 - left) * wall_kp;
      else if (right <= wall_tresh)
        err = (right - 75) * wall_kp;

      forward(80.0 + err, 80.0 - err);

      // calculate distance on a ramp
      double delta_x = abs(motorR.getTicks()) - abs(old_x);
      double delta_theta = abs(BNO_Z - start_pitch);
      distance += delta_x * cos(delta_theta * (PI / 180.0));
      height += delta_x * sin(delta_theta * (PI / 180.0));
      old_x = motorR.getTicks();
    }

    if ((distance / (30.0 * CM_TO_ENCODERS)) - 0.2 <= 0.65) {
      motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;
      // motorL.resetTicks();

      // while (abs(motorL.getTicks()) < 6 * CM_TO_ENCODERS) {
      //   forward(SPEED * -0.75);
      // }

      stopMotors();
      oled_clear();
      oled_print("Fake Ramp: ");
      oled_println((distance / (30.0 * CM_TO_ENCODERS)) - 0.2);
      oled_print("Distance: ");
      oled_println(distance);
      oled_print("Height: ");
      oled_println(height);
      stopMotors();
      delay(5000);

      return true;
    }

    stopMotors();
    delay(100);

    UPDATE_BNO();

    double BNO_STATIC_KP = 30.0;
    while (abs(BNO_X - new_angle) > 1) {
      UPDATE_BNO();
      double reading;
      if (BNO_X - new_angle > 180) {
        reading = BNO_X - 360;
      } else if (BNO_X - new_angle < -180) {
        reading = BNO_X + 360;
      } else {
        reading = BNO_X;
      }

      double bno_error = (reading - new_angle) * (BNO_STATIC_KP + abs(reading - new_angle) / 270.0);
      forward(bno_error, -bno_error);
    }

    stopMotors();
    alignAngle(true);
    oled_clear();
    oled_print("Ramps: ");
    oled_println((distance / (30.0 * CM_TO_ENCODERS)) - 0.2);
    oled_print(height / (30.0 * CM_TO_ENCODERS));
    delay(5000);
    pi_send_tag("ramp");
    PI_SERIAL.print(10.0);
    PI_SERIAL.print(",");
    PI_SERIAL.print(round((distance / (30.0 * CM_TO_ENCODERS)) - 0.2));
    PI_SERIAL.print(",");
    height = round(height / (30.0 * CM_TO_ENCODERS));
    if (height == 0) {
      height = 1;
    }
    PI_SERIAL.println(height);
    alignAngle(true);
    return true;
  }
}

int left_obstacle() {
  oled_println("Left");

  stopMotors();
  delay(100);

  bool forward_obstacle = true;
  int forward_ticks = 15 * CM_TO_ENCODERS;

  resetTicks();
  forward_obstacle = false;

  while (abs(motorR.getTicks()) < forward_ticks) {
    forward(-SPEED * 0.7);

    if (digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT) || tofCalibrated(5) <= 40) {
      break;
    }
  }
  forward_ticks = abs(motorR.getTicks());

  stopMotors();
  delay(100);

  raw_right(10, SPEED * 0.7, true);

  stopMotors();
  delay(100);

  return abs(forward_ticks);
}

int right_obstacle() {
  oled_println("Right");

  stopMotors();
  delay(100);

  bool forward_obstacle = true;
  int forward_ticks = 15 * CM_TO_ENCODERS;

  resetTicks();
  forward_obstacle = false;

  while (abs(motorR.getTicks()) < forward_ticks) {
    forward(-SPEED * 0.7);

    if (digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT) || tofCalibrated(5) <= 40) {
      break;
    }
  }

  forward_ticks = abs(motorR.getTicks());

  stopMotors();
  delay(100);

  raw_left(10, SPEED * 0.7, true);

  stopMotors();
  delay(100);

  return abs(forward_ticks);
}

void drive(int32_t encoders, int speed) {
#ifndef MOTORSOFF
  addBoost(DRIVE_BOOST);
  UPDATE_BNO();
  resetTicks();
  int angle = 60, tofR1, tofR2;
  double p, d, i = 0;
  double PID;
  double start_pitch = BNO_Z;
  double start_yaw = BNO_X;
  bool ramp_detect = false;
  bool down_ramp_detect = false;
  uint32_t tstart = millis();
  int32_t ticks_before = 0;
  bool limit_detected = false;
  double BNO_KP = 7;
  double new_angle = start_yaw;
  // encoders = orig_encoders / cos(-orientationData.orientation.z * (2 * PI / 360));


  while ((abs(motorR.getTicks()) < abs(encoders) && abs(motorL.getTicks()) < abs(encoders) && (tofCalibrated(4) >= 90)) || ramp_detect || down_ramp_detect) {
    UPDATE_BNO();

    if ((BNO_Z - start_pitch < -5.5 /* || BNO_Z < -5.5 */) && !down_ramp_detect) {
      oled_println("down ramp detect!!");
      bool res = handle_up_ramp(start_pitch, encoders);
      ramp_detect = res;

      if (res)
        return;
    }

    if ((BNO_Z - start_pitch > 5.5 /* || BNO_Z > 5.5 */) && !ramp_detect) {
      // stopMotors();
      oled_println("down ramp detect!!");
      bool res = handle_down_ramp(start_pitch, encoders);
      down_ramp_detect = res;
      tstart = millis();

      if (res)
        return;
    }

#ifndef NO_LIMIT
    if (digitalRead(FRONT_RIGHT) == HIGH && abs(BNO_Z) < 4) {
      ticks_before = abs(motorR.getTicks());
      int dist = left_obstacle();
      motorR.setTicks(-(ticks_before - dist));
      // encoders *= 1.0 / cos(15.0 * (PI/180));
      UPDATE_BNO();
      new_angle = BNO_X;
      tstart = millis();
      limit_detected = true;
      Serial.println("obstacle is done");
    }

    if (digitalRead(FRONT_LEFT) == HIGH && abs(BNO_Z) < 4) {
      ticks_before = abs(motorR.getTicks());
      int dist = right_obstacle();
      motorR.setTicks(-(ticks_before - dist));
      UPDATE_BNO();
      new_angle = BNO_X;
      // encoders *= 1.0 / cos(15.0 * (PI/180));
      tstart = millis();
      limit_detected = true;
      Serial.println("obstacle is done");
    }
#endif

    p = speed * (double)(abs(encoders) - abs(motorR.getTicks())) / abs(encoders);
    //i = i + p;
    //d = p - last_dist;
    PID = p * KP_FORWARD;
    //Serial.println(PID);

    if (returnColor(true) == 1) {
      while (/* motorR.getTicks() > 0 && */ motorL.getTicks() > 0 && tofCalibrated(5) >= 50) {
        forward(-speed);
      }
      stopMotors();
      //pi_send_data(false, false);
      black_tile_detected = true;
      pi_send_tag("ramp");
      PI_SERIAL.print(0.0);
      PI_SERIAL.print(',');
      PI_SERIAL.print(0.0);
      PI_SERIAL.print(',');
      PI_SERIAL.println(0.0);
      resetBoost();
      return;
    }


    //we are close enough to the target at this point, so quit the loop
    if (/* abs(p_turn * DRIVE_STRAIGHT_KP) <= 0.01 && */ PID <= 0.01)
      break;
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    // Serial.println(speed * (double)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    if (abs(BNO_Z) < 5) {
      while (PI_SERIAL.available()) {
        auto right_ticks = motorR.getTicks();
        auto left_ticks = motorL.getTicks();
        stopMotors();
        pi_read_vision();
        if (restart)
          return;
        oled_println("detected");
        motorR.getTicks() = right_ticks;
        motorL.getTicks() = left_ticks;
      }
    }

    // double new_angle = global_angle;

    double reading;
    UPDATE_BNO();

    if (BNO_X - new_angle > 180) {
      reading = BNO_X - 360;
    } else if (BNO_X - new_angle < -180) {
      reading = BNO_X + 360;
    } else {
      reading = BNO_X;
    }

    double error = (reading - new_angle) * (BNO_KP + abs(reading - new_angle) / 270.0);
    if (error > 100) {
      error = 100;
    } else if (error < -100) {
      error = -100;
    }

    if (limit_detected) {
      if (millis() - tstart < 5000) {
        forward(SPEED * 0.85);
      } else {
        forward(200);
      }
    } else {
      if (millis() - tstart < 5000) {
        forward(PID + error, PID - error);
      } else {
        forward(200 + error, 200 - error);
      }
    }
  }

  stopMotors();
  resetBoost();

  pi_send_tag("ramp");
  PI_SERIAL.print(0.0);
  PI_SERIAL.print(",");
  PI_SERIAL.print(0.0);
  PI_SERIAL.print(",");
  PI_SERIAL.println(0.0);
#else
  pi_send_data(true, true);
  delay(100);
  pi_send_data(false, true);
#endif
}

int closestToDirection(double num) {
  for (int i = 0; i <= 360; i += 90) {
    if (abs(num - i) <= 45) {
      return i;
    }
  }
}

void alignAngle(bool reset, int tolerance = 10, int32_t start_yaw = INT32_MAX) {
  int tofR1, tofR2;
  int tofR3, tofR4;
  int lnum = 1, rnum = 0;
  float kP = 0.7;
  float BNO_KP = 1.4;
  addBoost(ALIGN_TURN_BOOST);

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
    oled_println("BNO ALIGNMENT!");
    UPDATE_BNO();

    double new_angle = closestToDirection(BNO_X);
    if (start_yaw != INT32_MAX) {
      new_angle = start_yaw;
    }
    // double new_angle = global_angle;
    double reading;

    if (abs(new_angle - BNO_X) < 3) {
      return;
    }

    double tstart = millis();

    do {
      UPDATE_BNO();

      if (BNO_X - new_angle > 180) {
        reading = BNO_X - 360;
      } else if (BNO_X - new_angle < -180) {
        reading = BNO_X + 360;
      } else {
        reading = BNO_X;
      }

      double error = (reading - new_angle) * BNO_KP;

      if (millis() - tstart > 3000) {
        addBoost(ALIGN_TURN_BOOST + 60);
      }

      forward(error * BNO_KP, -error * BNO_KP);

      if (abs(error * BNO_KP) < 10) {
        break;
      }

    } while (abs(reading - new_angle) > 2);
    return;
  }

  float len = abs((int)tofCalibrated(lnum) - (int)tofCalibrated(rnum));
  Serial.print("Length: ");
  Serial.println(len);

  if (len <= tolerance) {
    //Serial.println("return");
    return;
  }

  //const int width = TOF_DISTANCE;
  //const int angle = atan(width/len) * (180/PI);
  addBoost(ALIGN_TURN_BOOST + 10);

  double tstart = millis();

  while (abs(len) >= tolerance) {
    if (millis() - tstart > 3000) {
      addBoost(ALIGN_TURN_BOOST + 60);
    }
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
    global_angle = 0;
    oled_println("BNO has reset!");
    delay(50);
  }
}

uint _tofRawValue(int select) {
  tcaselect(select);
  return tof.readRangeSingleMillimeters();
}

uint _tofCalibrated(int select) {
  uint dist = 0;
  uint cal = 0;
  const uint max_dist = 250;
  switch (select) {
    case 0:
      {
        tcaselect(0);
        cal = tof.readRangeSingleMillimeters();
        cal = min(cal, max_dist);
        return cal;
      }
    case 1:
      {
        tcaselect(1);
        cal = tof.readRangeSingleMillimeters();
        cal = min(cal, max_dist);
        return cal;
      }
    case 2:
      {
        tcaselect(2);
        dist = tof.readRangeSingleMillimeters();
        if (dist <= 80) {
          cal = (1.39 * dist) - 57.3;
        } else {
          cal = (0.925 * dist) - 22.5;
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
        cal = tof.readRangeSingleMillimeters();
        ;
        cal = min(cal, max_dist);
        return cal;
        //6/14 essentially fine...
      }
    default:
      {
        Serial.println("Invalid TOF sensor");
        oled_println("Invalid TOF sensor");
        return -1;
      }
  }
}

uint tofCalibrated(int select) {
  uint32_t dist = 0;
  const int samples = 2;
  for (int n = 0; n < samples; n++) {
    dist += _tofCalibrated(select);
  }
  dist /= 2;
  //oled_println(dist);
  return dist;
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

void kitDrop(int num, char side) {
  static int columnNum = 1;
  static int numDropped = 0;
  const int offset_for_stack[3] = { 10, 7, 0 };

  bool* arr = get_tof_vals(wall_tresh);

  bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };

  if (side == 'r') {
    if (!walls[1])
      return;
  }
  if (side == 'l') {
    if (!walls[3])
      return;
  }

  if (numDropped <= total) {
    myservo.attach(servopin);
    myservo2.attach(servopin2);
  }

  analogWrite(5, 50);
  if (num > 0) {
    if (side == 'r') {
      myservo2.write(50);
    } else {
      myservo2.write(0);
    }
  }

  for (int i = 0; i < num; i++) {
    if (numDropped && !(numDropped % num_per_column))
      columnNum++;

    if (numDropped > total) {
      myservo.write(0);
      delay(200);
      analogWrite(5, 0);
      return;
    }

    myservo.write(180 - (60 * columnNum + offset_for_stack[columnNum - 1]));
    Serial.print("columnNum");
    Serial.println(columnNum);
    Serial.println("numDropped");
    Serial.println(numDropped);
    delay(1000);
    myservo.write(175);
    delay(1000);
    numDropped++;
  }

  myservo.detach();
  myservo2.detach();
  delay(1000);
  analogWrite(5, 0);
}

//#define TEST
#ifndef TEST

int clear_oled_counter = 0;

void loop() {

  bool* arr = get_tof_vals(wall_tresh);

  // //n e s w
  bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };
  // not wrapped around and stuff
  oled_display_walls(walls);
  //acceleration_position();
  //pi_read_data();

  while (PI_SERIAL.available()) {
    //pi_read_vision();
    pi_read_data();
  }

  if (restart) {
    setup();
  }

  oled_print("dir: ");
  oled_println(dir_to_char(cur_direction));

#ifdef DEBUG_DISPLAY
  oled.setCursor(0, 0);
  clear_oled_counter++;
  if (clear_oled_counter > 5) {
    oled_clear();
    clear_oled_counter = 0;
  }
#endif

  delay(100);
}
#else

void loop() {


#ifndef NO_PI
#ifndef ALIGN_ANGLE

  //  drive(100 * CM_TO_ENCODERS, 110);
  //  delay(1000);
  returnColor();

// int clear_oled_counter = 0;

// for (int i = 0; i <= 5; i++) {
//   Serial.print(i);
//   Serial.print(": ");
//   Serial.print(_tofCalibrated(i));
//   Serial.print(", ");
// }

// drive(30 * CM_TO_ENCODERS, SPEED);
// delay(1000);
// Serial.println();
// kitDrop(1, 'r');
// delay(1000);
// returnColor(false);
// Serial.print("Front Left: ");
// Serial.print(digitalRead(FRONT_LEFT));
// Serial.print(" Front Right: ");
// Serial.print(digitalRead(FRONT_RIGHT));
// Serial.print(" Back Left: ");
// Serial.print(digitalRead(BACK_LEFT));
// Serial.print(" Back Right: ");
// Serial.println(digitalRead(BACK_RIGHT));

// drive(100 * CM_TO_ENCODERS, SPEED);
// delay(1000);
// Serial.println(motorR.getTicks());
// right(90, SPEED);
// delay(500);

// UPDATE_BNO();
// Serial.println(BNO_Z);

// int r,g,b,c;
// tcs.getRawData(&r, &g, &b, &c);
// oled_println();
// oled_print(r);
// oled_print(" ");
// oled_print(g);
// oled_print(" ");
// oled_print(b);
// oled_println(" ");
// oled_print(c);
// bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
// // Serial.print("Orientation X:");
// // Serial.println(orientationData.orientation.x);


// oled.setCursor(0, 0);
// clear_oled_counter++;
// if(clear_oled_counter > 5)
// {
//   oled.clearDisplay();
//   clear_oled_counter = 0;
// }
// delay(200);
#else
  // alignAngle(true);
  // delay(1000);
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

  if (!walls[0]) {
    driveCM(tile_dist, 110);
  } else if (!walls[1]) {
    right(90, SPEED);
  } else if (!walls[3]) {
    left(90, SPEED);
  } else if (!walls[2]) {
    right(180, SPEED);
  }

  oled.setCursor(0, 0);
  clear_oled_counter++;
  if (clear_oled_counter > 5) {
    oled_clear();
    clear_oled_counter = 0;
  }

#endif
}

#endif
