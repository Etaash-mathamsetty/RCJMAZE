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
// #define TURN_TEST
#define AMS

//define: debug display, motorsoff, test, comment out all others if you want to calibrate tofs

#include "common.h"
#include "Motors.h"
#include "utils.h"
#include "tof.h"
#include "color.h"
#include "comm.h"
#include "dropper.h"
#include "ramp.h"
#include "move.h"
#include "turn.h"

using namespace utils;

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

  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  
  analogWrite(2, 50);

  //buzzer pin 

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
  // pinMode(BACK_RIGHT, INPUT_PULLUP);
  // pinMode(BACK_LEFT, INPUT_PULLUP);

  if (digitalRead(FRONT_LEFT)) {
    Serial.println("front left limit disconnected");
    oled_println("front left disconnect");
  }

  if (digitalRead(FRONT_RIGHT)) {
    Serial.println("front right limit disconnected");
    oled_println("front right disconnect");
  }

  // if (digitalRead(BACK_LEFT)) {
  //   Serial.println("back left limit disconnected");
  //   oled_println("back left disconnect");
  // }

  // if (digitalRead(BACK_RIGHT)) {
  //   Serial.println("back right limit disconnected");
  //   oled_println("back right disconnect");
  // }

  if (!(digitalRead(FRONT_LEFT) || digitalRead(FRONT_RIGHT) )/*|| digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT))*/) {
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
    delay(10);

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

  Serial.println("TOF INIT SUCCEED!");
  oled_println("Startup Done!");

  pinMode(3, OUTPUT); 
  analogWrite(3, 255);

  delay(1000);
  oled_clear();

  analogWrite(2, 0);
  // delay(500);

}

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
        pi_send_drop_status(true, false);

        bool ret = kitDrop(num, 'l');

        cur_cmd.remove(0);

        pi_send_drop_status(false, ret);
      }
    } else if (c == 'r') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_drop_status(true, false);

        bool ret = kitDrop(num, 'r');
        
        cur_cmd.remove(0);

        pi_send_drop_status(false, ret);
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
        pi_send_walls(walls);

        //checkpoint detection
        //pi_send_tag("CP");
        //PI_SERIAL.println(float(returnColor() == 2));

        if (returnColor() == 2) {
          pi_send_tag("CP");
          PI_SERIAL.println("1.0");
          oled_clear();
          stopMotors();
          delay(200);
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
          empty_serial_buffer();
          return;
        }
      }
      cur_cmd = "";
    }
  }
}

void loop() {

  both_println("please move near box");
  delay(5000);

  if (abs(tofCalibrated(0) - tofCalibrated(1)) < 10) {
    both_println("left side calibrated");
  } else {
    both_println("left side not calibrated");
  }

  delay(10);

  if (abs(tofCalibrated(3) - tofCalibrated(2)) < 10) {
    both_println("right side calibrated");
  } else {
    both_println("right side not calibrated");
  }
  delay(10);

  oled_clear();
  both_println("place under black");
  delay(3000);

  if (returnColor() == 1) {
    both_println("black detected!");
  } else {
    both_println("black not detected!");
  }

  oled_clear();
  both_println("place under silver");
  delay(3000);

  if (returnColor() == 2) {
    both_println("silver detected!");
  } else {
    both_println("silver not detected!");
  }

  oled_clear();
  both_println("place under blue");
  delay(3000);

  if (returnColor() == 3) {
    both_println("blue detected!");
  } else {
    both_println("blue not detected!");
  }
  delay(3000);

  oled_clear();
  if (digitalRead(FRONT_LEFT) == HIGH) {
    both_println("front left limit broken");
  } else {
    both_println("front left limit working");
  }

  if (digitalRead(FRONT_RIGHT) == HIGH) {
    both_println("front right limit broken");
  } else {
    both_println("front right limit working");
  }

  delay(3000);

  while (motorR.getTicks() < )


}