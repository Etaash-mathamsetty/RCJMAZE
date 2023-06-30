#ifndef _TURN_H_
#define _TURN_H_

#include "common.h"
#include "utils.h"

using namespace std;

void backup_align(int speed, int time) {

  while (tofCalibrated(5) >= 80 /*|| digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT) */) {
    forward(-speed * 0.75);
  }
  stopMotors();
  delay(100);

  // uint32_t tstart = millis();
  // while((int32_t) millis() - (int32_t) tstart < 600) {
  //   forward(-speed * 0.75);
  // }

#ifndef NO_LIMIT
  // while ((int32_t)millis() - tstart < time) {
  //   if (!digitalRead(BACK_LEFT)) {
  //     motorL.run(-SPEED * 0.7);
  //   } else {
  //     motorL.stop();
  //   }

  //   if (!digitalRead(BACK_RIGHT)) {
  //     motorR.run(-SPEED * 0.7);
  //   } else {
  //     motorR.stop();
  //   }
  // }
#else
  // while ((int32_t)millis() - tstart < time) {
  //   forward(-SPEED * 0.7);
  // }


#endif

  // stopMotors();
  // bno.begin(OPERATION_MODE_IMUPLUS);
  // global_angle = 0;
  // delay(50);
}

void raw_right(double relative_angle, int speed, bool alignment) {

if (abs(relative_angle) < 1) {
  return;
}

#ifndef TURN_TEST
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

    p = (angle - orientation) / relative_angle;
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

    /*if (digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 4 * CM_TO_ENCODERS) {
        forward(SPEED * 0.75);
      }
    } else*/ 
    if (digitalRead(FRONT_LEFT) || digitalRead(FRONT_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 1.5 * CM_TO_ENCODERS) {
        forward(-SPEED * 0.75);
      }
    } 
    // else if (tofCalibrated(5) < 60) {
    //   while (tofCalibrated(5) < 60) {
    //     forward(SPEED * 0.75);
    //   }
    // }

#ifndef NO_PID

    if (millis() - tstart < 3000) {
      forward((PID * -speed), (PID * speed));
    } else {
      if (alignment) {
        addBoost(ALIGN_TURN_BOOST + 150);
      } else {
        addBoost(TURN_BOOST + 150);
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
#else 

  if (!alignment) {
    motorL.addBoost(TURN_BOOST);
    motorR.addBoost(TURN_BOOST);
  } else {
    motorL.addBoost(ALIGN_TURN_BOOST);
    motorR.addBoost(ALIGN_TURN_BOOST);
    speed = ALIGN_SPEED;
  }

  UPDATE_BNO();
  int32_t new_angle = BNO_X + relative_angle;
  if (new_angle < 0) {
    new_angle += 360;
  } else if (new_angle >= 360) {
    new_angle -= 360;
  }

  double reading;
  int32_t tstart = millis();
  const double BNO_STATIC_KP = 30;

  do {
    UPDATE_BNO();
    if (BNO_X - new_angle > 180) {
      reading = BNO_X - 360;
    } else if (BNO_X - new_angle < -180) {
      reading = BNO_X + 360;
    } else {
      reading = BNO_X;
    }

    double bno_error = (reading - new_angle) * (BNO_STATIC_KP + abs(reading - new_angle) / 300.0);
    if ((int32_t) millis() - (int32_t) tstart > 5000) {
      addBoost(TURN_BOOST + 80);
    }
    forward(bno_error, -bno_error);
  } while (abs(reading - new_angle) > 1);

  resetBoost();

#endif
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

  raw_right(relative_angle, speed, false);

  if (tofCalibrated(5) <= wall_tresh - 30) {
    backup_align(SPEED, 600);
    // while(tofCalibrated(5) >= 70)
    // {
    //   forward(-speed);
    // }

    while (tofCalibrated(5) <= 80) {
      forward(speed);
    }
    stopMotors();
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
}

void raw_left(double relative_angle, int speed, bool alignment) {

  if (abs(relative_angle) < 1) {
    return;
  }

#ifndef TURN_TEST
#ifndef MOTORSOFF

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

    p = (orientation - angle) / relative_angle;
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

   /* if (digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 4 * CM_TO_ENCODERS) {
        forward(SPEED * 0.75);
      }
    } else*/ 
    if (digitalRead(FRONT_LEFT) || digitalRead(FRONT_RIGHT)) {
      resetTicks();
      while (abs(motorR.getTicks()) < 1.5 * CM_TO_ENCODERS) {
        forward(-SPEED * 0.75);
      }
    } 
    // else if (tofCalibrated(5) < 60) {
    //   while (tofCalibrated(5) < 60) {
    //     forward(SPEED * 0.75);
    //   }
    // }

#ifndef NO_PID
    if (millis() - tstart < 3000) {
      forward((PID * speed), (PID * -speed));
    } else {
      if (alignment) {
        addBoost(ALIGN_TURN_BOOST + 150);
      } else {
        addBoost(TURN_BOOST + 150);
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
#else

  if (!alignment) {
    motorL.addBoost(TURN_BOOST);
    motorR.addBoost(TURN_BOOST);
  } else {
    motorL.addBoost(ALIGN_TURN_BOOST);
    motorR.addBoost(ALIGN_TURN_BOOST);
    speed = ALIGN_SPEED;
  }

  UPDATE_BNO();
  int32_t new_angle = BNO_X - relative_angle;
  if (new_angle < 0) {
    new_angle += 360;
  } else if (new_angle >= 360) {
    new_angle -= 360;
  }

  double reading;
  int32_t tstart = millis();
  const double BNO_STATIC_KP = 30;

  do {
    UPDATE_BNO();
    if (BNO_X - new_angle > 180) {
      reading = BNO_X - 360;
    } else if (BNO_X - new_angle < -180) {
      reading = BNO_X + 360;
    } else {
      reading = BNO_X;
    }

    double bno_error = (reading - new_angle) * (BNO_STATIC_KP + abs(reading - new_angle) / 300.0);

    if ((int32_t) millis() - (int32_t) tstart > 5000) {
      addBoost(TURN_BOOST + 80);
    }

    forward(bno_error, -bno_error);
  } while (abs(reading - new_angle) > 1);

  resetBoost();

#endif
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

  raw_left(relative_angle, speed, false);

  if (tofCalibrated(5) <= wall_tresh - 30) {
    backup_align(SPEED, 600);
    // while(tofCalibrated(5) >= 70)
    // {
    //   forward(-speed);
    // }
    while (tofCalibrated(5) <= 80) {
      forward(speed);
    }
    stopMotors();
  }

  if (turn_status) {
    pi_send_tag("turn_status");
    PI_SERIAL.println(0.0);
  }
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

//DO NOT USE IN ANYTHING OTHER THAN drive()
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

    if (/*digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT) || */ tofCalibrated(5) <= 80) {
      break;
    }
  }
  forward_ticks = abs(motorR.getTicks());

  stopMotors();
  delay(500);

  raw_right(15, SPEED, true);

  stopMotors();
  delay(500);

  empty_serial_buffer();
  //ignore any commands if pi saw something
  pi_send_drop_status(false, false);

  return abs(forward_ticks);
}

//DO NOT USE IN ANYTHING OTHER THAN drive()
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

    if (/*digitalRead(BACK_LEFT) || digitalRead(BACK_RIGHT) ||*/ tofCalibrated(5) <= 80) {
      break;
    }
  }

  forward_ticks = abs(motorR.getTicks());

  stopMotors();
  delay(500);

  raw_left(15, SPEED, true);

  stopMotors();
  delay(500);

  empty_serial_buffer();
  //ignore any commands if pi saw something
  pi_send_drop_status(false, false);

  return abs(forward_ticks);
}

void alignAngle(bool reset, int tolerance = 10, double start_yaw = INFINITY) {
  int tofR1, tofR2;
  int tofR3, tofR4;
  int lnum = 1, rnum = 0;
  float kP = 0.7;
  float BNO_KP = 1.4;
  addBoost(ALIGN_TURN_BOOST);

  tofR1 = tofCalibrated(0, 5);
  tofR2 = tofCalibrated(1, 5);
  tofR3 = tofCalibrated(2, 5);
  tofR4 = tofCalibrated(3, 5);
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
    if(start_yaw != INFINITY)
    {
      new_angle = start_yaw;
    }
    // double new_angle = global_angle;
    double reading = 0.0;

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

#endif