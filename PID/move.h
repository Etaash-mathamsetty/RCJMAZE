#include "common.h"
#include "comm.h"
#include "utils.h"
#include "color.h"
#include "tof.h"
#include "turn.h"

using namespace utils;

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
      while (/* motorR.getTicks() > 0 && */ motorL.getTicks() > 0 && tofCalibrated(5) >= 80) {
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
  pi_send_forward_status(true, true);
  delay(100);
  pi_send_forward_status(false, true);
#endif
}

void driveCM(float cm, int speed = 200, int tolerance = 10) {
  //kitDrop(1);
  double start_yaw = 0.0;
  UPDATE_BNO();
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    alignAngle(true);
    UPDATE_BNO();
    start_yaw = BNO_X;
  }

  pi_send_forward_status(true, true);
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
        pi_send_forward_status(false, true, true);
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

        pi_send_forward_status(false, true, true);
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
      pi_send_forward_status(false, true, true);
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
      pi_send_forward_status(false, true, true);
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

      pi_send_forward_status(false, true, true);
      return;
    }
  }

  if (tofCalibrated(4) <= wall_tresh) {
    while (tofCalibrated(4) >= 90) {
      forward(speed * 0.7);
    }
    stopMotors();
  }

  UPDATE_BNO();
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    alignAngle(true, 10, start_yaw);
  }

  //pause for blue if detected
  returnColor();

  //WARN: drive function can no longer be used on it's own (when communicating with stereo pi) !!!!!
  pi_send_forward_status(false, !black_tile_detected);
  black_tile_detected = false;
}