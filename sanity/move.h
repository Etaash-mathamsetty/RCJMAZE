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

#ifndef NO_LIMIT
    if (digitalRead(FRONT_RIGHT) == HIGH && abs(BNO_Z) < 4) {

      if (abs(encoders - abs(motorR.getTicks())) < 1.25 * CM_TO_ENCODERS) {
        pi_send_tag("ramp");
        PI_SERIAL.print(0.0);
        PI_SERIAL.print(",");
        PI_SERIAL.print(0.0);
        PI_SERIAL.print(",");
        PI_SERIAL.println(0.0);
        return;
      } 

      ticks_before = abs(motorR.getTicks());
      int dist = left_obstacle();
      motorR.setTicks(ticks_before - dist);
      // encoders *= 1.0 / cos(10.0 * (PI/180.0));
      UPDATE_BNO();
      new_angle = BNO_X;
      tstart = millis();
      limit_detected = true;
      Serial.println("obstacle is done");
    }

    if (digitalRead(FRONT_LEFT) == HIGH && abs(BNO_Z) < 4) {

      if (abs(encoders - abs(motorR.getTicks())) < 1.25 * CM_TO_ENCODERS) {
        pi_send_ramp(0.0, 0.0, 0.0);
        return;
      } 

      ticks_before = abs(motorR.getTicks());
      int dist = right_obstacle();
      motorR.setTicks(ticks_before - dist);
      UPDATE_BNO();
      new_angle = BNO_X;
      // encoders *= 1.0 / cos(10.0 * (PI/180.0));
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

    double dist_percent = 0.0;

    if(ticks_before <= motorR.getTicks())
      dist_percent = (double)abs(motorR.getTicks()) / (double)abs(encoders);
    //otherwise, just assume 0

    pi_send_tag("dist_percent");
    PI_SERIAL.println(dist_percent);

    if (returnColor(true) == 1) {
      while (/* motorR.getTicks() > 0 && */ motorL.getTicks() > 0 && tofCalibrated(5) >= 80) {
        forward(-speed);
      }
      // stopMotors();
      //pi_send_data(false, false);
      black_tile_detected = true;
      pi_send_ramp(0.0, 0.0, 0.0);
      resetBoost();
      return;
    }


    //we are close enough to the target at this point, so quit the loop
    if (/* abs(p_turn * DRIVE_STRAIGHT_KP) <= 0.01 && */ PID <= 0.01)
      break;
    // speed = speed * (abs(encoders) - abs(motor1.getTicks()))/abs(encoders);

    // Serial.println(speed * (double)(abs(encoders) - abs(motor1.getTicks()))/abs(encoders));
    UPDATE_BNO();

    if (abs(BNO_Z) < 5) {

      //TODO: find correct values
      if(dist_percent <= 0.1 || dist_percent >= 0.9)
      {
        empty_serial_buffer();
        pi_send_drop_status(false, false);
      }
      else
      {
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
          tstart = millis();
        }
      }
    }
    else
    {
      //litterally impossible, ramp code is handled above
      oled_clear();
      oled_println("impossible!");
      empty_serial_buffer();
      pi_send_drop_status(false, false);
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

  pi_send_ramp(0.0, 0.0, 0.0);
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
  UPDATE_BNO();
  // ramp detection
  if (_tofCalibrated(4) > 220 && _tofCalibrated(4) <= 470 && _tofCalibrated(6) < 255  && _tofCalibrated(6) > 190 && abs(BNO_Z) < 5) {
    oled_clear();
    oled_println("up ramp detected!");
    delay(500);
    UPDATE_BNO();
    handle_up_ramp(BNO_Z);
    // pi_send_forward_status(false, !black_tile_detected);
    // black_tile_detected = false;
    return;
  }

  if (_tofCalibrated(4) >= 490 && _tofCalibrated(6) >= 470 && _tofCalibrated(6) <= 2000) {
    oled_clear();
    oled_println("down ramp detected!");
    delay(500);
    UPDATE_BNO();
    handle_down_ramp(BNO_Z);
    // pi_send_forward_status(false, !black_tile_detected);
    // black_tile_detected = false;
    return;
  }

#if 1
  const float mult_factor = 1.0;
  uint right = (tofCalibrated(2, 10) + tofCalibrated(3, 10)) / 2;
  uint left = (tofCalibrated(0, 10) + tofCalibrated(1, 10)) / 2;
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
  UPDATE_BNO();

  bool optimal_alignment = horizontalError >= tolerance && abs(orientationData.orientation.z) < 12;
  if (optimal_alignment && left <= wall_tresh && right <= wall_tresh && abs(left - right) > 25) {

    if (left < right) {

      raw_right(90 - min(90, angle * mult_factor), SPEED, true);

      if (tofCalibrated(4) > wall_tresh - 50)
        drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI / 180))) - forward_offset, speed);
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

        pi_send_ramp(0.0, 0.0, 0.0);
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
        drive((cm * CM_TO_ENCODERS) / abs(sin(angle * (PI / 180))) - forward_offset, speed);
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


        pi_send_ramp(0.0, 0.0, 0.0);
        pi_send_forward_status(false, true, true);
        return;
      }

      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      if (abs(orientationData.orientation.z) < 12)
        raw_right(90 - min(90, angle * mult_factor), SPEED, true);
      // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      // raw_right(360-orientationData.orientation.x, SPEED);
    }
  } else if (optimal_alignment && tofCalibrated(0) <= wall_tresh && tofCalibrated(1) <= wall_tresh && 
            right >= wall_tresh && abs(left - target_dist_from_wall) > 30.0) {
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
      drive(cm / sin(angle * (PI / 180)) * CM_TO_ENCODERS - forward_offset, speed);
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

      pi_send_ramp(0.0, 0.0, 0.0);
      pi_send_forward_status(false, true, true);
      return;
    }
    if (left - target_dist_from_wall > 0.0) {
      raw_right(90.0 - angle, SPEED, true);
    } else if (left - target_dist_from_wall < 0.0) {
      raw_left(90.0 - angle, SPEED, true);
    }

  } else if (optimal_alignment && left >= wall_tresh && tofCalibrated(2) <= wall_tresh
            && tofCalibrated(3) <= wall_tresh && abs(right - target_dist_from_wall) > 30.0) {
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
      drive(cm / sin(angle * (PI / 180)) * CM_TO_ENCODERS - forward_offset, speed);
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

      pi_send_ramp(0.0, 0.0, 0.0);
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

      pi_send_ramp(0.0, 0.0, 0.0);
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

  pi_send_forward_status(false, !black_tile_detected);
  black_tile_detected = false;

  UPDATE_BNO();
  if (abs(orientationData.orientation.z) < 12) {
    //pi_read_data();
    alignAngle(true, 10, start_yaw);
  }

  //pause for blue
  if(returnColor() == 3)
  {
    stopMotors();
    delay(5000);
  }
}
