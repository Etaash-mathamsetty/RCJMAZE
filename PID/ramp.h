#include "common.h"
#include "utils.h"
#include "tof.h"
#include "turn.h"

using namespace utils;

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

    while (BNO_Z - start_pitch <= -5.5 && tofCalibrated(4) >= 90) {
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

      if (digitalRead(FRONT_RIGHT) == HIGH && digitalRead(FRONT_LEFT) == LOW && abs(BNO_Z) < 4) {
        raw_left(15, SPEED * 0.75, true);
      } else if (digitalRead(FRONT_LEFT) == HIGH && digitalRead(FRONT_RIGHT) == LOW && abs(BNO_Z) < 4) {
        raw_right(15, SPEED * 0.75, true);
      }

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

    // double BNO_STATIC_KP = 30;
    // double reading;
    // do {
    //   UPDATE_BNO();
    //   if (BNO_X - new_angle > 180.0) {
    //     reading = BNO_X - 360;
    //   } else if (BNO_X - new_angle < -180.0) {
    //     reading = BNO_X + 360;
    //   } else {
    //     reading = BNO_X;
    //   }

    //   double bno_error = (reading - new_angle) * (BNO_STATIC_KP + abs(reading - new_angle) / 270.0);
    //   forward(bno_error, -bno_error);
    // } while (abs(BNO_X - new_angle) > 1);

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
  double new_angle = closestToDirection(BNO_X);

  motorR.resetTicks();
  while (abs(motorR.getTicks()) < abs(ticks)) {
    forward(SPEED + 50);
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
    while (BNO_Z - start_pitch >= 4 && tofCalibrated(4) >= 90) {
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

      if (digitalRead(FRONT_RIGHT) == HIGH && digitalRead(FRONT_LEFT) == LOW && abs(BNO_Z) < 4) {
        raw_left(15, SPEED * 0.75, true);
      } else if (digitalRead(FRONT_LEFT) == HIGH && digitalRead(FRONT_RIGHT) == LOW && abs(BNO_Z) < 4) {
        raw_right(15, SPEED * 0.75, true);
      }

      forward(120.0 + bno_error + err, 120.0 - bno_error - err);

      // calculate distance on a ramp
      double delta_x = abs(motorR.getTicks()) - abs(old_x);
      double delta_theta = abs(BNO_Z - start_pitch);
      distance += delta_x * cos(delta_theta * (PI / 180.0));
      height += delta_x * sin(delta_theta * (PI / 180.0));
      old_x = motorR.getTicks();
    }

    // forwardTicks(SPEED, 7 * CM_TO_ENCODERS);

    if ((distance / (30.0 * CM_TO_ENCODERS)) <= 0.65) {
      motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;
      // motorL.resetTicks();

      // while (abs(motorL.getTicks()) < 6 * CM_TO_ENCODERS) {
      //   forward(SPEED * -0.75);
      // }

      stopMotors();
      oled_clear();
      oled_print("Fake Ramp: ");
      oled_println((distance / (30.0 * CM_TO_ENCODERS)));
      oled_print("Distance: ");
      oled_println(distance);
      oled_print("Height: ");
      oled_println(height);
      stopMotors();
      delay(5000);

      return true;
    }

    // UPDATE_BNO();

    // double BNO_STATIC_KP = 30.0;
    // double reading;
    // do {
    //   UPDATE_BNO();
    //   if (BNO_X - new_angle > 180) {
    //     reading = BNO_X - 360;
    //   } else if (BNO_X - new_angle < -180) {
    //     reading = BNO_X + 360;
    //   } else {
    //     reading = BNO_X;
    //   }

    //   double bno_error = (reading - new_angle) * (BNO_STATIC_KP + abs(reading - new_angle) / 300.0);
    //   forward(bno_error, -bno_error);
    // } while (abs(reading - new_angle) > 1);

    stopMotors();
    alignAngle(true);
    oled_clear();
    oled_print("Ramps: ");
    oled_println((distance / (30.0 * CM_TO_ENCODERS)));
    oled_print(height / (30.0 * CM_TO_ENCODERS));
    delay(5000);
    pi_send_tag("ramp");
    PI_SERIAL.print(10.0);
    PI_SERIAL.print(",");
    PI_SERIAL.print(round((distance / (30.0 * CM_TO_ENCODERS))));
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