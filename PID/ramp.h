#include "common.h"
#include "utils.h"
#include "tof.h"
#include "turn.h"

using namespace utils;

bool handle_up_ramp(double start_pitch) {
  int32_t ticks = 15 * CM_TO_ENCODERS;
  int32_t back_up = 0;
  int32_t delta_x = 0;
  int32_t delta_theta = 0;
  int32_t delta_time = 10;
  const double BNO_KP = 5;

  UPDATE_BNO();
  double new_angle = closestToDirection(BNO_X);

  double distance = 0;
  double height = 0;
  auto old_ticks = motorR.getTicks();
  resetTicks();

  // while (abs(motorR.getTicks()) < abs(ticks)) {
  //   forward(SPEED + 25);
  // }

  //move until ramp
  while (BNO_Z - start_pitch >= -6.7 && tofCalibrated(4) >= 45) {
    UPDATE_BNO();
    forward(SPEED + 25);
  }

  double old_x = motorR.getTicks();
  const float wall_kp = 0.15f;
  UPDATE_BNO();

  //move until not ramp
  while (BNO_Z - start_pitch <= -6.5 && tofCalibrated(4) >= 90) {
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
    UPDATE_BNO();

    // calculate distance on a ramp
    double delta_x = abs(motorR.getTicks()) - abs(old_x);
    old_x = motorR.getTicks();
    double delta_theta = abs(BNO_Z - start_pitch);
    height += delta_x * sin(delta_theta * (PI / 180.0));
    distance += delta_x * cos(delta_theta * (PI / 180.0));
    Serial.print("Rampe distance: ");
    Serial.println(distance);
  }


  if ((distance / (30.0 * CM_TO_ENCODERS)) - 0.25 <= 0.65) {
    motorR.getTicks() = - old_ticks - ticks - 4 * CM_TO_ENCODERS;
    motorL.resetTicks();

    oled_clear();
    oled_print("Fake Ramp: ");
    oled_println((distance / (30.0 * CM_TO_ENCODERS)) - 0.25);
    oled_print("Distance: ");
    oled_println(distance);
    oled_print("Height: ");
    oled_println(height);
    stopMotors();
    delay(5000);
    pi_send_ramp(0.0, 0.0, 0.0);
    pi_send_forward_status(false, !black_tile_detected);
    black_tile_detected = false;
    return true;
  }

  stopMotors();
  delay(100);

  UPDATE_BNO();

  stopMotors();
  oled_clear();
  oled_print("Ramps: ");
  oled_println((distance / (30.0 * CM_TO_ENCODERS)) - 0.25);
  oled_println(height / (30.0 * CM_TO_ENCODERS));
  oled_print("Delta Angle");
  oled_print(BNO_Z - start_pitch);
  delay(5000);
  pi_send_ramp(1.0, ((distance / (30.0 * CM_TO_ENCODERS)) - 0.25), (height / (30.0 * CM_TO_ENCODERS)));
  height = round(height / (30.0 * CM_TO_ENCODERS));
  if (height == 0) {
    height = 1;
  }
  PI_SERIAL.println(height);
  alignAngle(true);

  if (tofCalibrated(4) <= wall_tresh) {
    while (tofCalibrated(4) >= 90) {
      forward(SPEED * 0.7);
    }
    stopMotors();
  } else {
    resetTicks();
    while(abs(motorR.getTicks()) < 5 * CM_TO_ENCODERS) {
      forward(SPEED * 0.7);
    }
    stopMotors();
  }

  return true;
}

bool handle_down_ramp(double start_pitch) {
  int32_t ticks = 15 * CM_TO_ENCODERS;
  int32_t back_up = 0;
  auto old_ticks = motorR.getTicks();
  int32_t delta_time = 10;
  double distance = 0;
  double height = 0;
  UPDATE_BNO();
  double new_angle = closestToDirection(BNO_X);

  motorR.resetTicks();
  // while (abs(motorR.getTicks()) < abs(ticks)) {
  //   forward(70);
  // }

  // move until ramp

  while (BNO_Z - start_pitch <= 6.7 && tofCalibrated(4) >= 45) {
    UPDATE_BNO();
    forward(50);
  }

  UPDATE_BNO();

  double old_x = motorR.getTicks();
  const float wall_kp = 0.15f;
  const double BNO_KP = 5;

  // move until not ramp

  while (BNO_Z - start_pitch >= 6.5 && tofCalibrated(4) >= 90) {
    UPDATE_BNO();

    double reading;

    if (BNO_X - new_angle > 180) {
      reading = BNO_X - 360;
    } else if (BNO_X - new_angle < -180) {
      reading = BNO_X + 360;
    } else {
      reading = BNO_X;
    }

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

    // calculate distance on a ramp
    UPDATE_BNO();
    double delta_theta = abs(BNO_Z - start_pitch);

    forward(70.0 + bno_error + err, 70.0 - bno_error - err);

    double delta_x = abs(motorR.getTicks()) - abs(old_x);
    old_x = motorR.getTicks();
    distance += delta_x * cos(delta_theta * (PI / 180.0));
    height += delta_x * sin(delta_theta * (PI / 180.0));
    Serial.print("Ramp length: ");
    Serial.println(distance);
  }

  // forwardTicks(SPEED, 7 * CM_TO_ENCODERS);

  if ((distance / (30.0 * CM_TO_ENCODERS)) <= 0.25 && (height / (30.0 * CM_TO_ENCODERS)) < 0.2) {
    motorR.getTicks() = old_ticks - ticks - 4 * CM_TO_ENCODERS;

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
    pi_send_ramp(0.0, 0.0, 0.0);
    pi_send_forward_status(false, !black_tile_detected);
    black_tile_detected = false;

    return true;
  }

  stopMotors();
  oled_clear();
  oled_print("Ramps: ");
  oled_println((distance / (30.0 * CM_TO_ENCODERS)));
  oled_print(height / (30.0 * CM_TO_ENCODERS));
  delay(5000);
  pi_send_ramp(10.0, (distance / (30.0 * CM_TO_ENCODERS)), (height / (30.0 * CM_TO_ENCODERS)));
  height = round(height / (30.0 * CM_TO_ENCODERS));
  if (height == 0) {
    height = 1;
  }
  PI_SERIAL.println(height);
  alignAngle(true);

  if (tofCalibrated(4) <= wall_tresh) {
    while (tofCalibrated(4) >= 90) {
      forward(SPEED * 0.7);
    }
    stopMotors();
  } else {
    resetTicks();
    while(abs(motorR.getTicks()) < 5 * CM_TO_ENCODERS) {
      forward(SPEED * 0.7);
    }
    stopMotors();
  }

  return true;

}