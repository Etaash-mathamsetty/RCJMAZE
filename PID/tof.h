#ifndef _TOF_H_
#define _TOF_H_

#include "common.h"

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

#endif