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
        dist = tof.readRangeSingleMillimeters();
        cal = dist - 17;
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
        cal -= 4;
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
        // cal = min(cal, max_dist);
        return cal;
        //not in need of calibration 6/14
      }
    case 5:
      {
        //TODO: calibrate (low priority)
        tcaselect(5);
        cal = tof.readRangeSingleMillimeters();
        cal = min(cal, max_dist);
        return cal;
        //6/14 essentially fine...
      }
    case 6:
      {
        tcaselect(6);
        cal = tof.readRangeSingleMillimeters();
        // while (cal > 2000) {
        //   cal = tof.readRangeSingleMillimeters();
        // }
        // cal = min(cal, 500);
        return cal;
      }
    default:
      {
        Serial.println("Invalid TOF sensor");
        oled_println("Invalid TOF sensor");
        return -1;
      }
  }
}

uint tofCalibrated(int select, int samples = 1, int* invalid_count = nullptr) {
  uint64_t dist = 0;
  uint32_t val = 0;
  if(invalid_count)
    invalid_count = 0;
  for (int n = 0; n < samples; n++) {
    val = _tofCalibrated(select);
    if(invalid_count && val >= 5000)
    {
      *invalid_count++;
    }
    dist += val;
  }
  dist /= samples;
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