#ifndef _DROPPER_H_
#define _DROPPER_H_

#include "common.h"
#include "tof.h"
#include "utils.h"

using namespace utils;

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

#endif