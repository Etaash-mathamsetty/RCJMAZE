#ifndef _DROPPER_H_
#define _DROPPER_H_

#include "common.h"
#include "tof.h"
#include "utils.h"

using namespace utils;

bool kitDrop(int num, char side) {
  static int columnNum = 1;
  static int numDropped = 0;
  const int offset_for_stack[3] = { 10, 7, 0 };

  bool* arr = get_tof_vals(wall_tresh);

  bool walls[4] = { arr[4], arr[2] || arr[3], arr[5], arr[0] || arr[1] };

  if (side == 'r') {
    if (!walls[1])
      return false;
  }
  if (side == 'l') {
    if (!walls[3])
      return false;
  }

  if(side == 'l')
  {
    if(seen_l[num] == side)
      return false;
  }
  else
  {
    if(seen_r[num] == side)
      return false;
  }

  // avoid misdetecting orange obstacles
  if (num == 1 && side == 'r') {
    if (abs(tofCalibrated(2) - tofCalibrated(3)) > 50) {
      oled.clearDisplay();
      oled.setCursor(0,0);
      oled.println("obstacle seen r");
      delay(2000);
      return false;
    }
  } else if (num == 1 && side == 'l') {
    if (abs(tofCalibrated(0) - tofCalibrated(1)) > 50) {
      oled.clearDisplay();
      oled.setCursor(0,0);
      oled.println("obstacle seen l");
      delay(2000);
      return false;
    }
  }


  analogWrite(3, 150);
  delay(200);
  analogWrite(3, 255);

  if (numDropped <= total) {
    myservo.attach(servopin);
    myservo2.attach(servopin2);
  }

  analogWrite(5, 50);
  analogWrite(2, 50);
  analogWrite(4, 50);
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
      delay(50);
      myservo2.write(25);
      delay(50);
      myservo2.detach();
      myservo.detach();
      delay(max(0, 3000 - 2000*i - 100));
      analogWrite(5, 0);
      analogWrite(2, 0);
      analogWrite(4, 0);
      return true;
    }

    myservo.write(180 - (60 * columnNum + offset_for_stack[columnNum - 1]));
    Serial.print("columnNum");
    Serial.println(columnNum);
    Serial.println("numDropped");
    Serial.println(numDropped);
    delay(1000);
    myservo.write(180);
    delay(1000);
    numDropped++;
  }

  myservo.detach();
  myservo2.detach();
  
  if(num > 0)
    delay(1000);
  else
    //required by rules
  delay(3000);
  analogWrite(5, 0);
  analogWrite(2, 0);
  analogWrite(4, 0);

  if(side == 'l')
    seen_l[num] = side;
  else
    seen_r[num] = side;

  for (int i = 0; i < ARRAY_SIZE(seen_l); i++) {
    Serial.print(seen_l[i]);
    Serial.print(" ");
  }
  Serial.println();

  for(int i = 0; i < ARRAY_SIZE(seen_r); i++) {
    Serial.print(seen_r[i]);
    Serial.print(" ");
  }
  Serial.println();

  return true;
}

#endif