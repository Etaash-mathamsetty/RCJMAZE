#ifndef _COLOR_H_
#define _COLOR_H_

#include "common.h"
#include "utils.h"

using namespace utils;

void readColors() {
#ifdef AMS
  //ams.drvOn();
  ams.startMeasurement();  //begin a measurement

  //wait till data is available
  bool rdy = false;
  while (!rdy) {
    // delay(1);
    rdy = ams.dataReady();
  }
  //ams.drvOff(); //uncomment this if you want to use the driver LED for readings

  //read the values!
  ams.readRawValues(amsValues);
#endif
}

int returnColor(bool only_black = false) {
  uint16_t r, g, b, c = 0;
  int silver_detect = 0;
  int black_detect = 0;
  int blue_detect = 0;

  const int persistance_count = 0;
#ifdef TEST
#ifdef TCS
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print("clear:");
  Serial.print(c);
  Serial.print(",");
  Serial.print("r/g:");
  Serial.print((float)r / g * 100.0);
  Serial.print(",");
  Serial.print("b/r:");
  Serial.print((double)b / r * 100.0);
  Serial.print(',');
  Serial.print("sub:");
  Serial.print((double)r / g * 100.0 - (double)b / r * 100.0);
#endif
#endif

#ifdef TCS
  UPDATE_BNO();
  for (int i = 0; i <= persistance_count; i++) {

    if (c >= 40 && abs((r / (double)g) * 100.0 - ((double)b / r) * 100.0) <= 10 && abs(BNO_Z) < 12) {
      //   silver_persistance++;
      // Serial.println("silver detected");
      // oled_println("silver");
      //pi_send_tag("color");
      //PI_SERIAL.println("silver");
      silver_detect++;  //change later
    }
    if (c < 20 && (double)b / r < 1.0 && abs(BNO_Z) < 12) {
      // Serial.println("black detected");
      //oled_println("black");
      //pi_send_tag("color");
      //PI_SERIAL.println("black");
      black_detect++;
    }
    if (c <= 50 && (double)b / r >= 1.5 && abs(BNO_Z) < 12) {
      blue_detect++;
    }
  }

  if (blue_detect >= persistance_count && !only_black) {
    Serial.println(" blue detected");
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println(" blue detected");
    stopMotors();
    delay(5000);
    return 3;
  } else if (black_detect >= persistance_count) {
    Serial.println(" black detected"); 
   // delay(5000);
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println(" black detected");
    return 1;
  } else if (silver_detect >= persistance_count && !only_black) {
    Serial.println(" silver detected");
    // oled.clearDisplay();
    // oled.setCursor(0, 0);
    // oled_println(" silver detected");
    return 2;
  } else {
    Serial.println(" white detected");
    return 0;
  }
#endif
#ifdef AMS
  UPDATE_BNO();

  readColors();
#ifdef TEST
  Serial.print("Violet:");
  Serial.print(amsValues[AS726x_VIOLET]);
  Serial.print(",Blue:");
  Serial.print(amsValues[AS726x_BLUE]);
  Serial.print(",Green:");
  Serial.print(amsValues[AS726x_GREEN]);
  Serial.print(",Yellow:");
  Serial.print(amsValues[AS726x_YELLOW]);
  Serial.print(",Orange:");
  Serial.print(amsValues[AS726x_ORANGE]);
  Serial.print(",Red:");
  Serial.print(amsValues[AS726x_RED]);
  Serial.println();
#endif

  bool violet_greatest = true;
  int dark_count = 0;
  int bright_count = 0;
  int speedbump_count = 0;

  for (int i = 0; i <= AS726x_RED; i++) {
    if (amsValues[AS726x_VIOLET] < amsValues[i]) {
      violet_greatest = false;
    }

    if (amsValues[i] <= 30) {
      dark_count++;
    }

    if (amsValues[i] >= 150) {
      bright_count++;
    }

    if (amsValues[i] >= 500) {
      speedbump_count++;
    }
  }

  if (dark_count >= 5 && abs(BNO_Z) < 7 && amsValues[AS726x_VIOLET] <= 32) {
    // stopMotors ();
    // oled_clear();
    // oled_println(" black detected");
    if (only_black) {
      int black_persistence = 0;
      for (int i = 0; i < 2; i++) {        
        int dark_count2 = 0;
        UPDATE_BNO();

        for (int i = 0; i <= AS726x_RED; i++) {
          if (amsValues[i] <= 30) {
            dark_count2++;
          }
        }

        if (dark_count2 >= 5 && abs(BNO_Z) < 7 && amsValues[AS726x_VIOLET] <= 32) {
          black_persistence++;
        } 
      }

      if (black_persistence >= 2) {
        Serial.println("black detected");
        return 1;
      } else {
        return 0;
      }
    }
    Serial.println(" black detected");
    return 1;
  } else if (bright_count >= 3 && abs((double) amsValues[AS726x_GREEN] - (double) amsValues[AS726x_YELLOW]) < 80 && abs(BNO_Z) < 12 && !only_black) {
    // stopMotors();
    oled_clear();
    oled_println(" silver detected");
    // delay(5000);
    Serial.println(" silver detected");
    return 2;
  } else if (violet_greatest && !only_black && amsValues[AS726x_VIOLET] > 35 && amsValues[AS726x_RED] <= 18 && amsValues[AS726x_BLUE] >= 16 && abs(BNO_Z) < 12) {
    //stopMotors();
    Serial.println(" blue detected");
    oled_clear();
    oled_println(" blue detected");
    //delay(5000);
    return 3;
  }
#endif

  return 0;
}


#endif