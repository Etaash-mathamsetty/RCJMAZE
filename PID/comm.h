#ifndef _COMM_H_
#define _COMM_H_

#include "common.h"
#include "dropper.h"

inline void pi_send_tag(const char* tag) {
  PI_SERIAL.print(tag);
  PI_SERIAL.print("::");
}

inline void display_tag(const char* tag) {
  Serial.print(tag);
  Serial.print(": ");
}

//forward_status[0] = is forward command runnning
//forward_status[1] = true = no black tile
//forward_status[2] = true = failed for other reason, (wall in front for ex)
void pi_send_forward_status(bool forward, bool black_tile, bool failed = false) {
  pi_send_tag("forward_status");

  PI_SERIAL.print((double)forward);
  PI_SERIAL.print(',');
  PI_SERIAL.print((double)black_tile);
  PI_SERIAL.print(',');
  PI_SERIAL.println((double)failed);
}

void pi_send_ramp(float ramp_type, float length, float height) {
  pi_send_tag("ramp");
  PI_SERIAL.print(ramp_type);
  PI_SERIAL.print(",");
  PI_SERIAL.print(length);
  PI_SERIAL.print(",");
  PI_SERIAL.println(height);
}

void pi_send_walls(bool walls[4]) {
  pi_send_tag("W");
  PI_SERIAL.print(walls[math::wrapAround((int)n - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.print(walls[math::wrapAround((int)e - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.print(walls[math::wrapAround((int)s - (int)cur_direction, 4)]);
  PI_SERIAL.print(",");
  PI_SERIAL.println(walls[math::wrapAround((int)w - (int)cur_direction, 4)]);
}

void pi_read_vision() {
  String data = "";
  char ch = 0;
  int num = 0;

  while (PI_SERIAL.available() && ch != '\n') {
    ch = PI_SERIAL.read();
    data += ch;
    delay(5);
  }
  data.trim();
  data.toLowerCase();
  data += '\n';
  String cur_cmd = "";
  Serial.println(data);

  for (char c : data) {
    if (c == 'd') {
      cur_cmd.remove(0);
      cur_cmd += c;
    } else if (c == 'l') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        kitDrop(num, 'l');

        cur_cmd.remove(0);

        if (num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c == 'r') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        pi_send_tag("drop_status");
        PI_SERIAL.println("1.0");

        kitDrop(num, 'r');

        cur_cmd.remove(0);

        if (num == 0)
          delay(1000);

        pi_send_tag("drop_status");
        PI_SERIAL.println("0.0");
      }
    } else if (c >= '0' && c <= '9') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        num = c - '0';
      }
    } else if (c == 'q') {
      Serial.println("Restarting...");
      delay(200);
      restart = true;
      return;
    }
  }

  //get_tof_vals(wall_tresh);
}

#endif