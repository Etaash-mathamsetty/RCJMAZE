#ifndef _COMM_H_
#define _COMM_H_

#include "common.h"
#include "dropper.h"

inline void empty_serial_buffer()
{
  while(PI_SERIAL.available())
  {
    PI_SERIAL.read();
    //delay(5);
  }
}

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

void pi_send_drop_status(bool status, bool success)
{
  pi_send_tag("drop_status");
  PI_SERIAL.print((double)status);
  PI_SERIAL.print(",");
  PI_SERIAL.println((double)success);
}

int pi_read_vision(double* left_vic_ticks, double* right_vic_ticks, double dist_percent) {
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
        if(left_vic_ticks && dist_percent - *left_vic_ticks < strip_vic_percent)
        {
          pi_send_drop_status(false, false);
          continue;
        }

        if(left_vic_ticks && (dist_percent < full_vic_percent_begin || dist_percent > full_vic_percent_end))
        {
          pi_send_drop_status(false, false);
          continue;
        }

        pi_send_drop_status(true, false);

        bool ret = true; //kitDrop(num, 'l');
        BT.println(num);

        cur_cmd.remove(0);

        pi_send_drop_status(false, ret);

        if(dist_percent != 0 && left_vic_ticks)
          *left_vic_ticks = (double)motorR.getTicks()/dist_percent;
      }
    } else if (c == 'r') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        if(right_vic_ticks && dist_percent - *right_vic_ticks < strip_vic_percent)
        {
          pi_send_drop_status(false, false);
          continue;
        }

        if(right_vic_ticks && (dist_percent > full_vic_percent_end || dist_percent < full_vic_percent_begin))
        {
          pi_send_drop_status(false, false);
          continue;
        }

        pi_send_drop_status(true, false);

        bool ret = true; // kitDrop(num, 'r');
        BT.println(num);

        cur_cmd.remove(0);

        pi_send_drop_status(false, ret);

        if(dist_percent != 0 && right_vic_ticks)
          *right_vic_ticks = (double)motorR.getTicks()/dist_percent;
      }
    } else if (c >= '0' && c <= '9') {
      if (cur_cmd.length() > 0 && cur_cmd[0] == 'd') {
        num = c - '0';
      }
    } else if (c == 'q') {
      Serial.println("Restarting...");
      delay(200);
      restart = true;
      empty_serial_buffer();
      return 0;
    }
  }

  return num;
  //get_tof_vals(wall_tresh);
}

#endif