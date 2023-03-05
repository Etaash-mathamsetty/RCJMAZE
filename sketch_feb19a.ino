
#include "Motors.h"

Motor motorL(MPORT2);
Motor motorR(MPORT1, true, true);

const double CM_TO_ENCODERS = 100.0/4.6;
const double ENCODERS_TO_CM = 1/CM_TO_ENCODERS;

#define PI_SERIAL Serial2
#define DEBUG_SERIAL Serial
#define MUX_ADDR 0x70

void tcaselect(uint8_t n)
{
  if(n > 7)
    break;
  
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << n);
  Wire.endTransmission();
}

void pi_send_tag(const char* tag)
{
  PI_SERIAL.print(tag);
  PI_SERIAL.print("::");
}

void update_forward_status(bool forward, bool fail)
{
  double arr[2] = { forward, fail };
  pi_send_tag("forward_status");

  PI_SERIAL.
}

void setup() {
  // put your setup code here, to run once:

  DEBUG_SERIAL.begin(9600);
  PI_SERIAL.begin(9600);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
