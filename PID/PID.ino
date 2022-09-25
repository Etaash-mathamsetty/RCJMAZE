#include <Adafruit_BNO055.h>
#include "Motors.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t accelerometerData, gyroData;

#define PI_SERIAL Serial2

void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);
  bno.begin();
  
}

inline void pi_send_tag(const char* tag)
{
  PI_SERIAL.print(tag);
  PI_SERIAL.print("::");
}

inline void display_tag(const char* tag)
{
  Serial.print(tag);
  Serial.print(": ");
}

inline void display_data(const sensors_event_t& data)
{

  switch(data.type)
  {
    case SENSOR_TYPE_ACCELEROMETER:
    {
      display_tag("accel");
      Serial.print("X: ");
      Serial.print(data.acceleration.x);
      Serial.print(" Y: ");
      Serial.print(data.acceleration.y);
      Serial.print(" Z: ");
      Serial.println(data.acceleration.z);
      break;
    }
    case SENSOR_TYPE_GYROSCOPE:
    {
      display_tag("gyro");
      Serial.print("X: ");
      Serial.print(data.gyro.x);
      Serial.print(" Y: ");
      Serial.print(data.gyro.y);
      Serial.print(" Z: ");
      Serial.println(data.gyro.z);
      break;
    }
    default:
      break;
  }
}



inline void pi_send_data(const sensors_event_t& data)
{
  
  switch(data.type)
  {
    case SENSOR_TYPE_ACCELEROMETER:
    {
      pi_send_tag("accel");
      PI_SERIAL.print(data.acceleration.x);
      PI_SERIAL.print(",");
      PI_SERIAL.print(data.acceleration.y);
      PI_SERIAL.print(",");
      PI_SERIAL.println(data.acceleration.z);
    }
    case SENSOR_TYPE_GYROSCOPE:
    {
      pi_send_tag("gyro");
      PI_SERIAL.print(data.gyro.x);
      PI_SERIAL.print(",");
      PI_SERIAL.print(data.gyro.y);
      PI_SERIAL.print(",");
      PI_SERIAL.println(data.gyro.z);  
    }
    default:
      break;
  }

}

void loop() {
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);


  display_data(accelerometerData);
  pi_send_data(accelerometerData);
  display_data(gyroData);
  pi_send_data(gyroData);

  delay(100);
}
