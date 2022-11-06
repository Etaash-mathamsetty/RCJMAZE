#include "Motors.h"
#include "utils.h" 
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

sensors_event_t orientationData;


Motor motor1(MPORT1);
Motor motor2(MPORT2);


void setup() {
  // put your setup code here, to run once:
  utils::setMotors(&motor1, &motor2);
  Serial.begin(9600); 
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  Serial.println("Success"); 
}

void left(int angle, int speed)
{
  float orientation = 0;
 // angle -= subtract_ang;
  
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x - angle);
  orientation = orientationData.orientation.x < goal + 360 - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;
  //Serial.println("what is goinbg on"); 
  if (goal < 0)
  {

    goal += 360;

    while (orientation > goal)
    { 
       float p = abs((orientation - (360 - angle)) / angle);
      //Serial.println(orientationData.orientation.x);  
      Serial.println(p); 
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x < goal - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;
      utils::forward((p * speed) + 50, (p * -speed) - 50); 
      //motor2.run(-speed);
      //motor1.run(-speed);
    }
  }

  else
  {

    while ((int)orientationData.orientation.x > goal)
    { 
      float p = abs((orientation - (360 - angle)) / angle);
      //Serial.println(orientationData.orientation.x); 
      Serial.println(p); 
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      utils::forward((p * speed) + 50, (p * -speed) - 50); 
   //   motor2.run(-speed);
     // motor1.run(-speed);
    }
  }
  utils::stopMotors();
} 
//testing 
void loop() {
  // put your main code here, to run repeatedly:
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  left(90, 80);    
  delay(1000); 
}
