#include "Motors.h"
#include "utils.h" 
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
static char L = 'L';
static char R = 'R';

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
void right(int angle, int speed)
{
  float orientation = 0;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x + angle);
  orientation = orientationData.orientation.x > angle + (goal - 360) ? orientationData.orientation.x - 360 : orientationData.orientation.x;

  if (goal >= 360)
  {

    goal -= 360;
    while (orientation < goal)
    {
       float p = abs((orientation - angle) / angle);
    
     
     Serial.println(orientation);  
     // Serial.println(goal); 
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x > angle + goal ? orientationData.orientation.x - 360 : orientationData.orientation.x;
      // Serial.println(orient);
      utils::forward((p * -speed) - 70,(p * speed) + 70); 
    }
  }

  else
  {
    while ((int)orientationData.orientation.x < goal)
    {
       float p = abs((orientationData.orientation.x - angle) / angle);
      Serial.println(p); 
      // Serial.println(orientationData.orientation.x);  
    //  Serial.println(goal);
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      utils::forward((p * -speed) - 70,(p * speed) + 70); 
    }
  }
  utils::stopMotors();
}
void left(int angle, int speed)
{ 
  float orientation = 0;
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  int goal = (int)(orientationData.orientation.x - angle);
  
  orientation = orientationData.orientation.x < goal + 360 - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;
   
  if (goal < 0)
  {

    goal += 360;
    Serial.println(orientationData.orientation.x); 
    Serial.println(goal); 
    while (orientation > goal)
    { 
       float p = abs((orientation - (360 - angle)) / angle);
      Serial.println(orientationData.orientation.x);  
      Serial.println(p); 
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      orientation = orientationData.orientation.x < goal - angle ? orientationData.orientation.x + 360 : orientationData.orientation.x;
      utils::forward((p * speed) + 70, (p * -speed) - 70); 
     
    }
  }

  else
  {

    while ((int)orientationData.orientation.x > goal)
    { 
      float p = abs((orientation - (360 - angle)) / angle);
      Serial.println(orientationData.orientation.x); 
      Serial.println(p); 
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      utils::forward((p * speed) + 70, (p * -speed) - 70); 
   
    }
  }
  utils::stopMotors();
}
void turn(int angle, int speed, char direction){ 
if(direction == 'R')
  right(angle, speed);  

else if(direction == 'L') 
  left(angle, speed);  
else 
  Serial.println("bro what?"); 

}
void loop() {
  // put your main code here, to run repeatedly:
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  turn(90, 80, R); 
  delay(1000);  
  turn(179, 80, L);  
  delay(1000); 
}
