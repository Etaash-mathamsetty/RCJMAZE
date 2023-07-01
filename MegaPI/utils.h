#ifndef _UTILS_H_
#define _UTILS_H_

#include "Motors.h"
#include <Servo.h>

//#define MUXADDR 0x70

namespace utils{

Motor* motor;
Motor* motor2;
Servo myservo; 
Servo myservo2; 
int16_t servopin = A6; 
int16_t servopin2 = A7;
const int num_per_column = 4;
const int num_columns = 3;
const int total = num_per_column * num_columns;

void setMotors(Motor* _motor1, Motor* _motor2){
	motor = _motor1;
	motor2 = _motor2;
}

void forward(int rightSpeed, int leftSpeed) {
  motor->run(rightSpeed);
  motor2->run(leftSpeed);
}
	
void forward(int speed){
  forward(speed,speed);	
}

void resetTicks(){
	motor->resetTicks();
	motor2->resetTicks();
}

void stopMotors(){
	motor->stop();
	motor2->stop();
}

void forwardTicks(int speed, int ticks, bool reset = true){
  if(reset)
  resetTicks();
  while(abs(motor->getTicks()) <= abs(ticks))
  {
    forward(speed);
  }
  stopMotors();
}

void addBoost(int boost)
{
  motor->addBoost(boost);
  motor2->addBoost(boost);
}

void resetBoost()
{
  addBoost(0);
}

void resetServo() {
  myservo.write(175);
  delay(200);
  myservo2.write(25); 
  delay(200);
}

namespace logger{
	
	void begin(){
		#ifndef LOGGER_DISABLE
		Serial.begin(9600);	
		#endif
	}
	
	void println(){
		#ifndef LOGGER_DISABLE
		Serial.println();
		#endif
		
	}
	
	#if __cplusplus >= 201703L
	//template <typename T>
	void println(auto thing){
		#ifndef LOGGER_DISABLE
		Serial.println(thing);
		#endif
	}
	
	//template <typename T>
	void print(auto thing){
		#ifndef LOGGER_DISABLE
		Serial.print(thing);	
		#endif
	}
	#endif
};

namespace math {
  int wrapAround(int num, int mod) {
    if (num < 0) {
      num += mod;
    }
    num %= mod;

    return num;
  }
}

};
#endif