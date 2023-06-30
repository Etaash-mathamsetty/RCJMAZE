

#include "Motors.h"

//#define MUXADDR 0x70

namespace utils{

Motor* motor;
Motor* motor2;

void setMotors(Motor* _motor1, Motor* _motor2){
	motor = _motor1;
	motor2 = _motor2;
}

void forward(int rightSpeed, int leftSpeed) {
  motor->run(-rightSpeed);
  motor2->run(leftSpeed);
}
	
void forward(int speed){
  forward(speed,speed);	
}


void resetTicks(){
	motor->resetTicks();
	motor2->resetTicks();
}

void forwardTicks(int speed, int ticks, bool reset = true){
  if(reset)
  resetTicks();
  if(speed > 0){
    while(motor2->getTicks() <= ticks){
      forward(speed);
    }
  }
  else{
    while(motor->getTicks() <= ticks){
      forward(speed);
    }
  }
}

void stopMotors(){
	motor->stop();
	motor2->stop();
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

};
