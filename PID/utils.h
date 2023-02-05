

#include "Motors.h"
#include <Servo.h>

//#define MUXADDR 0x70

namespace utils{

Motor* motor;
Motor* motor2;
Servo myservo; 
int16_t servopin = A6; 
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

void kitDrop(int num) { 
  static int columnNum = 1; 
  static int numDropped = 0; 

  for (int i = 0; i < num; i++) {  
    if (numDropped && !(numDropped % num_per_column))
      columnNum++; 
    
    if (numDropped > total) {
      myservo.write(0);
      return;
    }

    myservo.write(180 - 60*columnNum - 3); 
    Serial.print("columnNum");
    Serial.println(columnNum);
    Serial.println("numDropped");
    Serial.println(numDropped);
    delay(1000); 
    myservo.write(180); 
    delay(1000); 
    numDropped++;
  }
}

void resetServo() {
  myservo.write(180);
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
    num %= mod;
    if (num < 0) {
      num += mod;
    }
    return num;
  }
}

};
