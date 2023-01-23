#include <Servo.h>
//#define RESET
Servo myservo; 
int16_t servopin = A6; 
const int num_per_column = 4;
const int num_columns = 3;
const int total = num_per_column * num_columns;

void kitDrop(int num) { 
  static int columnNum = 1; 
  static int numDropped = 0; 

  for (int i = 0; i < num; i++) { 
    numDropped++; 
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
  }
}

void setup() {
  Serial.begin(9600);
  myservo.attach(servopin);  // attaches the servo on servopin 
}

void loop() {

  #ifdef RESET
  myservo.write(180); 
  #endif 
   
  #ifndef RESET
  kitDrop(2); 
  delay(1000); 
  kitDrop(2); 
  delay(1000);
  #endif
}
