#include <Servo.h>
//#define RESET
Servo myservo; 
Servo myservo2; 
int16_t servopin = A1; 
int16_t servopin2 = A2;
const int num_per_column = 4;
const int num_columns = 3;
const int total = num_per_column * num_columns;
void kitDrop(int num, char side) { 
  static int columnNum = 1; 
  static int numDropped = 0; 
  const int offset_for_stack[3] = {10, 7, 0};

  analogWrite(5, 30);
  if(side == 'r'){
    myservo2.write(0); 
  } 
  else {
    myservo2.write(50); 
  }
  for (int i = 0; i < num; i++) {  
    if (numDropped && !(numDropped % num_per_column))
      columnNum++; 
    
    if (numDropped > total) {
      myservo.write(0);
      delay(200);
      analogWrite(5, 0);
      return;
    }

    myservo.write(180 - (60*columnNum + offset_for_stack[columnNum - 1])); 
    Serial.print("columnNum");
    Serial.println(columnNum);
    Serial.println("numDropped");
    Serial.println(numDropped);
    delay(1000); 
    myservo.write(175); 
    delay(1000); 
    numDropped++;
  }

  delay(1000);
  analogWrite(5, 0);
}


void resetServo() {
  myservo.write(180); 
  myservo2.write(0); 
}

void setup() {
  Serial.begin(9600);
  myservo.attach(servopin);  // attaches the servo on servopin  
  myservo2.attach(servopin2); 
}

void loop() {

  #ifdef RESET
  myservo.write(180); 
  myservo2.write(0); 
  #endif 
   
  #ifndef RESET
  kitDrop(2, 'r'); 
  delay(1000); 
  kitDrop(2, 'l'); 
  delay(1000);
  #endif
}
