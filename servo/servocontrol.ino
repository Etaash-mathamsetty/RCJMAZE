#include "MeOrion.h" 
Servo myservo; 
int16_t servopin = "A1"; 
static int columnNum; 
static int numDropped; 

void setup()
{
  myservo.attach(servopin);  // attaches the servo on servopin 
  columnNum = 1; 
  numDropped = 0; 
}
void kitDrop(int num){ 
  for(int i = 0; i < num; i++)
  { 
    numDropped++; 
    if(numDropped == 4 || numDropped == 8)
      columnNum++; 
    if(numDropped > 12){ 
      return; 
    }
    myservo.write(60*columnNum); 
    delay(100); 
    myservo.write(0); 
    delay(100); 


  }
  return; 

}

void loop()
{   
 kitDrop(2); 
 delay(1000); 
 kitDrop(2); 
 delay(1000);
 
}
