#include "Adafruit_TCS34725.h" 



Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

void setup(){
 
 Wire.begin();  
 Serial.begin(9600); 
 if (!tcs.begin())
  {
    Serial.println("error first!");
  } 
 
}

void print_raw_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
  //log_print("raw color:");
  Serial.print("R: ");
  Serial.print(r);
  Serial.print("\t G: ");
  Serial.print(g);
  Serial.print("\t B: ");
  Serial.print(b);
  Serial.print("\t C: ");
  Serial.println(c);
}

bool black_detect(){
 
  uint16_t r,g,b,c;
  tcs.getRawData(&r,&g, &b, &c);
  //print_raw_color(r,g,b,c);
  if(c < 250) { 
    Serial.println("Black square detected"); 
    return true;
  }
  return false;
}  
bool blue_detect(){ 
  uint16_t r,g,b,c; 
  tcs.getRawData(&r, &g, &b, &c);
  if(b > (r * 2.5)){ 
    Serial.println("Blue");  
    return(true); 
  } 
  
  return false; 
  
}
bool silver_detect(){ 
    uint16_t r, g, b, c = 0;  
    int silver_persistance; 
    //print_raw_color(r1,g1,b1,c1);
    tcs.getRawData(&r, &g, &b, &c);
   
    //print_raw_color(r2, g2, b2, c2);
    //Serial.print("qtr[4]: ");
    //Serial.println(qtr[4]);
    //print_raw_color(r2,g2,b2,c2);
  //  Serial.print("r/g:");
   // Serial.println((r2 / (float)g2) * 10);
    //Serial.print(" c:");
    //Serial.print(c2);
   
  if (c >=  950 && (r / (float)g) * 10 >= 10.5) {

   //   silver_persistance++;
      Serial.println("Checkpoint detected"); 
      return true; 
    
  } 
  else  
      return false; 
    /*
    else {
      silver_persistance = 0;
    }
    if (silver_persistance >= 2) {
      utils::stopMotors();
      Serial.println("Checkpoint detected"); 
      return;
    }   
    */
}  
char* returnColor(){
    uint16_t r, g, b, c = 0;  
    tcs.getRawData(&r, &g, &b, &c); 
    if (c >=  950 && (r / (float)g) * 10 >= 10.5) {

   //   silver_persistance++;
      Serial.println("silver detected"); 
      return "silver"; 
    }
    else if(c < 250){ 
      Serial.println("black detected"); 
      return "black"; 
    }
    else if(b > (r * 2.5)){ 
      Serial.println("blue detected"); 
      return "blue"; 
    }
    else 
      return " "; 
  
}
void loop(){
  ///Serial.println("Starting"); 
  returnColor(); 
  
  
  
} 
