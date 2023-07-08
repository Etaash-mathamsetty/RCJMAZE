#include "Arduino.h"
#include <FastLED.h>
#include <Wire.h>

// How many leds in your strip?
#define NUM_LEDS 1
#define DATA_PIN 2

// Define the array of leds

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Serial.write();
  btStart();
}
