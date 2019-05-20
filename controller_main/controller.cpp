#include <Arduino.h>
#include "controller.h"

Controller::Controller(uint16_t A0, uint16_t A1, uint16_t A2) {
  this->pin0 = A0;
  this->pin1 = A1;
  this->pin2 = A2;
}

void Controller::init() {
  pinMode(this->pin0, INPUT);
  pinMode(this->pin1, INPUT);
  pinMode(this->pin2, INPUT);

  //Read flex sensor values & Get average sensor values
  uint16_t temp0 = 0; uint16_t temp1 = 0; uint16_t temp2 = 0;
  for(int i = 0; i < 15; i++) {
    temp0 += analogRead(this->pin0); 
    temp1 += analogRead(this->pin1); 
    temp2 += analogRead(this->pin2); 
  }
  
  this->initialA0 = temp0/15;
  this->initialA1 = temp1/15;
  this->initialA2 = temp2/15;
} 

int Controller::readFlex(uint16_t pin) {
  return analogRead(pin);
}

String Controller::readFlex() {
  String comma_seperated = String(analogRead(this->pin0))+"," +String(analogRead(this->pin1))+","+String(analogRead(this->pin2));  
  return comma_seperated;
}

int Controller::readThrottle() {
  int t = 1000;
  int t_diff = (this->initialA0 - this->pin0);
  if (t_diff > 5) {
    t = map(t_diff, 0, 100, 1000, 2000);
  }
  return t;
}

int Controller::readYaw() {
  int y = 1500;
  int y_diff1 = (this->initialA1 - this->pin1);
  int y_diff2 = (this->initialA2 - this->pin2);

  // > 1500 turns left, < 1500 turns right
  int y_diff = y_diff1 -  y_diff2;
  if (abs(y_diff) > 5) {
    y = map(y_diff, -100, 100, 1000, 2000);  
  }
  return y;
}
