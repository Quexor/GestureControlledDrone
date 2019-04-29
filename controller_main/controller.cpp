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

  //Read sensor values & Get average sensor values
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
