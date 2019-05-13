#include "controller.h"
#include "MPU6050_DMP.h"
/*
#define A0 54
#define A1 55
#define A2 56
*/
#define A0 14
#define A1 15
#define A2 16

Controller control(A0, A1, A2);

void setup() {
  Serial.begin(38400);
  control.init();
  Serial.println("I'm alive");
}

void loop() {
  //value0
  Serial.println(control.readFlex()); // Printing Sensor value on Serial monitor
  delay(100);
}
