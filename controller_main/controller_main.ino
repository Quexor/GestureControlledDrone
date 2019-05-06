#include "controller.h"
#define A0 54
#define A1 55
#define A2 56

Controller control(A0, A1, A2);

void setup() {
  Serial.begin(38400);
  control.init();
  Serial.println("I'm alive");
}

void loop() {
  //value0
  Serial.print(control.readFlex(A0)); // Printing Sensor value on Serial monitor
  Serial.print(",");
  //value1
  Serial.print(control.readFlex(A1)); // Printing Sensor value on Serial monitor
  Serial.print(",");
  //value2
  Serial.println (control.readFlex(A2)); // Printing Sensor value on Serial monitor
  delay(100);
}
