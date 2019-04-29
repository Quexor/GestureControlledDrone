#include "controller.h"
#define A0 54
#define A1 55
#define A2 56

Controller control(A0, A1, A2);

void setup() {
  Serial.begin(9600);
  control.init();
  Serial.println("I'm alive");
}

void loop() {
  Serial.print("Flex1 value 0 = ");
  Serial.println (control.readFlex(A0)); // Printing Sensor value on Serial monitor
  Serial.print("Flex1 value 1 = ");
  Serial.println (control.readFlex(A1)); // Printing Sensor value on Serial monitor
  delay(1000);
}
