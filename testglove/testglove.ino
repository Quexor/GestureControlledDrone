// ================================================================
// ===                      FLEX SENSORS                        ===
// ================================================================
#include "controller.h"
//#define A0 54
//#define A1 55
//#define A2 56
#define A0 23
#define A1 24
#define A2 25

Controller control(A0, A1, A2);


void setup() {
  Serial.begin(115200);
  Serial.println("I'm alive");
  control.init();
}

void loop() {
    //Serial.println(control.readFlex());
    Serial.print(control.readFilteredFlex(A0));
    Serial.print(",");
    Serial.print(control.readFilteredFlex(A1));
    Serial.print(",");
    Serial.println(control.readFilteredFlex(A2));
    /*
    Throttle = limit(control.readThrottle(), 1000, 2000); //limit(map(control.readFlex(A1), 0, 700, 1000, 2000), 1000, 2000); //TODO: Change map;
    Yaw = limit(control.readYaw(), 1000, 2000); //TODO: Change map;
    //Serial.print("Throttle: ");
    Serial.print(Throttle);
    Serial.print(",");
    //Serial.print(", Yaw: ");
    Serial.print(Yaw);
    */
}
