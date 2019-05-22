// ================================================================
// ===                      FLEX SENSORS                        ===
// ================================================================
#include "controller.h"
#define A0 14 //#define A0 54
#define A1 15 //#define A1 55
#define A2 16 //#define A2 56

Controller control(A0, A1, A2);


void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println("I'm alive");
  control.init();
}

void loop() {
    Throttle = limit(control.readThrottle(), 1000, 2000); //limit(map(control.readFlex(A1), 0, 700, 1000, 2000), 1000, 2000); //TODO: Change map;
    Yaw = limit(control.readYaw(), 1000, 2000); //TODO: Change map;
    //Serial.print("Throttle: ");
    Serial.print(Throttle);
    Serial.print(",");
    //Serial.print(", Yaw: ");
    Serial.print(Yaw);
}
