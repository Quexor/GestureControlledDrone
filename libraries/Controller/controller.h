#include <Arduino.h>
#include "Filters.h"

class Controller {
  public:
    Controller(uint16_t A0, uint16_t A1, uint16_t A2);
    void init();
    int readFlex(uint16_t pin);
    String readFlex();
    int readThrottle();
    int readYaw();
    
  private:
    uint16_t initialA0;
    uint16_t initialA1;
    uint16_t initialA2;
    uint16_t pin0;
    uint16_t pin1;
    uint16_t pin2;
    //uint16_t A0vals[16];
    //uint16_t A1vals[16];
    //uint16_t A2vals[16];
    //uint8_t A0index;
    //uint8_
};
