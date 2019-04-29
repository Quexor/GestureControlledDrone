#include <Arduino.h>

class Controller {
  public:
    Controller(uint16_t A0, uint16_t A1, uint16_t A2);
    void init();
    int readFlex(uint16_t pin);
    
  private:
    uint16_t initialA0;
    uint16_t initialA1;
    uint16_t initialA2;
    uint16_t pin0;
    uint16_t pin1;
    uint16_t pin2;
};
