#include <Arduino.h>
#include "Filters.h"

#define FILTER_WIN_SIZE 64

class Controller {
  public:
    Controller(uint16_t A0, uint16_t A1, uint16_t A2);
    void init();
    int readFlex(uint16_t pin);
    String readFlex();
    int readThrottle();
    int readYaw();
    int readFilteredFlex(uint16_t pin);
    long limit(long x, long low_val, long max_val);

  private:
    uint16_t initialA0;
    uint16_t initialA1;
    uint16_t initialA2;
    uint16_t pin0;
    uint16_t pin1;
    uint16_t pin2;
    uint32_t A0vals[FILTER_WIN_SIZE];
    uint32_t A1vals[FILTER_WIN_SIZE];
    uint32_t A2vals[FILTER_WIN_SIZE];
    uint16_t A0index;
    uint16_t A1index;
    uint16_t A2index;
    //uint8_
};
