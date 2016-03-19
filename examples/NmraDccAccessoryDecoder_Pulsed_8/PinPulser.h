#include <Arduino.h>

#define PIN_PULSER_MAX_PINS    16

enum PP_State
{
  PP_IDLE = 0,
  PP_OUTPUT_ON_DELAY,
  PP_CDU_RECHARGE_DELAY,
};

class PinPulser
{
  private:
    uint16_t      onMs;
    uint16_t      cduRechargeMs;
    PP_State      state = PP_IDLE;
    unsigned long targetMs = 0;
    uint8_t       activeOutputState = HIGH;
    uint8_t       pinQueue[PIN_PULSER_MAX_PINS + 1];

  public:
    void init(uint16_t onMs, uint16_t cduRechargeMs, uint8_t activeOutputState);
    uint8_t addPin(uint8_t pin);
    PP_State process(void);
};

