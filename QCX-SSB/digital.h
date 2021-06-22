#pragma once
#include <Arduino.h>
#include "config.h"


void initPins(){
  // initialize
  digitalWrite(SIG_OUT, LOW);
  digitalWrite(RX, HIGH);
  digitalWrite(KEY_OUT, LOW);
  digitalWrite(SIDETONE, LOW);

  // pins
  pinMode(SIDETONE, OUTPUT);
  pinMode(SIG_OUT, OUTPUT);
  pinMode(RX, OUTPUT);
  pinMode(KEY_OUT, OUTPUT);
#ifdef ONEBUTTON
  pinMode(BUTTONS, INPUT_PULLUP);  // rotary button
#else
  pinMode(BUTTONS, INPUT);  // L/R/rotary button
#endif
  pinMode(DIT, INPUT_PULLUP);
  pinMode(DAH, INPUT);  // pull-up DAH 10k via AVCC
  //pinMode(DAH, INPUT_PULLUP); // Could this replace D4? But leaks noisy VCC into mic input!

  digitalWrite(AUDIO1, LOW);  // when used as output, help can mute RX leakage into AREF
  digitalWrite(AUDIO2, LOW);
  pinMode(AUDIO1, INPUT);
  pinMode(AUDIO2, INPUT);

#ifdef NTX
  digitalWrite(NTX, HIGH);
  pinMode(NTX, OUTPUT);
#endif //NTX
#ifdef PTX
  digitalWrite(PTX, LOW);
  pinMode(PTX, OUTPUT);
#endif //PTX
#ifdef SWR_METER
  pinMode(vin_FWD, INPUT);
  pinMode(vin_REF, INPUT);
#endif
#ifdef OLED  // assign unused LCD pins
  pinMode(PD4, OUTPUT);
  pinMode(PD5, OUTPUT);
#endif
}

#ifdef CAT_EXT
volatile uint8_t cat_key = 0;
uint8_t _digitalRead(uint8_t pin)
{                // reads pin or (via CAT) artificially overriden pins
  serialEvent(); // allows CAT update
  if (cat_key)
  {
    return (pin == BUTTONS) ? ((cat_key & 0x07) > 0) : (pin == DIT) ? ~cat_key & 0x10
                                                   : (pin == DAH)   ? ~cat_key & 0x20
                                                                    : 0;
  } // overrides digitalRead(DIT, DAH, BUTTONS);
  return digitalRead(pin);
}
#else
#define _digitalRead(x) digitalRead(x)
#endif //CAT_EXT


uint8_t delayWithKeySense(uint32_t ms)
{
  uint32_t event = millis() + ms;
  for (; millis() < event;)
  {
    wdt_reset();
    if (inv ^ digitalRead(BUTTONS) || !digitalRead(DAH) || !digitalRead(DIT))
    {
      for (; inv ^ digitalRead(BUTTONS);)
        wdt_reset(); // wait until buttons released
      return 1;      // stop when button/key pressed
    }
  }
  return 0;
}