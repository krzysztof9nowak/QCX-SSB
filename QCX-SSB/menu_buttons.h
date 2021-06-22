#pragma once

#include <Arduino.h>
#include "config.h"
#include "digital.h"
#include "adc.h"


static int32_t _step = 0;

  enum event_t
  {
    NONE = 0,
    BL = 0x10, // button left
    BR = 0x20, // button right
    BE = 0x30, // button encode
    SC = 0x01, // single click
    DC = 0x02, // double click
    PL = 0x04, // push long
    PLC = 0x05,
    PT = 0x0C // push and turn
  };

volatile event_t event;

//#define ONEBUTTON_INV 1 // Encoder button goes from PC3 to GND (instead PC3 to 5V, with 10k pull down)
#ifdef ONEBUTTON_INV
uint8_t inv = 1;
#else
uint8_t inv = 0;
#endif


event_t getEvent(event_t event){
    if (inv ^ _digitalRead(BUTTONS))
  { // Left-/Right-/Rotary-button (while not already pressed)
    if (!((event & PL) || (event & PLC)))
    { // hack: if there was long-push before, then fast forward
      uint16_t v = analogSafeRead(BUTTONS);
#ifdef CAT_EXT
      if (cat_key)
      {
        v = (cat_key & 0x04) ? 512 : (cat_key & 0x01) ? 870
                                 : (cat_key & 0x02)   ? 1024
                                                      : 0;
      } // override analog value exercised by BUTTONS press
#endif  //CAT_EXT
      event = SC;
      int32_t t0 = millis();
      while (inv ^ _digitalRead(BUTTONS))
      { // until released or long-press
        if ((millis() - t0) > 300)
        {
          event = PL;
          break;
        }
        wdt_reset();
      }
      delay(10); //debounce
      while((event != PL) && ((millis() - t0) < 500))
      { // until 2nd press or timeout
        if (inv ^ _digitalRead(BUTTONS))
        {
          event = DC;
          break;
        }
        wdt_reset();
      }
      while(inv ^ _digitalRead(BUTTONS))
      { // until released, or encoder is turned while longpress
        if (encoder_val && event == PL)
        {
          event = PT;
          break;
        }
#ifdef ONEBUTTON
        if (event == PL)
          break; // do not lock on longpress, so that L and R buttons can be used for tuning
#endif
        wdt_reset();
      } // Max. voltages at ADC3 for buttons L,R,E: 3.76V;4.55V;5V, thresholds are in center
      event |= (v < (4.2 * 1024.0 / 5.0)) ? BL : (v < (4.8 * 1024.0 / 5.0)) ? BR
                                                                            : BE; // determine which button pressed based on threshold levels
    }
    else
    {                                                             // hack: fast forward handling
      event = (event & 0xf0) | ((encoder_val) ? PT : PLC /*PL*/); // only alternate between push-long/turn when applicable
    }
   }
  else
    event = NONE; // no button pressed: reset event

    return event;
}

