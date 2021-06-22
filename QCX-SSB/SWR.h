#include <Arduino.h>
#include "config.h"

#ifdef SWR_METER
void readSWR()
// reads FWD / REF values from A6 and A7 and computes SWR
// credit Duwayne, KV4QB
{
  float v_FWD = 0;
  float v_REF = 0;
  for (int i = 0; i <= 7; i++) {
    v_FWD = v_FWD + (ref_V / 1023) * (int) analogRead(vin_FWD);
    v_REF = v_REF + (ref_V / 1023) * (int) analogRead(vin_REF);
    delay(5);
  }
  v_FWD = v_FWD / 8;
  v_REF = v_REF / 8;

  float p_FWD = sq(v_FWD);
  float p_REV = sq(v_REF);

  float vRatio = v_REF / v_FWD;
  float VSWR = (1 + vRatio) / (1 - vRatio);

  if ((VSWR > 9.99) || (VSWR < 1) )VSWR = 9.99;

  if (p_FWD != FWD || VSWR != SWR) {
      lcd.noCursor();
      lcd.setCursor(0,0);
      switch(swrmeter) {
        case 1:
          lcd.print(" "); lcd.print(floor(100*p_FWD)/100); lcd.print("W  SWR:"); lcd.print(floor(100*VSWR)/100);
          break;
        case 2:
          lcd.print(" F:"); lcd.print(floor(100*p_FWD)/100); lcd.print("W R:"); lcd.print(floor(100*p_REV)/100); lcd.print("W");
          break;
        case 3:
          lcd.print(" F:"); lcd.print(floor(100*v_FWD)/100); lcd.print("V R:"); lcd.print(floor(100*v_REF)/100); lcd.print("V");
          break;
      }
    FWD = p_FWD;
    SWR = VSWR;
  }
}

volatile uint8_t swrmeter = 1;
#endif