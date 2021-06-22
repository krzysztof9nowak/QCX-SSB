#pragma once
#include <Arduino.h>
#include "config.h"
#include "Display.h"
#include "digital.h"


// Iambic Morse Code Keyer Sketch, Contribution by Uli, DL2DBG. Copyright (c) 2009 Steven T. Elliott Source: http://openqrp.org/?p=343,  Trimmed by Bill Bishop - wrb[at]wrbishop.com.  This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version. This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details: Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.

// keyerControl bit definitions
#define DIT_L 0x01    // Dit latch
#define DAH_L 0x02    // Dah latch
#define DIT_PROC 0x04 // Dit is being processed
#define PDLSWAP 0x08  // 0 for normal, 1 for swap
#define IAMBICB 0x10  // 0 for Iambic A, 1 for Iambic B
#define IAMBICA 0x00  // 0 for Iambic A, 1 for Iambic B
#define SINGLE 2      // Keyer Mode 0 1 -> Iambic2  2 ->SINGLE

int keyer_speed = 25;
static unsigned long ditTime; // No. milliseconds per dit
static uint8_t keyerControl;
static uint8_t keyerState;
static uint8_t keyer_mode = 2; //->  SINGLE
static uint8_t keyer_swap = 0; //->  DI/DAH

static uint32_t ktimer;
static int Key_state;
int debounce;

enum KSTYPE
{
  IDLE,
  CHK_DIT,
  CHK_DAH,
  KEYED_PREP,
  KEYED,
  INTER_ELEMENT
}; // State machine states

void update_PaddleLatch() // Latch dit and/or dah press, called by keyer routine
{
  if (_digitalRead(DIT) == LOW)
  {
    keyerControl |= keyer_swap ? DAH_L : DIT_L;
  }
  if (_digitalRead(DAH) == LOW)
  {
    keyerControl |= keyer_swap ? DIT_L : DAH_L;
  }
}

void loadWPM(int wpm) // Calculate new time constants based on wpm value
{
#if (F_MCU != 20000000)
  ditTime = (1200ULL * F_MCU / 16000000) / wpm; //ditTime = 1200/wpm;  compensated for F_CPU clock (running in a 16MHz Arduino environment)
#else
  ditTime = (1200 * 5 / 4) / wpm; //ditTime = 1200/wpm;  compensated for 20MHz clock (running in a 16MHz Arduino environment)
#endif
}




const char m2c[] PROGMEM = "~ ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";

#ifdef CW_MESSAGE
#define MENU_STR 1


#ifdef CW_MESSAGE_EXT
char cw_msg[6][48] = {"CQ PE1NNN +", "CQ CQ DE PE1NNN PE1NNN +", "GE TKS 5NN 5NN NAME IS GUIDO GUIDO HW?", "FB RPTR TX 5W 5W ANT INV V 73 CUAGN", "73 TU E E", "PE1NNN"};
#else
char cw_msg[1][48] = {"CQ PE1NNN +"};
#endif
uint8_t cw_msg_interval = 5; // number of seconds CW message is repeated
uint32_t cw_msg_event = 0;
uint8_t cw_msg_id = 0; // selected message

int cw_tx(char ch)
{ // Transmit message in CW
  char sym;
  for (uint8_t j = 0; (sym = pgm_read_byte_near(m2c + j)); j++)
  { // lookup msg[i] in m2c, skip if not found
    if (sym == ch)
    { // found -> transmit CW character j
      wdt_reset();
      uint8_t k = 0x80;
      for (; !(j & k); k >>= 1)
        ;
      k >>= 1; // shift start of cw code to MSB
      if (k == 0)
        delay(ditTime * 4); // space -> add word space
      else
      {
        for (; k; k >>= 1)
        {                 // send dit/dah one by one, until everythng is sent
          switch_rxtx(1); // key-on  tx
          if (delayWithKeySense(ditTime * ((j & k) ? 3 : 1)))
          {
            switch_rxtx(0);
            return 1;
          }               // symbol: dah or dih length
          switch_rxtx(0); // key-off tx
          if (delayWithKeySense(ditTime))
            return 1; // add symbol space
        }
        if (delayWithKeySense(ditTime * 2))
          return 1; // add letter space
      }
      break; // next character
    }
  }
  return 0;
}

int cw_tx(char *msg)
{
  for (uint8_t i = 0; msg[i]; i++)
  { // loop over message
    lcd.setCursor(0, 0);
    lcd.print(i);
    lcd.print("    ");
    if (cw_tx(msg[i]))
      return 1;
  }
  return 0;
}
#endif // CW_MESSAGE

#ifdef CW_DECODER
volatile uint8_t cwdec = 1;
static int32_t avg = 256;
static uint8_t sym;
static uint32_t amp32 = 0;
volatile uint32_t _amp32 = 0;
static char out[] = "                ";
volatile uint8_t cw_event = false;

void printsym(bool submit = true)
{
  if (sym < 128)
  {
    char ch = pgm_read_byte_near(m2c + sym);
    if (ch != '*')
    {
#ifdef CW_INTERMEDIATE
      out[15] = ch;
      cw_event = true;
      if (submit)
      {
        for (int i = 0; i != 15; i++)
        {
          out[i] = out[i + 1];
        }
        out[15] = ' ';
      } // update LCD, only shift when submit is true, otherwise update last char only
#else
      for (int i = 0; i != 15; i++)
        out[i] = out[i + 1];
      out[15] = ch;
      cw_event = true; // update LCD
#endif
    }
  }
  if (submit)
    sym = 1;
}

bool realstate = LOW;
bool realstatebefore = LOW;
bool filteredstate = LOW;
bool filteredstatebefore = LOW;
uint8_t nbtime = 16; // 6 // ms noise blanker
uint32_t starttimehigh;
uint32_t highduration;
uint32_t hightimesavg;
uint32_t lowtimesavg;
uint32_t startttimelow;
uint32_t lowduration;
uint32_t laststarttime = 0;
uint8_t wpm = 25;

inline void cw_decode()
{
  int32_t in = _amp32;
  EA(avg, in, (1 << 8));
  realstate = (in > (avg * 1 / 2)); // threshold

  // here we clean up the state with a noise blanker
  if (realstate != realstatebefore)
  {
    laststarttime = millis();
  }
//#define NB_SCALED_TO_WPM    1   // Scales noise-blanker timing the actual CW speed; this should reduce errors from noise at low speeds; this may have side-effect with fast speed changes that fast CW will be filtered out
#ifdef NB_SCALED_TO_WPM
  if ((millis() - laststarttime) > min(1200 / (20 * 2), max(1200 / (40 * 2), hightimesavg / 6)))
  {
#else
  if ((millis() - laststarttime) > nbtime)
  {
#endif
    if (realstate != filteredstate)
    {
      filteredstate = realstate;
      //dec2();
    }
  }
  else
    avg += avg / 100; // keep threshold above noise spikes (increase threshold with 1%)

  dec2();
  realstatebefore = realstate;
}

//#define NEW_CW  1   // CW decoder portions from by Hjalmar Skovholm Hansen OZ1JHM, source: http://www.skovholm.com/decoder11.ino
#ifdef NEW_CW
void dec2()
{
  // Then we do want to have some durations on high and low
  if (filteredstate != filteredstatebefore)
  {
    if (menumode == 0)
    {
      lcd.noCursor();
      lcd.setCursor(15, 1);
      lcd.print((filteredstate) ? 'R' : ' ');
      stepsize_showcursor();
    }

    if (filteredstate == HIGH)
    {
      starttimehigh = millis();
      lowduration = (millis() - startttimelow);
    }

    if (filteredstate == LOW)
    {
      startttimelow = millis();
      highduration = (millis() - starttimehigh);
      if (highduration < (2 * hightimesavg) || hightimesavg == 0)
      {
        hightimesavg = (highduration + hightimesavg + hightimesavg) / 3; // now we know avg dit time ( rolling 3 avg)
      }
      if (highduration > (5 * hightimesavg))
      {
        hightimesavg = highduration / 3; // if speed decrease fast ..
        //hightimesavg = highduration+hightimesavg;     // if speed decrease fast ..
      }
    }
  }

  // now we will check which kind of baud we have - dit or dah, and what kind of pause we do have 1 - 3 or 7 pause, we think that hightimeavg = 1 bit
  if (filteredstate != filteredstatebefore)
  {
    if (filteredstate == LOW)
    { //// we did end a HIGH
#define FAIR_WEIGHTING 1
#ifdef FAIR_WEIGHTING
      if (highduration < (hightimesavg + hightimesavg / 2) && highduration > (hightimesavg * 6 / 10))
      { /// 0.6 filter out false dits
#else
      if (highduration < (hightimesavg * 2) && highduration > (hightimesavg * 6 / 10))
      { /// 0.6 filter out false dits
#endif
        sym = (sym << 1) | (0); // insert dit (0)
      }
#ifdef FAIR_WEIGHTING
      if (highduration > (hightimesavg + hightimesavg / 2) && highduration < (hightimesavg * 6))
      {
#else
      if (highduration > (hightimesavg * 2) && highduration < (hightimesavg * 6))
      {
#endif
        sym = (sym << 1) | (1); // insert dah (1)
        wpm = (wpm + (1200 / ((highduration) / 3) * 4 / 3)) / 2;
      }
    }

    if (filteredstate == HIGH)
    { // we did end a LOW
      uint16_t lacktime = 10;
      if (wpm > 25)
        lacktime = 10; // when high speeds we have to have a little more pause before new letter or new word
      if (wpm > 30)
        lacktime = 12;
      if (wpm > 35)
        lacktime = 15;

      if (lowduration > (hightimesavg * (lacktime * 7 / 80)) && lowduration < hightimesavg * (lacktime * 5 / 10))
      { // letter space
        //if(lowduration > (hightimesavg*(2*lacktime/10)) && lowduration < hightimesavg*(5*lacktime/10)){ // letter space
        printsym();
      }
      if (lowduration >= hightimesavg * (lacktime * 5 / 10))
      { // word space
        printsym();
        printsym(); // print space
      }
    }
  }

  // write if no more letters
  if ((millis() - startttimelow) > (highduration * 6) && (sym > 1))
  {
    printsym();
  }

  filteredstatebefore = filteredstate;
}

#else // OLD_CW

void dec2()
{
  if (filteredstate != filteredstatebefore)
  { // then we do want to have some durations on high and low
    if (menumode == 0)
    {
      lcd.noCursor();
      lcd.setCursor(15, 1);
      lcd.print((filteredstate) ? 'R' : ' ');
      stepsize_showcursor();
    }

    if (filteredstate == HIGH)
    {
      starttimehigh = millis();
      lowduration = (millis() - startttimelow);
      //highduration = 0;

      if ((sym > 1) && lowduration > (hightimesavg * 2) /* && lowduration < hightimesavg*(5*lacktime)*/)
      { // letter space
        printsym();
        wpm = (1200 / hightimesavg * 4 / 3);
        //if(lowduration >= hightimesavg*(5)){ sym=1; printsym(); } // (print additional space) word space
      }
      if (lowduration >= hightimesavg * (5))
      {
        sym = 1;
        printsym();
      } // (print additional space) word space
    }

    if (filteredstate == LOW)
    {
      startttimelow = millis();
      highduration = (millis() - starttimehigh);
      //lowduration = 0;
      if (highduration < (2 * hightimesavg) || hightimesavg == 0)
      {
        hightimesavg = (highduration + hightimesavg + hightimesavg) / 3; // now we know avg dit time (rolling 3 avg)
      }
      if (highduration > (5 * hightimesavg))
      {
        hightimesavg = highduration / 3; // if speed decrease fast ..
        //hightimesavg = highduration+hightimesavg;     // if speed decrease fast ..
      }
      if (highduration > (hightimesavg / 2))
      {
        sym = (sym << 1) | (highduration > (hightimesavg * 2)); // dit (0) or dash (1)
#if defined(CW_INTERMEDIATE) && !defined(OLED) && (F_MCU >= 20000000)
        printsym(false);
#endif
      }
    }
  }

  if (((millis() - startttimelow) > hightimesavg * (6)) && (sym > 1))
  {
    //if(((millis() - startttimelow) > hightimesavg*(12)) && (sym > 1)){
    //if(sym == 2) sym = 1; else // skip E E E E E
    printsym(); // write if no more letters
    //sym=0; printsym(); // print special char
    //startttimelow = millis();
  }

  filteredstatebefore = filteredstate;
}
#endif //OLD_CW
#endif //CW_DECODER