//  QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define VERSION "1.02t"

#include <Arduino.h>
#include "config.h"
#include "Display.h"
#include "Si5351.h"
#include "custom_math.h"
#include "adc.h"
#include "timers.h"
#include "settings.h"
#include "encoder.h"
#include "digital.h"
#include "menu.h"
#include "tx.h"
#include "rx.h"
#include "menu_buttons.h"

// #if(ARDUINO < 10810)
//    #error "Unsupported Arduino IDE version, use Arduino IDE 1.8.10 or later from https://www.arduino.cc/en/software"
// #endif

extern char __bss_end;
static int freeMemory()
{
  char *sp = reinterpret_cast<char *>(SP);
  return sp - &__bss_end;
} // see: http://www.nongnu.org/avr-libc/user-manual/malloc.html



static uint8_t practice = false; // Practice mode

#ifdef DEBUG
static uint32_t sr = 0;
static uint32_t cpu_load = 0;
volatile uint16_t param_a = 0; // registers for debugging, testing and experimental purposes
volatile int16_t param_b = 0;
volatile int16_t param_c = 0;
#endif

enum dsp_cap_t
{
  ANALOG,
  DSP,
  SDR
};
#ifdef QCX
const uint8_t dsp_cap = 0;
const uint8_t ssb_cap = 0;
#else
// force SSB and SDR capability
const uint8_t ssb_cap = 1;
const uint8_t dsp_cap = SDR;
#endif

enum mode_t
{
  LSB,
  USB,
  CW,
  FM,
  AM
};
volatile uint8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile uint8_t menumode = 0; // 0=not in menu, 1=selects menu item, 2=selects parameter value

#define F_SAMP_PWM (78125 / 1)
//#define F_SAMP_RX 78125  // overrun, do not use
#define F_SAMP_RX 62500
//#define F_SAMP_RX 52083
//#define F_SAMP_RX 44643
//#define F_SAMP_RX 39062
//#define F_SAMP_RX 34722
//#define F_SAMP_RX 31250
//#define F_SAMP_RX 28409
#define F_ADC_CONV (192307 / 2) //was 192307/1, but as noted this produces clicks in audio stream. Slower ADC clock cures this (but is a problem for VOX when sampling mic-input simulatanously).

#ifdef FAST_AGC
volatile uint8_t agc = 2;
#else
volatile uint8_t agc = 1;
#endif
volatile uint8_t nr = 0;
volatile uint8_t att = 0;
volatile uint8_t att2 = 2; // Minimum att2 increased, to prevent numeric overflow on strong signals
volatile uint8_t _init = 0;

// Old AGC algorithm which only increases gain, but does not decrease it for very strong signals.
// Maximum possible gain is x32 (in practice, x31) so AGC range is x1 to x31 = 30dB approx.
// Decay time is fine (about 1s) but attack time is much slower than I like.
// For weak/medium signals it aims to keep the sample value between 1024 and 2048.
static int16_t gain = 1024;
inline int16_t process_agc_fast(int16_t in)
{
  int16_t out = (gain >= 1024) ? (gain >> 10) * in : in;
  int16_t accum = (1 - abs(out >> 10));
  if ((INT16_MAX - gain) > accum)
    gain = gain + accum;
  if (gain < 1)
    gain = 1;
  return out;
}

// Contribution by Alan, M0PUB: Experimental new AGC algorithm.
// ASSUMES: Input sample values are constrained to a maximum of +/-4096 to avoid integer overflow in earlier
// calculations.
//
// This algorithm aims to keep signals between a peak sample value of 1024 - 1536, with fast attack but slow
// decay.
//
// The variable centiGain actually represents the applied gain x 128 - i.e. the numeric gain applied is centiGain/128
//
// Since the largest valid input sample has a value of +/- 4096, centiGain should never be less than 32 (i.e.
// a 'gain' of 0.25). The maximum value for centiGain is 32767, and hence a gain of 255. So the AGC range
// is 0.25:255, or approx. 60dB.
//
// Variable 'slowdown' allows the decay time to be slowed down so that it is not directly related to the value
// of centiCount.

static int16_t centiGain = 128;
#define DECAY_FACTOR 400 // AGC decay occurs <DECAY_FACTOR> slower than attack.
static uint16_t decayCount = DECAY_FACTOR;
#define HI(x) ((x) >> 8)
#define LO(x) ((x)&0xFF)

inline int16_t process_agc(int16_t in)
{
  static bool small = true;
  int16_t out;

  if (centiGain >= 128)
    out = (centiGain >> 5) * in; // net gain >= 1
  else
    out = (centiGain >> 2) * (in >> 3); // net gain < 1
  out >>= 2;

  if (HI(abs(out)) > HI(1536))
  {
    centiGain -= (centiGain >> 4); // Fast attack time when big signal encountered (relies on CentiGain >= 16)
  }
  else
  {
    if (HI(abs(out)) > HI(1024))
      small = false;
    if (--decayCount == 0)
    { // But slow ramp up of gain when signal disappears
      if (small)
      { // 400 samples below lower threshold - increase gain
        if (centiGain < (INT16_MAX - (INT16_MAX >> 4)))
          centiGain += (centiGain >> 4);
        else
          centiGain = INT16_MAX;
      }
      decayCount = DECAY_FACTOR;
      small = true;
    }
  }
  return out;
}

inline int16_t process_nr_old(int16_t ac)
{
  ac = ac >> (6 - abs(ac)); // non-linear below amp of 6; to reduce noise (switchoff agc and tune-up volume until noise dissapears, todo:extra volume control needed)
  ac = ac << 3;
  return ac;
}

inline int16_t process_nr_old2(int16_t ac)
{
  static int16_t ea1;
  //ea1 = MLEA(ea1, ac, 5, 6); // alpha=0.0156
  ea1 = EA(ea1, ac, 64); // alpha=1/64=0.0156
  //static int16_t ea2;
  //ea2 = EA(ea2, ea1, 64); // alpha=1/64=0.0156

  return ea1;
}

inline int16_t process_nr(int16_t in)
{
  static int16_t ea1;
  ea1 = EA(ea1, in, 1 << (nr - 1));

  return ea1;
}

#define N_FILT 7
//volatile uint8_t filt = 0;
uint8_t prev_filt[] = {0, 4}; // default filter for modes resp. CW, SSB

/* basicdsp filter simulation:
  samplerate=7812
  za0=in
  p1=slider1*10
  p2=slider2*10
  p3=slider3*10
  p4=slider4*10
  zb0=(za0+2*za1+za2)/2-(p1*zb1+p2*zb2)/16
  zc0=(zb0+2*zb1+zb2)/4-(p3*zc1+p4*zc2)/16
  zc2=zc1
  zc1=zc0
  zb2=zb1
  zb1=zb0
  za2=za1
  za1=za0
  out=zc0

  samplerate=7812
  za0=in
  p1=slider1*100+100
  p2=slider2*100
  p3=slider3*100+100
  p4=slider4*100
  zb0=(za0+2*za1+za2)-(-p1*zb1+p2*zb2)/64
  zc0=(zb0-2*zb1+zb2)/8-(-p3*zc1+p4*zc2)/64
  zc2=zc1
  zc1=zc0
  zb2=zb1
  zb1=zb0
  za2=za1
  za1=za0
  out=zc0/8
*/
inline int16_t filt_var(int16_t za0) //filters build with www.micromodeler.com
{
  static int16_t za1, za2;
  static int16_t zb0, zb1, zb2;
  static int16_t zc0, zc1, zc2;

  if (filt < 4)
  { // for SSB filters
    // 1st Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    // M0PUB: There was a bug here, since za1 == zz1 at this point in the code, and the old algorithm for the 300Hz high-pass was:
    //    za0=(29*(za0-zz1)+50*za1)/64;
    //    zz2=zz1;
    //    zz1=za0;
    // After correction, this filter still introduced almost 6dB attenuation, so I adjusted the coefficients
    static int16_t zz1, zz2;
    //za0=(29*(za0-zz1)+50*za1)/64;                                //300-Hz
    zz2 = zz1;
    zz1 = za0;
    za0 = (30 * (za0 - zz2) + 25 * za1) / 32; //300-Hz

    // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    switch (filt)
    {
    case 1:
      zb0 = (za0 + 2 * za1 + za2) / 2 - (13 * zb1 + 11 * zb2) / 16;
      break; // 0-2900Hz filter, first biquad section
    case 2:
      zb0 = (za0 + 2 * za1 + za2) / 2 - (2 * zb1 + 8 * zb2) / 16;
      break; // 0-2400Hz filter, first biquad section
    //case 3: zb0=(za0+2*za1+za2)/2-(4*zb1+2*zb2)/16; break;     // 0-2400Hz filter, first biquad section
    case 3:
      zb0 = (za0 + 2 * za1 + za2) / 2 - (0 * zb1 + 4 * zb2) / 16;
      break; //0-1800Hz  elliptic
      //case 3: zb0=(za0+7*za1+za2)/16-(-24*zb1+9*zb2)/16; break;  //0-1700Hz  elliptic with slope
    }

    switch (filt)
    {
    case 1:
      zc0 = (zb0 + 2 * zb1 + zb2) / 2 - (18 * zc1 + 11 * zc2) / 16;
      break; // 0-2900Hz filter, second biquad section
    case 2:
      zc0 = (zb0 + 2 * zb1 + zb2) / 4 - (4 * zc1 + 8 * zc2) / 16;
      break; // 0-2400Hz filter, second biquad section
    //case 3: zc0=(zb0+2*zb1+zb2)/4-(1*zc1+9*zc2)/16; break;       // 0-2400Hz filter, second biquad section
    case 3:
      zc0 = (zb0 + 2 * zb1 + zb2) / 4 - (0 * zc1 + 4 * zc2) / 16;
      break; //0-1800Hz  elliptic
      //case 3: zc0=(zb0+zb1+zb2)/16-(-22*zc1+47*zc2)/64; break;   //0-1700Hz  elliptic with slope
    }
    /*switch(filt){
      case 1: zb0=za0; break; //0-4000Hz (pass-through)
      case 2: zb0=(10*(za0+2*za1+za2)+16*zb1-17*zb2)/32; break;    //0-2500Hz  elliptic -60dB@3kHz
      case 3: zb0=(7*(za0+2*za1+za2)+48*zb1-18*zb2)/32; break;     //0-1700Hz  elliptic
    }
  
    switch(filt){
      case 1: zc0=zb0; break; //0-4000Hz (pass-through)
      case 2: zc0=(8*(zb0+zb2)+13*zb1-43*zc1-52*zc2)/64; break;   //0-2500Hz  elliptic -60dB@3kHz
      case 3: zc0=(4*(zb0+zb1+zb2)+22*zc1-47*zc2)/64; break;   //0-1700Hz  elliptic
    }*/

    zc2 = zc1;
    zc1 = zc0;

    zb2 = zb1;
    zb1 = zb0;

    za2 = za1;
    za1 = za0;

    return zc0;
  }
  else
  { // for CW filters
    //   (2nd Order (SR=4465Hz) IIR in Direct Form I, 8x8:16), adding 64x front-gain (to deal with later division)
//#define FILTER_700HZ   1
#ifdef FILTER_700HZ
    if (cw_tone == 0)
    {
      switch (filt)
      {
      case 4:
        zb0 = (za0 + 2 * za1 + za2) / 2 + (41L * zb1 - 23L * zb2) / 32;
        break; //500-1000Hz
      case 5:
        zb0 = 5 * (za0 - 2 * za1 + za2) + (105L * zb1 - 58L * zb2) / 64;
        break; //650-840Hz
      case 6:
        zb0 = 3 * (za0 - 2 * za1 + za2) + (108L * zb1 - 61L * zb2) / 64;
        break; //650-750Hz
      case 7:
        zb0 = (2 * za0 - 3 * za1 + 2 * za2) + (111L * zb1 - 62L * zb2) / 64;
        break; //630-680Hz
        //case 4: zb0=(0*za0+1*za1+0*za2)+(28*zb1-14*zb2)/16; break; //600Hz+-250Hz
        //case 5: zb0=(0*za0+1*za1+0*za2)+(28*zb1-15*zb2)/16; break; //600Hz+-100Hz
        //case 6: zb0=(0*za0+1*za1+0*za2)+(27*zb1-15*zb2)/16; break; //600Hz+-50Hz
        //case 7: zb0=(0*za0+1*za1+0*za2)+(27*zb1-15*zb2)/16; break; //630Hz+-18Hz
      }

      switch (filt)
      {
      case 4:
        zc0 = (zb0 - 2 * zb1 + zb2) / 4 + (105L * zc1 - 52L * zc2) / 64;
        break; //500-1000Hz
      case 5:
        zc0 = ((zb0 + 2 * zb1 + zb2) + 97L * zc1 - 57L * zc2) / 64;
        break; //650-840Hz
      case 6:
        zc0 = ((zb0 + zb1 + zb2) + 104L * zc1 - 60L * zc2) / 64;
        break; //650-750Hz
      case 7:
        zc0 = ((zb1) + 109L * zc1 - 62L * zc2) / 64;
        break; //630-680Hz
        //case 4: zc0=(zb0-2*zb1+zb2)/1+(24*zc1-13*zc2)/16; break; //600Hz+-250Hz
        //case 5: zc0=(zb0-2*zb1+zb2)/4+(26*zc1-14*zc2)/16; break; //600Hz+-100Hz
        //case 6: zc0=(zb0-2*zb1+zb2)/16+(28*zc1-15*zc2)/16; break; //600Hz+-50Hz
        //case 7: zc0=(zb0-2*zb1+zb2)/32+(27*zc1-15*zc2)/16; break; //630Hz+-18Hz
      }
    }
    if (cw_tone == 1)
#endif
    {
      switch (filt)
      {
        //case 4: zb0=(1*za0+2*za1+1*za2)+(90L*zb1-38L*zb2)/64; break; //600Hz+-250Hz
        //case 5: zb0=(1*za0+2*za1+1*za2)/2+(102L*zb1-52L*zb2)/64; break; //600Hz+-100Hz
        //case 6: zb0=(1*za0+2*za1+1*za2)/2+(107L*zb1-57L*zb2)/64; break; //600Hz+-50Hz
        //case 7: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-61L*zb2)/64; break; //600Hz+-25Hz

      case 4:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (114L * zb1 - 57L * zb2) / 64;
        break; //600Hz+-250Hz
      case 5:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (113L * zb1 - 60L * zb2) / 64;
        break; //600Hz+-100Hz
      case 6:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (110L * zb1 - 62L * zb2) / 64;
        break; //600Hz+-50Hz
      case 7:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (110L * zb1 - 61L * zb2) / 64;
        break; //600Hz+-18Hz
        //case 8: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-60L*zb2)/64; break; //591Hz+-12Hz

        /*case 4: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+7L*zb2)/64; break; //600Hz+-250Hz
        case 5: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-15L*zb1+4L*zb2)/64; break; //600Hz+-100Hz
        case 6: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+2L*zb2)/64; break; //600Hz+-50Hz
        case 7: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+3L*zb2)/64; break; //600Hz+-18Hz*/
      }

      switch (filt)
      {
        //case 4: zc0=(zb0-2*zb1+zb2)/4+(95L*zc1-44L*zc2)/64; break; //600Hz+-250Hz
        //case 5: zc0=(zb0-2*zb1+zb2)/8+(104L*zc1-53L*zc2)/64; break; //600Hz+-100Hz
        //case 6: zc0=(zb0-2*zb1+zb2)/16+(106L*zc1-56L*zc2)/64; break; //600Hz+-50Hz
        //case 7: zc0=(zb0-2*zb1+zb2)/32+(112L*zc1-62L*zc2)/64; break; //600Hz+-25Hz

      case 4:
        zc0 = (zb0 - 2 * zb1 + zb2) / 1 + (95L * zc1 - 52L * zc2) / 64;
        break; //600Hz+-250Hz
      case 5:
        zc0 = (zb0 - 2 * zb1 + zb2) / 4 + (106L * zc1 - 59L * zc2) / 64;
        break; //600Hz+-100Hz
      case 6:
        zc0 = (zb0 - 2 * zb1 + zb2) / 16 + (113L * zc1 - 62L * zc2) / 64;
        break; //600Hz+-50Hz
      case 7:
        zc0 = (zb0 - 2 * zb1 + zb2) / 32 + (112L * zc1 - 62L * zc2) / 64;
        break; //600Hz+-18Hz
        //case 8: zc0=(zb0-2*zb1+zb2)/64+(113L*zc1-63L*zc2)/64; break; //591Hz+-12Hz

        /*case 4: zc0=(zb0-2*zb1+zb2)/1+zc1-zc2+(31L*zc1+12L*zc2)/64; break; //600Hz+-250Hz
        case 5: zc0=(zb0-2*zb1+zb2)/4+2*zc1-zc2+(-22L*zc1+5L*zc2)/64; break; //600Hz+-100Hz
        case 6: zc0=(zb0-2*zb1+zb2)/16+2*zc1-zc2+(-15L*zc1+2L*zc2)/64; break; //600Hz+-50Hz
        case 7: zc0=(zb0-2*zb1+zb2)/16+2*zc1-zc2+(-16L*zc1+2L*zc2)/64; break; //600Hz+-18Hz*/
      }
    }
    zc2 = zc1;
    zc1 = zc0;

    zb2 = zb1;
    zb1 = zb0;

    za2 = za1;
    za1 = za0;

    //return zc0 / 64; // compensate the 64x front-end gain
    return zc0 / 8; // compensate the front-end gain
  }
}

static uint32_t absavg256 = 0;
volatile uint32_t _absavg256 = 0;
volatile int16_t i, q;

inline int16_t slow_dsp(int16_t ac)
{
  static uint8_t absavg256cnt;
  if (!(absavg256cnt--))
  {
    _absavg256 = absavg256;
    absavg256 = 0;
  }
  else
    absavg256 += abs(ac);

  if (mode == AM)
  {
    ac = magn(i, q);
    //static int16_t last_sample = 1;
    //MLEA(last_sample,ac, 1, 2); // 1/2: alpha = 0.25, Lyons 13.33.2 p.763
    /*{ static int16_t dc;
      dc += (ac - dc) / 2;
      ac = ac - dc; }  // DC decoupling*/
  }
  else if (mode == FM)
  {
    static int16_t z1;
    int16_t z0 = _arctan3(q, i);
    ac = z0 - z1; // Differentiator
    z1 = z0;
    //ac = ac * (F_SAMP_RX/R) / _UA;  // =ac*3.5 -> skip
  } // needs: p.12 https://www.veron.nl/wp-content/uploads/2014/01/FmDemodulator.pdf
  else
  {
    ;
  } // USB, LSB, CW

#ifdef FAST_AGC
  if (agc == 2)
  {
    ac = process_agc(ac);
    ac = ac >> (16 - volume);
  }
  else if (agc == 1)
  {
    ac = process_agc_fast(ac);
    ac = ac >> (16 - volume);
#else
  if (agc == 1)
  {
    ac = process_agc_fast(ac);
    ac = ac >> (16 - volume);
#endif //!FAST_AGC
  }
  else
  {
    //ac = ac >> (16-volume);
    if (volume <= 13) // if no AGC allow volume control to boost weak signals
      ac = ac >> (13 - volume);
    else
      ac = ac << (volume - 13);
  }
  if (nr)
    ac = process_nr(ac);

  //  if(filt) ac = filt_var(ac) << 2;
  if (filt)
    ac = filt_var(ac);
/*
  if(mode == CW){
    if(cwdec){  // CW decoder enabled?
      char ch = cw(ac >> 0);
      if(ch){
        for(int i=0; i!=15;i++) out[i]=out[i+1];
        out[15] = ch;
        cw_event = true;
      }
    }
  }*/
#ifdef CW_DECODER
  if (!(absavg256cnt % 64))
  {
    _amp32 = amp32;
    amp32 = 0;
  }
  else
    amp32 += abs(ac);
#endif //CW_DECODER
  //if(!(absavg256cnt--)){ _absavg256 = absavg256; absavg256 = 0; } else absavg256 += abs(ac);  //hack

  //static int16_t dc;
  //dc += (ac - dc) / 2;
  //dc = (15*dc + ac)/16;
  //dc = (15*dc + (ac - dc))/16;
  //ac = ac - dc;    // DC decoupling

  ac = min(max(ac, -512), 511);
  //ac = min(max(ac, -128), 127);
#ifdef QCX
  if (!dsp_cap)
    return 0; // in QCX-SSB mode (no DSP), slow_dsp() should return 0 (in order to prevent upsampling filter to generate audio)
#endif
  return ac;
}

#ifdef TESTBENCH
// Sine table with 72 entries results in 868Hz sine wave at effective sampling rate of 31250 SPS
// for each of I and Q, since thay are sampled alternately, and hence I (for example) only
// gets 36 samples from this table before looping.
const int8_t sine[] = {
    11, 22, 33, 43, 54, 64, 73, 82, 90, 97, 104, 110, 115, 119, 123, 125, 127, 127, 127, 125, 123, 119, 115, 110, 104, 97, 90, 82, 73, 64, 54, 43, 33, 22, 11, 0, -11, -22, -33, -43, -54, -64, -73, -82, -90, -97, -104, -110, -115, -119, -123, -125, -127, -127, -127, -125, -123, -119, -115, -110, -104, -97, -90, -82, -73, -64, -54, -43, -33, -22, -11, 0};

// Short Sine table with 36 entries results in 1736Hz sine wave at effective sampling rate of 62500 SPS.
/* const int8_t sine[] = {
  22, 43, 64, 82, 97, 110, 119, 125, 127, 125, 119, 110, 97, 82, 64, 43, 22, 0, -22, -43, -64, -82, -97, -110, -119, -125, -127, -125, -119, -110, -97, -82, -64, -43, -22, 0
}; */

uint8_t ncoIdx = 0;
int16_t NCO_Q()
{
  ncoIdx++;
  if (ncoIdx >= sizeof(sine))
    ncoIdx = 0;
  return (int16_t(sine[ncoIdx])) << 2;
}

int16_t NCO_I()
{
  uint8_t i;

  ncoIdx++;
  if (ncoIdx >= sizeof(sine))
    ncoIdx = 0;

  i = ncoIdx + (sizeof(sine) / 4); // Advance by 90 degrees
  if (i >= sizeof(sine))
    i -= sizeof(sine);
  return (int16_t(sine[i])) << 2;
}
#endif // TESTBENCH

/*ISR (TIMER2_COMPA_vect  ,ISR_NAKED) {
asm("push r24         \n\t"
    "lds r24,  0\n\t"
    "sts 0xB4, r24    \n\t"
    "pop r24          \n\t"
    "reti             \n\t");
}*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Below a radio-specific implementation based on the above components (seperation of concerns)
//
// Feel free to replace it with your own custom radio implementation :-)

volatile bool change = true;
volatile int32_t freq = 14000000;
static int32_t vfo[] = {7074000, 14074000};
static uint8_t vfomode[] = {USB, USB};
enum vfo_t
{
  VFOA = 0,
  VFOB = 1,
  SPLIT = 2
};
volatile uint8_t vfosel = VFOA;
volatile int16_t rit = 0;

// We measure the average amplitude of the signal (see slow_dsp()) but the S-meter should be based on RMS value.
// So we multiply by 0.707/0.639 in an attempt to roughly compensate, although that only really works if the input
// is a sine wave
uint8_t smode = 1;
uint32_t max_absavg256 = 0;
int16_t dbm;

static int16_t smeter_cnt = 0;

int16_t smeter(int16_t ref = 0)
{
  max_absavg256 = max(_absavg256, max_absavg256); // peak

  if ((smode) && ((++smeter_cnt % 2048) == 0))
  { // slowed down display slightly
    float rms = (float)max_absavg256 * (float)(1 << att2);
    if (dsp_cap == SDR)
      rms /= (256.0 * 1024.0 * (float)R * 8.0 * 500.0 * 1.414 / (0.707 * 1.1)); // = -98.8dB  1 rx gain stage: rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * "RMS compensation"]
    else
      rms /= (256.0 * 1024.0 * (float)R * 2.0 * 100.0 * 120.0 / (1.750 * 5.0)); // = -94.6dB
    dbm = 10 * log10((rms * rms) / 50) + 30 - ref;                              //from rmsV to dBm at 50R

    lcd.noCursor();
    if (smode == 1)
    { // dBm meter
      lcd.setCursor(9, 0);
      lcd.print((int16_t)dbm);
      lcd.print(F("dBm "));
    }
    if (smode == 2)
    {                                                                                    // S-meter
      uint8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10; // dBm to S (modified to work correctly above S9)
      lcd.setCursor(14, 0);
      if (s < 10)
      {
        lcd.print('S');
      }
      lcd.print(s);
    }
    if (smode == 3)
    {                                                                                   // S-bar
      int8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10; // dBm to S (modified to work correctly above S9)
      char tmp[5];
      for (uint8_t i = 0; i != 4; i++)
      {
        tmp[i] = max(2, min(5, s + 1));
        s = s - 3;
      }
      tmp[4] = 0;
      lcd.setCursor(12, 0);
      lcd.print(tmp);
    }
#ifdef CW_DECODER
    if (smode == 4)
    { // wpm-indicator
      lcd.setCursor(14, 0);
      if (mode == CW)
        lcd.print(wpm);
      lcd.print("  ");
    }
#endif //CW_DECODER
#ifdef VSS_METER
    if (smode == 5)
    {             // Supply-voltage indicator; add resistor Rvss (see below) between 12V supply input and pin 26 (PC3)   Contribution by Jeff WB4LCG: https://groups.io/g/ucx/message/4470
#define Rgnd 10   //kOhm (PC3 to GND)
#define Rvss 1000 //kOhm (PC3 to VSS)
      uint16_t vss = analogSafeRead(BUTTONS, true) * (Rvss + Rgnd) * 11 / (Rgnd * 1024);
      lcd.setCursor(10, 0);
      lcd.print(vss / 10);
      lcd.print('.');
      lcd.print(vss % 10);
      lcd.print("V ");
    }
#endif //VSS_METER
#ifdef CLOCK
    if (smode == 6)
    { // clock-indicator
      uint32_t _s = (millis() * 16000000ULL / F_MCU) / 1000;
      uint8_t h = (_s / 3600) % 24;
      uint8_t m = (_s / 60) % 60;
      uint8_t s = (_s) % 60;
      lcd.setCursor(8, 0);
      lcd.print(h / 10);
      lcd.print(h % 10);
      lcd.print(':');
      lcd.print(m / 10);
      lcd.print(m % 10);
      lcd.print(':');
      lcd.print(s / 10);
      lcd.print(s % 10);
      lcd.print("  ");
    }
#endif //CLOCK
    stepsize_showcursor();
    max_absavg256 /= 2; // Implement peak hold/decay for all meter types
  }
  return dbm;
}

void start_rx()
{
  _init = 1;
  rx_state = 0;
  func_ptr = sdr_rx_00; //enable RX DSP/SDR
  adc_start(2, true, F_ADC_CONV * 4);
  admux[2] = ADMUX; // Note that conversion-rate for TX is factors more
  if (dsp_cap == SDR)
  {
//#define SWAP_RX_IQ 1    // Swap I/Q ADC inputs, flips RX sideband
#ifdef SWAP_RX_IQ
    adc_start(1, !(att == 1) /*true*/, F_ADC_CONV);
    admux[0] = ADMUX;
    adc_start(0, !(att == 1) /*true*/, F_ADC_CONV);
    admux[1] = ADMUX;
#else
    adc_start(0, !(att == 1) /*true*/, F_ADC_CONV);
    admux[0] = ADMUX;
    adc_start(1, !(att == 1) /*true*/, F_ADC_CONV);
    admux[1] = ADMUX;
#endif //SWAP_RX_IQ
  }
  else
  { // ANALOG, DSP
    adc_start(0, false, F_ADC_CONV);
    admux[0] = ADMUX;
    admux[1] = ADMUX;
  }
  timer1_start(F_SAMP_PWM);
  timer2_start(F_SAMP_RX);
  TCCR1A &= ~(1 << COM1B1);
  digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM
}

int16_t _centiGain = 0;

uint8_t txdelay = 0;
uint8_t semi_qsk = false;
uint32_t semi_qsk_timeout = 0;

void switch_rxtx(uint8_t tx_enable)
{
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  //delay(1);
  delayMicroseconds(20); // wait until potential RX interrupt is finalized
  noInterrupts();
#ifdef TX_DELAY
#ifdef SEMI_QSK
  if (!(semi_qsk_timeout))
#endif
    if ((txdelay) && (tx_enable) && (!(tx)) && (!(practice)))
    {                        // key-up TX relay in advance before actual transmission
      digitalWrite(RX, LOW); // TX (disable RX)
#ifdef NTX
      digitalWrite(NTX, LOW); // TX (enable TX)
#endif                        //NTX
#ifdef PTX
      digitalWrite(PTX, HIGH); // TX (enable TX)
#endif                         //PTX
      lcd.setCursor(15, 1);
      lcd.print('D'); // note that this enables interrupts again.
      interrupts();   //hack.. to allow delay()
      delay(F_MCU * txdelay / 16000000);
      noInterrupts(); //end of hack
    }
#endif //TX_DELAY
  tx = tx_enable;
  if (tx_enable)
  {                         // tx
    _centiGain = centiGain; // backup AGC setting
#ifdef SEMI_QSK
    semi_qsk_timeout = 0;
#endif
    switch (mode)
    {
    case USB:
    case LSB:
      func_ptr = dsp_tx;
      break;
    case CW:
      func_ptr = dsp_tx_cw;
      break;
    case AM:
      func_ptr = dsp_tx_am;
      break;
    case FM:
      func_ptr = dsp_tx_fm;
      break;
    }
  }
  else
  { // rx
    if ((mode == CW) && (!(semi_qsk_timeout)))
    {
#ifdef SEMI_QSK
#ifdef KEYER
      semi_qsk_timeout = millis() + ditTime * 8;
#else
      semi_qsk_timeout = millis() + 8 * 8; // no keyer? assume dit-time of 20 WPM
#endif //KEYER
#endif //SEMI_QSK
      if (semi_qsk)
        func_ptr = dummy;
      else
        func_ptr = sdr_rx_00;
    }
    else
    {
      centiGain = _centiGain; // restore AGC setting
#ifdef SEMI_QSK
      semi_qsk_timeout = 0;
#endif
      func_ptr = sdr_rx_00;
    }
  }
  if ((!dsp_cap) && (!tx_enable) && vox)
    func_ptr = dummy; //hack: for SSB mode, disable dsp_rx during vox mode enabled as it slows down the vox loop too much!
  interrupts();
  if (tx_enable)
    ADMUX = admux[2];
  else
    _init = 1;
  rx_state = 0;
#ifdef CW_DECODER
  if ((cwdec) && (mode == CW))
  {
    filteredstate = tx_enable;
    dec2();
  }
#endif //CW_DECODER

  if (tx_enable)
  { // tx
    if (practice)
    {
      digitalWrite(RX, LOW); // TX (disable RX)
      lcd.setCursor(15, 1);
      lcd.print('P');
      si5351.SendRegister(SI_CLK_OE, 0b11111111); // CLK2_EN,CLK1_EN,CLK0_EN=0
      // Do not enable PWM (KEY_OUT), do not enble CLK2
    }
    else
    {
      digitalWrite(RX, LOW); // TX (disable RX)
#ifdef NTX
      digitalWrite(NTX, LOW); // TX (enable TX)
#endif                        //NTX
#ifdef PTX
      digitalWrite(PTX, HIGH); // TX (enable TX)
#endif                         //PTX
      lcd.setCursor(15, 1);
      lcd.print('T');
      if (mode == CW)
      {
        si5351.freq_calc_fast(-cw_offset);
        si5351.SendPLLRegisterBulk();
      } // for CW, TX at freq
#ifdef RIT_ENABLE
      else if (rit)
      {
        si5351.freq_calc_fast(0);
        si5351.SendPLLRegisterBulk();
      }
#endif //RIT_ENABLE
#ifdef TX_CLK0_CLK1
      si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
#else
      si5351.SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
#endif               //TX_CLK0_CLK1
      OCR1AL = 0x80; // make sure SIDETONE is set at 2.5V
      if ((!mox) && (mode != CW))
        TCCR1A &= ~(1 << COM1A1); // disable SIDETONE, prevent interference during SSB TX
      TCCR1A |= (1 << COM1B1);    // enable KEY_OUT PWM
#ifdef _SERIAL
      if (cat_active)
      {
        DDRC &= ~(1 << 2);
      } // disable PC2, so that ADC2 can be used as mic input
#endif
    }
  }
  else
  { // rx
#ifdef KEY_CLICK
    if (OCR1BL != 0)
    {
      for (uint16_t i = 0; i != 31; i++)
      { // ramp down of amplitude: soft falling edge to prevent key clicks
        OCR1BL = lut[pgm_read_byte_near(ramp[i])];
        delayMicroseconds(60);
      }
    }
#endif                       //KEY_CLICK
    TCCR1A |= (1 << COM1A1); // enable SIDETONE (was disabled to prevent interference during ssb tx)
    TCCR1A &= ~(1 << COM1B1);
    digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM, prevents interference during RX
    OCR1BL = 0;                 // make sure PWM (KEY_OUT) is set to 0%
#ifdef QUAD
#ifdef TX_CLK0_CLK1
    si5351.SendRegister(16, 0x0f); // disable invert on CLK0
    si5351.SendRegister(17, 0x0f); // disable invert on CLK1
#else
    si5351.SendRegister(18, 0x0f);         // disable invert on CLK2
#endif                                          //TX_CLK0_CLK1
#endif                                          //QUAD
    si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
#ifdef SEMI_QSK
    if ((!semi_qsk_timeout) || (!semi_qsk)) // enable RX when no longer in semi-qsk phase; so RX and NTX/PTX outputs are switching only when in RX mode
#endif                                      //SEMI_QSK
    {
      digitalWrite(RX, !(att == 2)); // RX (enable RX when attenuator not on)
#ifdef NTX
      digitalWrite(NTX, HIGH); // RX (disable TX)
#endif                         //NTX
#ifdef PTX
      digitalWrite(PTX, LOW); // TX (disable TX)
#endif                        //PTX
    }
#ifdef RIT_ENABLE
    si5351.freq_calc_fast(rit);
    si5351.SendPLLRegisterBulk(); // restore original PLL RX frequency
#else
    si5351.freq_calc_fast(0);
    si5351.SendPLLRegisterBulk();                                                                                                                                          // restore original PLL RX frequency
#endif //RIT_ENABLE
#ifdef SWR_METER
    if (swrmeter > 0)
    {
      show_banner();
      lcd.print("                ");
    }
#endif
    lcd.setCursor(15, 1);
    lcd.print((vox) ? 'V' : 'R');
#ifdef _SERIAL
    if (!vox)
      if (cat_active)
      {
        DDRC |= (1 << 2);
      } // enable PC2, so that ADC2 is pulled-down so that CAT TX is not disrupted via mic input
#endif
  }
  OCR2A = ((F_CPU / 64) / ((tx_enable) ? F_SAMP_TX : F_SAMP_RX)) - 1;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt TIMER2_COMPA_vect
}

uint8_t rx_ph_q = 90;

#ifdef QCX
#define CAL_IQ 1
#ifdef CAL_IQ
int16_t cal_iq_dummy = 0;
// RX I/Q calibration procedure: terminate with 50 ohm, enable CW filter, adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB
void calibrate_iq()
{
  smode = 1;
  lcd.setCursor(0, 0);
  lcd_blanks();
  lcd_blanks();
  digitalWrite(SIG_OUT, true);                // loopback on
  si5351.freq(freq, 0, 90);                   // RX in USB
  si5351.SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  float dbc;
  si5351.freqb(freq + 700);
  delay(100);
  dbc = smeter();
  si5351.freqb(freq - 700);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("I-Q bal. 700Hz");
  lcd_blanks();
  for (; !_digitalRead(BUTTONS);)
  {
    wdt_reset();
    smeter(dbc);
  }
  for (; _digitalRead(BUTTONS);)
    wdt_reset();
  si5351.freqb(freq + 600);
  delay(100);
  dbc = smeter();
  si5351.freqb(freq - 600);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Phase Lo 600Hz");
  lcd_blanks();
  for (; !_digitalRead(BUTTONS);)
  {
    wdt_reset();
    smeter(dbc);
  }
  for (; _digitalRead(BUTTONS);)
    wdt_reset();
  si5351.freqb(freq + 800);
  delay(100);
  dbc = smeter();
  si5351.freqb(freq - 800);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Phase Hi 800Hz");
  lcd_blanks();
  for (; !_digitalRead(BUTTONS);)
  {
    wdt_reset();
    smeter(dbc);
  }
  for (; _digitalRead(BUTTONS);)
    wdt_reset();

  lcd.setCursor(9, 0);
  lcd_blanks();                               // cleanup dbmeter
  digitalWrite(SIG_OUT, false);               // loopback off
  si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  change = true;                              //restore original frequency setting
}
#endif
#endif //QCX

uint8_t prev_bandval = 3;
uint8_t bandval = 3;
#define N_BANDS 11

#ifdef CW_FREQS_QRP
uint32_t band[N_BANDS] = {/*472000,*/ 1810000, 3560000, 5351500, 7030000, 10106000, 14060000, 18096000, 21060000, 24906000, 28060000, 50096000 /*, 70160000, 144060000*/}; // CW QRP freqs
#else
#ifdef CW_FREQS_FISTS
uint32_t band[N_BANDS] = {/*472000,*/ 1818000, 3558000, 5351500, 7028000, 10118000, 14058000, 18085000, 21058000, 24908000, 28058000, 50058000 /*, 70158000, 144058000*/}; // CW FISTS freqs
#else
uint32_t band[N_BANDS] = {/*472000,*/ 1840000, 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000 /*, 70101000, 144125000*/}; // FT8 freqs
#endif
#endif

enum step_t
{
  STEP_10M,
  STEP_1M,
  STEP_500k,
  STEP_100k,
  STEP_10k,
  STEP_1k,
  STEP_500,
  STEP_100,
  STEP_10,
  STEP_1
};
uint32_t stepsizes[10] = {10000000, 1000000, 500000, 100000, 10000, 1000, 500, 100, 10, 1};
volatile uint8_t stepsize = STEP_1k;
uint8_t prev_stepsize[] = {STEP_1k, STEP_500}; //default stepsize for resp. SSB, CW

void process_encoder_tuning_step(int8_t steps)
{
  int32_t stepval = stepsizes[stepsize];
  //if(stepsize < STEP_100) freq %= 1000; // when tuned and stepsize > 100Hz then forget fine-tuning details
  if (rit)
  {
    rit += steps * stepval;
    rit = max(-9999, min(9999, rit));
  }
  else
  {
    freq += steps * stepval;
    freq = max(1, min(999999999, freq));
  }
  change = true;
}

void stepsize_showcursor()
{
  lcd.setCursor(stepsize + 1, 1); // display stepsize with cursor
  lcd.cursor();
}

void stepsize_change(int8_t val)
{
  stepsize += val;
  if (stepsize < STEP_1M)
    stepsize = STEP_10;
  if (stepsize > STEP_10)
    stepsize = STEP_1M;
  if (stepsize == STEP_10k || stepsize == STEP_500k)
    stepsize += val;
  stepsize_showcursor();
}

void powerDown()
{ // Reduces power from 110mA to 70mA (back-light on) or 30mA (back-light off), remaining current is probably opamp quiescent current
  lcd.setCursor(0, 1);
  lcd.print(F("Power-off 73 :-)"));
  lcd_blanks();

  MCUSR = ~(1 << WDRF); // MSY be done before wdt_disable()
  wdt_disable();        // WDTON Fuse High bit need to be 1 (0xD1), if NOT it will override and set WDE=1; WDIE=0, meaning MCU will reset when watchdog timer is zero, and this seems to happen when wdt_disable() is called

  timer2_stop();
  timer1_stop();
  adc_stop();

  si5351.powerDown();

  delay(1500);

  // Disable external interrupts INT0, INT1, Pin Change
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
  // Disable internal interrupts
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  WDTCSR = 0;
  // Enable BUTTON Pin Change interrupt
  *digitalPinToPCMSK(BUTTONS) |= (1 << digitalPinToPCMSKbit(BUTTONS));
  *digitalPinToPCICR(BUTTONS) |= (1 << digitalPinToPCICRbit(BUTTONS));

  // Power-down sub-systems
  PRR = 0xff;

  lcd.noDisplay();
  PORTD &= ~0x08; // disable backlight

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  interrupts();
  sleep_bod_disable();
  //MCUCR |= (1<<BODS) | (1<<BODSE);  // turn bod off by settings BODS, BODSE; note BODS is reset after three clock-cycles, so quickly go to sleep before it is too late
  //MCUCR &= ~(1<<BODSE);  // must be done right before sleep
  sleep_cpu(); // go to sleep mode, wake-up by either INT0, INT1, Pin Change, TWI Addr Match, WDT, BOD
  sleep_disable();

  //void(* reset)(void) = 0; reset();   // soft reset by calling reset vector (does not reset registers to defaults)
  do
  {
    wdt_enable(WDTO_15MS);
    for (;;)
      ;
  } while (0); // soft reset by trigger watchdog timeout
}

void setup()
{
  digitalWrite(KEY_OUT, LOW); // for safety: to prevent exploding PA MOSFETs, in case there was something still biasing them.
  si5351.powerDown();         // disable all CLK outputs (especially needed for si5351 variants that has CLK2 enabled by default, such as Si5351A-B04486-GT)

  MCUSR = 0;

  wdt_enable(WDTO_4S); // Enable watchdog
  uint32_t t0, t1;
#ifdef DEBUG
  // Benchmark dsp_tx() ISR (this needs to be done in beginning of setup() otherwise when VERSION containts 5 chars, mis-alignment impact performance by a few percent)
  func_ptr = dsp_tx;
  t0 = micros();
  TIMER2_COMPA_vect();
  //func_ptr();
  t1 = micros();
  uint16_t load_tx = (float)(t1 - t0) * (float)F_SAMP_TX * 100.0 / 1000000.0 * 16000000.0 / (float)F_CPU;
  // benchmark sdr_rx_00() ISR
  func_ptr = sdr_rx_00;
  rx_state = 0;
  uint16_t load_rx[8];
  uint16_t load_rx_avg = 0;
  uint16_t i;
  for (i = 0; i != 8; i++)
  {
    rx_state = i;
    t0 = micros();
    TIMER2_COMPA_vect();
    //func_ptr();
    t1 = micros();
    load_rx[i] = (float)(t1 - t0) * (float)F_SAMP_RX * 100.0 / 1000000.0 * 16000000.0 / (float)F_CPU;
    load_rx_avg += load_rx[i];
  }
  load_rx_avg /= 8;

  //adc_stop();  // recover general ADC settings so that analogRead is working again
#endif                  //DEBUG
  ADMUX = (1 << REFS0); // restore reference voltage AREF (5V)

  // disable external interrupts
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;

  encoder_setup();
  initPins();

  delay(100);       // at least 40ms after power rises above 2.7V before sending commands
  lcd.begin(16, 2); // Init LCD

  //TODO: join all of that in show banner
  show_banner();
  lcd.setCursor(7, 0);
  lcd.print(F(" R"));
  lcd.print(F(VERSION));
  lcd_blanks();

#ifdef QCX
  // Test if QCX has DSP/SDR capability: SIDETONE output disconnected from AUDIO2
  si5351.SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0
  digitalWrite(RX, HIGH);                     // generate pulse on SIDETONE and test if it can be seen on AUDIO2
  delay(1);                                   // settle
  digitalWrite(SIDETONE, LOW);
  int16_t v1 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, HIGH);
  int16_t v2 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, LOW);
  dsp_cap = !(abs(v2 - v1) > (0.05 * 1024.0 / 5.0)); // DSP capability?
  if (dsp_cap)
  { // Test if QCX has SDR capability: AUDIO2 is disconnected from AUDIO1  (only in case of DSP capability)
    delay(400);
    wdt_reset(); // settle:  the following test only works well 400ms after startup
    v1 = analogRead(AUDIO1);
    digitalWrite(AUDIO2, HIGH); // generate pulse on AUDIO2 and test if it can be seen on AUDIO1
    pinMode(AUDIO2, OUTPUT);
    delay(1);
    digitalWrite(AUDIO2, LOW);
    delay(1);
    digitalWrite(AUDIO2, HIGH);
    v2 = analogRead(AUDIO1);
    pinMode(AUDIO2, INPUT);
    if (!(abs(v2 - v1) > (0.125 * 1024.0 / 5.0)))
      dsp_cap = SDR; // SDR capacility?
  }
  // Test if QCX has SSB capability: DAH is connected to DVM
  delay(1); // settle
  pinMode(DAH, OUTPUT);
  digitalWrite(DAH, LOW);
  v1 = analogRead(DVM);
  digitalWrite(DAH, HIGH);
  v2 = analogRead(DVM);
  digitalWrite(DAH, LOW);
  //pinMode(DAH, INPUT_PULLUP);
  pinMode(DAH, INPUT);
  ssb_cap = (abs(v2 - v1) > (0.05 * 1024.0 / 5.0)); // SSB capability?

  //ssb_cap = 0; dsp_cap = ANALOG;  // force standard QCX capability
  //ssb_cap = 1; dsp_cap = ANALOG;  // force SSB and standard QCX-RX capability
  //ssb_cap = 1; dsp_cap = DSP;     // force SSB and DSP capability
  //ssb_cap = 1; dsp_cap = SDR;     // force SSB and SDR capability
#endif // QCX

#ifdef DEBUG
  /*if((mcusr & WDRF) && (!(mcusr & EXTRF)) && (!(mcusr & BORF))){
    lcd.setCursor(0, 1); lcd.print(F("!!Watchdog RESET")); lcd_blanks();
    delay(1500); wdt_reset();
  }
  if((mcusr & BORF) && (!(mcusr & WDRF))){
    lcd.setCursor(0, 1); lcd.print(F("!!Brownout RESET")); lcd_blanks();  // Brow-out reset happened, CPU voltage not stable or make sure Brown-Out threshold is set OK (make sure E fuse is set to FD)
    delay(1500); wdt_reset();
  }
  if(mcusr & PORF){
    lcd.setCursor(0, 1); lcd.print(F("!!Power-On RESET")); lcd_blanks();
    delay(1500); wdt_reset();
  }*/
  /*if(mcusr & EXTRF){
  lcd.setCursor(0, 1); lcd.print(F("Power-On")); lcd_blanks();
    delay(1); wdt_reset();
  }*/

  // Measure CPU loads
  if (!(load_tx <= 100))
  {
    fatal(F("CPU_tx"), load_tx, '%');
  }

  if (!(load_rx_avg <= 100))
  {
    fatal(F("CPU_rx"), load_rx_avg, '%');
  }
  /*for(i = 0; i != 8; i++){
    if(!(load_rx[i] <= 100)){   // and specify individual timings for each of the eight alternating processing functions
      //fatal(F("CPU_rx"), load_rx[i], '%');
      lcd.setCursor(0, 1); lcd.print(F("!!CPU_rx")); lcd.print(i); lcd.print('='); lcd.print(load_rx[i]); lcd.print('%'); lcd_blanks();
    }
  }*/
#endif

#ifdef DIAG
  // Measure VDD (+5V); should be ~5V
  si5351.SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0
  digitalWrite(KEY_OUT, LOW);
  digitalWrite(RX, LOW); // mute RX
  delay(100);            // settle
  float vdd = 2.0 * (float)analogRead(AUDIO2) * 5.0 / 1024.0;
  digitalWrite(RX, HIGH);
  if (!(vdd > 4.8 && vdd < 5.2))
  {
    fatal(F("V5.0"), vdd, 'V');
  }

  // Measure VEE (+3.3V); should be ~3.3V
  float vee = (float)analogRead(SCL) * 5.0 / 1024.0;
  if (!(vee > 3.2 && vee < 3.8))
  {
    fatal(F("V3.3"), vee, 'V');
  }

  // Measure AVCC via AREF and using internal 1.1V reference fed to ADC; should be ~5V
  analogRead(6);    // setup almost proper ADC readout
  bitSet(ADMUX, 3); // Switch to channel 14 (Vbg=1.1V)
  delay(1);         // delay improves accuracy
  bitSet(ADCSRA, ADSC);
  for (; bit_is_set(ADCSRA, ADSC);)
    ;
  float avcc = 1.1 * 1023.0 / ADC;
  if (!(avcc > 4.6 && avcc < 5.2))
  {
    fatal(F("Vavcc"), avcc, 'V');
  }

  // Report no SSB capability
  if (!ssb_cap)
  {
    fatal(F("No MIC input..."));
  }

  // Test microphone polarity
  /*if((ssb_cap) && (!_digitalRead(DAH))){
    fatal(F("MIC in rev.pol"));
  }*/

  // Measure DVM bias; should be ~VAREF/2
#ifdef _SERIAL
  DDRC &= ~(1 << 2); // disable PC2, so that ADC2 can be used as mic input
#else
  PORTD |= 1 << 1;
  DDRD |= 1 << 1;                                             // keep PD1 HIGH so that in case diode is installed to PC2 it is kept blocked (otherwise ADC2 input is pulled down!)
#endif
  delay(10);
#ifdef TX_ENABLE
  float dvm = (float)analogRead(DVM) * 5.0 / 1024.0;
  if ((ssb_cap) && !(dvm > 1.8 && dvm < 3.2))
  {
    fatal(F("Vadc2"), dvm, 'V');
  }
#endif

  // Measure AUDIO1, AUDIO2 bias; should be ~VAREF/2
  if (dsp_cap == SDR)
  {
    float audio1 = (float)analogRead(AUDIO1) * 5.0 / 1024.0;
    if (!(audio1 > 1.8 && audio1 < 3.2))
    {
      fatal(F("Vadc0"), audio1, 'V');
    }
    float audio2 = (float)analogRead(AUDIO2) * 5.0 / 1024.0;
    if (!(audio2 > 1.8 && audio2 < 3.2))
    {
      fatal(F("Vadc1"), audio2, 'V');
    }
  }

#ifdef TX_ENABLE
  // Measure I2C Bus speed for Bulk Transfers
  //si5351.freq(freq, 0, 90);
  wdt_reset();
  t0 = micros();
  for (i = 0; i != 1000; i++)
    si5351.SendPLLRegisterBulk();
  t1 = micros();
  uint32_t speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s
  if (false)
  {
    fatal(F("i2cspeed"), speed, 'k');
  }

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  si5351.freq(freq, 0, 90); // freq needs to be set in order to use freq_calc_fast()
  wdt_reset();
  uint16_t i2c_error = 0; // number of I2C byte transfer errors
  for (i = 0; i != 1000; i++)
  {
    si5351.freq_calc_fast(i);
    //for(int j = 0; j != 8; j++) si5351.pll_regs[j] = rand();
    si5351.SendPLLRegisterBulk();
#define SI_SYNTH_PLL_A 26
#ifdef NEW_TX
    for (int j = 4; j != 8; j++)
      if (si5351.RecvRegister(SI_SYNTH_PLL_A + j) != si5351.pll_regs[j])
        i2c_error++;
#else
    for (int j = 3; j != 8; j++)
      if (si5351.RecvRegister(SI_SYNTH_PLL_A + j) != si5351.pll_regs[j])
        i2c_error++;
#endif //NEW_TX
  }
  wdt_reset();
  if (i2c_error)
  {
    fatal(F("BER_i2c"), i2c_error, ' ');
  }
#endif //TX_ENABLE
#endif // DIAG

#ifdef QCX
  if (!ssb_cap)
  {
    vfomode[0] = CW;
    vfomode[1] = CW;
    filt = 4;
    stepsize = STEP_500;
  }
  if (dsp_cap != SDR)
    pwm_max = 255; // implies that key-shaping circuit is probably present, so use full-scale
  if (dsp_cap == DSP)
    volume = 10;
  if (!dsp_cap)
    cw_tone = 2; // use internal 700Hz QCX filter, so use same offset and keyer tone
#endif           //QCX

  cw_offset = tones[cw_tone];

  // Load parameters from EEPROM, reset to factory defaults when stored values are from a different version
  paramAction(LOAD, VERS);
  if ((eeprom_version != get_version_id()) || _digitalRead(BUTTONS))
  { // EEPROM clean: if rotary-key pressed or version signature in EEPROM does NOT corresponds with this firmware
    eeprom_version = get_version_id();
    //for(int n = 0; n != 1024; n++){ eeprom_write_byte((uint8_t *) n, 0); wdt_reset(); } //clean EEPROM
    //eeprom_write_dword((uint32_t *)EEPROM_OFFSET/3, 0x000000);
    paramAction(SAVE); // save default parameter values
    lcd.setCursor(0, 1);
    lcd.print(F("Reset settings.."));
    delay(500);
    wdt_reset();
  }
  else
  {
    paramAction(LOAD); // load all parameters
  }
  si5351.iqmsa = 0; // enforce PLL reset
  change = true;
  prev_bandval = bandval;
  vox = false; // disable VOX
  nr = 0;      // disable NR
  rit = false; // disable RIT
  freq = vfo[vfosel % 2];
  mode = vfomode[vfosel % 2];

#ifdef TX_ENABLE
  build_lut();
#endif

  show_banner(); // remove release number

  start_rx();

#if defined(CAT) || defined(TESTBENCH)
#ifdef CAT_STREAMING
#define BAUD 115200 //Baudrate used for serial communications
#else
#define BAUD 38400 //38400 //115200 //4800 //Baudrate used for serial communications (CAT, TESTBENCH)
#endif
  Serial.begin(16000000ULL * BAUD / F_MCU); // corrected for F_CPU=20M
  Command_IF();
#if !defined(OLED) && defined(TESTBENCH)
  smode = 0; // In case of LCD, turn of smeter
#endif
#endif //CAT TESTBENCH

#ifdef KEYER
  keyerState = IDLE;
  keyerControl = IAMBICB; // Or 0 for IAMBICA
  loadWPM(keyer_speed);   // Fix speed at 15 WPM
#endif                    //KEYER

  for (; !_digitalRead(DIT) || ((mode == CW && keyer_mode != SINGLE) && (!_digitalRead(DAH)));)
  {
    fatal(F("Check PTT/key"));
  } // wait until DIH/DAH/PTT is released to prevent TX on startup
}


void loop()
{
#ifdef VOX_ENABLE
  if ((vox) && ((mode == LSB) || (mode == USB)))
  { // If VOX enabled (and in LSB/USB mode), then take mic samples and feed ssb processing function, to derive amplitude, and potentially detect cross vox_threshold to detect a TX or RX event: this is expressed in tx variable
    if (!vox_tx)
    { // VOX not active
#ifdef MULTI_ADC
      if (vox_sample++ == 16)
      {                                                                // take N sample, then process
        ssb(((int16_t)(vox_adc / 16) - (512 - AF_BIAS)) >> MIC_ATTEN); // sampling mic
        vox_sample = 0;
        vox_adc = 0;
      }
      else
      {
        vox_adc += analogSampleMic();
      }
#else
      ssb(((int16_t)(analogSampleMic()) - 512) >> MIC_ATTEN); // sampling mic
#endif
      if (tx)
      { // TX triggered by audio -> TX
        vox_tx = 1;
        switch_rxtx(255);
        //for(;(tx);) wdt_reset();  // while in tx (workaround for RFI feedback related issue)
        //delay(100); tx = 255;
      }
    }
    else if (!tx)
    { // VOX activated, no audio detected -> RX
      switch_rxtx(0);
      vox_tx = 0;
      delay(32); //delay(10);
    }
  }
#endif //VOX_ENABLE

#ifdef CW_DECODER
  if ((mode == CW) && cwdec && ((!tx) && (!semi_qsk_timeout)))
    cw_decode(); // CW decoder only active during RX
#endif           //CW_DECODER

  if (menumode == 0)
  { // in main
#ifdef CW_DECODER
    if (cw_event)
    {
      uint8_t offset = (uint8_t[]){0, 7, 3, 5, 3, 7, 8}[smode]; // depending on smeter more/less cw-text
      lcd.noCursor();
#ifdef OLED
      //cw_event = false; for(int i = 0; out[offset + i] != '\0'; i++){ lcd.setCursor(i, 0); lcd.print(out[offset + i]); if((!tx) && (!semi_qsk_timeout)) cw_decode(); }   // like 'lcd.print(out + offset);' but then in parallel calling cw_decoding() to handle long OLED writes
      //uint8_t i = cw_event - 1; if(out[offset + i]){ lcd.setCursor(i, 0); lcd.print(out[offset + i]); cw_event++; } else cw_event = false;  // since an oled string write would hold-up reliable decoding/keying, write only a single char each time and continue
      uint8_t i = cw_event - 1;
      if (offset - i + 1)
      {
        lcd.setCursor(offset - i, 0);
        lcd.print(out[15 - i]);
        cw_event++;
      }
      else
        cw_event = false; // since an oled string write would hold-up reliable decoding/keying, write only a single char each time and continue
#else
      cw_event = false;
      lcd.setCursor(0, 0);
      lcd.print(out + offset);
#endif
      stepsize_showcursor();
    }
    else
#endif //CW_DECODER
        if ((!semi_qsk_timeout) && (!vox_tx))
      smeter();
  }

#ifdef KEYER //Keyer
  if (mode == CW && keyer_mode != SINGLE)
  { // check DIT/DAH keys for CW

    switch (keyerState)
    {          // Basic Iambic Keyer, keyerControl contains processing flags and keyer mode bits, Supports Iambic A and B, State machine based, uses calls to millis() for timing.
    case IDLE: // Wait for direct or latched paddle press
      if ((_digitalRead(DAH) == LOW) ||
          (_digitalRead(DIT) == LOW) ||
          (keyerControl & 0x03))
      {
#ifdef CW_MESSAGE
        cw_msg_event = 0; // clear cw message event
#endif                    //CW_MESSAGE
        update_PaddleLatch();
        keyerState = CHK_DIT;
      }
      break;
    case CHK_DIT: // See if the dit paddle was pressed
      if (keyerControl & DIT_L)
      {
        keyerControl |= DIT_PROC;
        ktimer = ditTime;
        keyerState = KEYED_PREP;
      }
      else
      {
        keyerState = CHK_DAH;
      }
      break;
    case CHK_DAH: // See if dah paddle was pressed
      if (keyerControl & DAH_L)
      {
        ktimer = ditTime * 3;
        keyerState = KEYED_PREP;
      }
      else
      {
        keyerState = IDLE;
      }
      break;
    case KEYED_PREP: // Assert key down, start timing, state shared for dit or dah
      Key_state = HIGH;
      switch_rxtx(Key_state);
      ktimer += millis();               // set ktimer to interval end time
      keyerControl &= ~(DIT_L + DAH_L); // clear both paddle latch bits
      keyerState = KEYED;               // next state
      break;
    case KEYED: // Wait for timer to expire
      if (millis() > ktimer)
      { // are we at end of key down ?
        Key_state = LOW;
        switch_rxtx(Key_state);
        ktimer = millis() + ditTime; // inter-element time
        keyerState = INTER_ELEMENT;  // next state
      }
      else if (keyerControl & IAMBICB)
      {
        update_PaddleLatch(); // early paddle latch in Iambic B mode
      }
      break;
    case INTER_ELEMENT:
      // Insert time between dits/dahs
      update_PaddleLatch(); // latch paddle state
      if (millis() > ktimer)
      { // are we at end of inter-space ?
        if (keyerControl & DIT_PROC)
        {                                      // was it a dit or dah ?
          keyerControl &= ~(DIT_L + DIT_PROC); // clear two bits
          keyerState = CHK_DAH;                // dit done, check for dah
        }
        else
        {
          keyerControl &= ~(DAH_L); // clear dah latch
          keyerState = IDLE;        // go idle
        }
      }
      break;
    }
  }
  else
  {
#endif //KEYER

#ifdef TX_ENABLE
    uint8_t pin = ((mode == CW) && (keyer_swap)) ? DAH : DIT;
    if (!vox_tx) //  ONLY if VOX not active, then check DIT/DAH (fix for VOX to prevent RFI feedback through EMI on DIT or DAH line)
      if (!_digitalRead(pin))
      { // PTT/DIT keys transmitter
#ifdef CW_MESSAGE
        cw_msg_event = 0; // clear cw message event
#endif                    //CW_MESSAGE
        switch_rxtx(1);
        do
        {
          wdt_reset();
          delay((mode == CW) ? 10 : 100); // keep the tx keyed for a while before sensing (helps against RFI issues on DAH/DAH line)
#ifdef SWR_METER
          if (smeter > 0 && mode == CW && millis() >= stimer)
          {
            readSWR();
            stimer = millis() + 500;
          }
#endif
          if (inv ^ _digitalRead(BUTTONS))
            break;                    // break if button is pressed (to prevent potential lock-up)
        } while (!_digitalRead(pin)); // until released
        switch_rxtx(0);
      }
#endif //TX_ENABLE
#ifdef KEYER
  }
#endif //KEYER

#ifdef SEMI_QSK
  if ((semi_qsk_timeout) && (millis() > semi_qsk_timeout))
  {
    switch_rxtx(0);
  } // delayed QSK RX
#endif

  event = getEvent(event);

   switch (event)
    {
#ifndef ONEBUTTON
    case BL | PL:  // Called when menu button pressed
    case BL | PLC: // or kept pressed
      menumode = 2;
      break;
    case BL | PT:
      menumode = 1;
      //if(menu == 0) menu = 1;
      break;
    case BL | SC:
#ifdef CW_MESSAGE
      if ((menumode == 1) && (menu >= CWMSG1) && (menu <= CWMSG6))
      {
        cw_msg_event = millis();
        cw_msg_id = menu - CWMSG1;
        menumode = 0;
        break;
      }
#endif //CW_MESSAGE
      int8_t _menumode;
      if (menumode == 0)
      {
        _menumode = 1;
        if (menu == 0)
          menu = 1;
      } // short left-click while in default screen: enter menu mode
      if (menumode == 1)
      {
        _menumode = 2;
      } // short left-click while in menu: enter value selection screen
      if (menumode >= 2)
      {
        _menumode = 0;
        paramAction(SAVE, menu);
      } // short left-click while in value selection screen: save, and return to default screen
      menumode = _menumode;
      break;
    case BL | DC:
      break;
    case BR | SC:
      if (!menumode)
      {
        int8_t prev_mode = mode;
        if (rit)
        {
          rit = 0;
          stepsize = prev_stepsize[mode == CW];
          change = true;
          break;
        }
        mode += 1;
        //encoder_val = 1;
        //paramAction(UPDATE, MODE); // Mode param //paramAction(UPDATE, mode, NULL, F("Mode"), mode_label, 0, _N(mode_label), true);
//#define MODE_CHANGE_RESETS  1
#ifdef MODE_CHANGE_RESETS
        if (mode != CW)
          stepsize = STEP_1k;
        else
          stepsize = STEP_500; // sets suitable stepsize
#endif
        if (mode > CW)
          mode = LSB; // skip all other modes (only LSB, USB, CW)
#ifdef MODE_CHANGE_RESETS
        if (mode == CW)
        {
          filt = 4;
          nr = 0;
        }
        else
          filt = 0; // resets filter (to most BW) and NR on mode change
#else
        if (mode == CW)
        {
          nr = 0;
        }
        prev_stepsize[prev_mode == CW] = stepsize;
        stepsize = prev_stepsize[mode == CW]; // backup stepsize setting for previous mode, restore previous stepsize setting for current selected mode; filter settings captured for either CQ or other modes.
        prev_filt[prev_mode == CW] = filt;
        filt = prev_filt[mode == CW]; // backup filter setting for previous mode, restore previous filter setting for current selected mode; filter settings captured for either CQ or other modes.
#endif
        //paramAction(UPDATE, MODE);
        vfomode[vfosel % 2] = mode;
        paramAction(SAVE, (vfosel % 2) ? MODEB : MODEA); // save vfoa/b changes
        paramAction(SAVE, MODE);
        paramAction(SAVE, FILTER);
        si5351.iqmsa = 0; // enforce PLL reset
#ifdef CW_DECODER
        if ((prev_mode == CW) && (cwdec))
          show_banner();
#endif
        change = true;
      }
      else
      {
        if (menumode == 1)
        {
          menumode = 0;
        } // short right-click while in menu: enter value selection screen
        if (menumode >= 2)
        {
          menumode = 1;
          change = true;
          paramAction(SAVE, menu);
        } // short right-click while in value selection screen: save, and return to menu screen
      }
      break;
    case BR | DC:
      filt++;
      _init = true;
      if (mode == CW && filt > N_FILT)
        filt = 4;
      if (mode == CW && filt == 4)
        stepsize = STEP_500; // reset stepsize for 500Hz filter
      if (mode == CW && (filt == 5 || filt == 6) && stepsize < STEP_100)
        stepsize = STEP_100; // for CW BW 200, 100      -> step = 100 Hz
      if (mode == CW && filt == 7 && stepsize < STEP_10)
        stepsize = STEP_10; // for CW BW 50 -> step = 10 Hz
      if (mode != CW && filt > 3)
        filt = 0;
      encoder_val = 0;
      paramAction(UPDATE, FILTER);
      paramAction(SAVE, FILTER);
      wdt_reset();
      delay(1500);
      wdt_reset();
      change = true; // refresh display
      break;
    case BR | PL:
#ifdef SIMPLE_RX
      // Experiment: ISR-less sdr_rx():
      smode = 0;
      TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
      delay(100);
      lcd.setCursor(15, 1);
      lcd.print('X');
      static uint8_t x = 0;
      uint32_t next = 0;
      for (;;)
      {
        func_ptr();
#ifdef DEBUG
        numSamples++;
#endif
        if (!rx_state)
        {
          x++;
          if (x > 16)
          {
            loop();
            //lcd.setCursor(9, 0); lcd.print((int16_t)100); lcd.print(F("dBm   "));  // delays are taking too long!
            x = 0;
          }
        }
        //for(;micros() < next;);  next = micros() + 16;   // sync every 1000000/62500=16ms (or later if missed)
      } //
#endif  //SIMPLE_RX
#ifdef RIT_ENABLE
      rit = !rit;
      stepsize = (rit) ? STEP_10 : prev_stepsize[mode == CW];
      if (!rit)
      { // after RIT comes VFO A/B swap
#else
    {
#endif //RIT_ENABLE
        vfosel = !vfosel;
        freq = vfo[vfosel % 2]; // todo: share code with menumode
        mode = vfomode[vfosel % 2];
        // make more generic:
        if (mode != CW)
          stepsize = STEP_1k;
        else
          stepsize = STEP_500;
        if (mode == CW)
        {
          filt = 4;
          nr = 0;
        }
        else
          filt = 0;
      }
      change = true;
      break;
//#define TUNING_DIAL  1
#ifdef TUNING_DIAL
    case BR | PLC: // while pressed long continues
    case BE | PLC:
      freq = freq + ((_step > 0) ? 1 : -1) * pow(2, abs(_step));
      change = true;
      break;
    case BR | PT:
      _step += encoder_val;
      encoder_val = 0;
      lcd.setCursor(0, 0);
      lcd.print(_step);
      lcd_blanks();
      break;
#endif //TUNING_DIAL
    case BE | SC:
      if (!menumode)
      {
        stepsize_change(+1);
      }
      else
      {
        int8_t _menumode;
        if (menumode == 1)
        {
          _menumode = 2;
        } // short encoder-click while in menu: enter value selection screen
        if (menumode == 2)
        {
          _menumode = 1;
          change = true;
          paramAction(SAVE, menu);
        } // short encoder-click while in value selection screen: save, and return to menu screen
#ifdef MENU_STR
        if (menumode == 3)
        {
          _menumode = 3;
          paramAction(NEXT_CH, menu);
        } // short encoder-click while in string edit mode: change position to next character
#endif
        menumode = _menumode;
      }
      break;
    case BE | DC:
      //delay(100);
      bandval++;
      //if(bandval >= N_BANDS) bandval = 0;
      if (bandval >= (N_BANDS - 1))
        bandval = 1; // excludes 6m, 160m
      stepsize = STEP_1k;
      change = true;
      break;
    case BE | PL:
      stepsize_change(-1);
      break;
    case BE | PT:
      for (; _digitalRead(BUTTONS);)
      { // process encoder changes until released
        wdt_reset();
        if (encoder_val)
        {
          paramAction(UPDATE, VOLUME);
          if (volume < 0)
          {
            volume = 10;
            paramAction(SAVE, VOLUME);
            powerDown();
          } // powerDown when volume < 0
          paramAction(SAVE, VOLUME);
        }
      }
      change = true; // refresh display
      break;
#else //ONEBUTTON
    case BE | SC:
      int8_t _menumode;
      if (menumode == 0)
      {
        _menumode = 1;
        if (menu == 0)
          menu = 1;
      } // short enc-click while in default screen: enter menu mode
      if (menumode == 1)
      {
        _menumode = 2;
      } // short enc-click while in menu: enter value selection screen
      if (menumode == 2)
      {
        _menumode = 0;
        paramAction(SAVE, menu);
      } // short enc-click while in value selection screen: save, and return to default screen
#ifdef MENU_STR
      if (menumode == 3)
      {
        _menumode = 3;
        paramAction(NEXT_CH, menu);
      } // short encoder-click while in string edit mode: change position to next character
#endif
      menumode = _menumode;
      break;
    case BE | DC:
      if (!menumode)
      {
        int8_t prev_mode = mode;
        if (rit)
        {
          rit = 0;
          stepsize = prev_stepsize[mode == CW];
          change = true;
          break;
        }
        mode += 1;
        //encoder_val = 1;
        //paramAction(UPDATE, MODE); // Mode param //paramAction(UPDATE, mode, NULL, F("Mode"), mode_label, 0, _N(mode_label), true);
//#define MODE_CHANGE_RESETS  1
#ifdef MODE_CHANGE_RESETS
        if (mode != CW)
          stepsize = STEP_1k;
        else
          stepsize = STEP_500; // sets suitable stepsize
#endif
        if (mode > CW)
          mode = LSB; // skip all other modes (only LSB, USB, CW)
#ifdef MODE_CHANGE_RESETS
        if (mode == CW)
        {
          filt = 4;
          nr = 0;
        }
        else
          filt = 0; // resets filter (to most BW) and NR on mode change
#else
        if (mode == CW)
        {
          nr = 0;
        }
        prev_stepsize[prev_mode == CW] = stepsize;
        stepsize = prev_stepsize[mode == CW]; // backup stepsize setting for previous mode, restore previous stepsize setting for current selected mode; filter settings captured for either CQ or other modes.
        prev_filt[prev_mode == CW] = filt;
        filt = prev_filt[mode == CW]; // backup filter setting for previous mode, restore previous filter setting for current selected mode; filter settings captured for either CQ or other modes.
#endif
        //paramAction(UPDATE, MODE);
        vfomode[vfosel % 2] = mode;
        paramAction(SAVE, (vfosel % 2) ? MODEB : MODEA); // save vfoa/b changes
        paramAction(SAVE, MODE);
        paramAction(SAVE, FILTER);
        si5351.iqmsa = 0; // enforce PLL reset
        if ((prev_mode == CW) && (cwdec))
          show_banner();
        change = true;
      }
      else
      {
        if (menumode == 1)
        {
          menumode = 0;
        } // short right-click while in menu: enter value selection screen
        if (menumode >= 2)
        {
          menumode = 1;
          change = true;
          paramAction(SAVE, menu);
        } // short right-click while in value selection screen: save, and return to menu screen
      }
      break;
    case BE | PL:
      stepsize += 1;
      if (stepsize < STEP_1k)
        stepsize = STEP_10;
      if (stepsize > STEP_10)
        stepsize = STEP_1k;
      stepsize_showcursor();
      break;
    case BE | PLC: // or kept pressed
      menumode = 2;
      break;
    case BE | PT:
      menumode = 1;
      //if(menu == 0) menu = 1;
      break;
    case BL | SC:
    case BL | DC:
    case BL | PL:
    case BL | PLC:
      encoder_val++;
      break;
    case BR | SC:
    case BR | DC:
    case BR | PL:
    case BR | PLC:
      encoder_val--;
      break;
#endif //ONEBUTTON
    }
  

  if ((menumode) || (prev_menumode != menumode))
  { // Show parameter and value
    int8_t encoder_change = encoder_val;
    if ((menumode == 1) && encoder_change)
    {
      menu += encoder_val; // Navigate through menu
#ifdef ONEBUTTON
      menu = max(0, min(menu, N_PARAMS));
#else
      menu = max(1 /* 0 */, min(menu, N_PARAMS));
#endif
      menu = paramAction(NEXT_MENU, menu); // auto probe next menu item (gaps may exist)
      encoder_val = 0;
    }
    if (encoder_change || (prev_menumode != menumode))
      paramAction(UPDATE_MENU, (menumode) ? menu : 0); // update param with encoder change and display
    prev_menumode = menumode;
    if (menumode == 2)
    {
      if (encoder_change)
      {
        lcd.setCursor(0, 1);
        lcd.cursor(); // edits menu item value; make cursor visible
        if (menu == MODE)
        { // post-handling Mode parameter
          vfomode[vfosel % 2] = mode;
          paramAction(SAVE, (vfosel % 2) ? MODEB : MODEA); // save vfoa/b changes
          change = true;
          si5351.iqmsa = 0; // enforce PLL reset
          // make more generic:
          if (mode != CW)
            stepsize = STEP_1k;
          else
            stepsize = STEP_500;
          if (mode == CW)
          {
            filt = 4;
            nr = 0;
          }
          else
            filt = 0;
        }
        if (menu == BAND)
        {
          change = true;
        }
        //if(menu == NR){ if(mode == CW) nr = false; }
        if (menu == VFOSEL)
        {
          freq = vfo[vfosel % 2];
          mode = vfomode[vfosel % 2];
          // make more generic:
          if (mode != CW)
            stepsize = STEP_1k;
          else
            stepsize = STEP_500;
          if (mode == CW)
          {
            filt = 4;
            nr = 0;
          }
          else
            filt = 0;
          change = true;
        }
#ifdef RIT_ENABLE
        if (menu == RIT)
        {
          stepsize = (rit) ? STEP_10 : STEP_500;
          change = true;
        }
#endif
        //if(menu == VOX){ if(vox){ vox_thresh-=1; } else { vox_thresh+=1; }; }
        if (menu == ATT)
        { // post-handling ATT parameter
          if (dsp_cap == SDR)
          {
            noInterrupts();
#ifdef SWAP_RX_IQ
            adc_start(1, !(att & 0x01) /*true*/, F_ADC_CONV);
            admux[0] = ADMUX;
            adc_start(0, !(att & 0x01) /*true*/, F_ADC_CONV);
            admux[1] = ADMUX;
#else
            adc_start(0, !(att & 0x01) /*true*/, F_ADC_CONV);
            admux[0] = ADMUX;
            adc_start(1, !(att & 0x01) /*true*/, F_ADC_CONV);
            admux[1] = ADMUX;
#endif //SWAP_RX_IQ
            interrupts();
          }
          digitalWrite(RX, !(att & 0x02));                // att bit 1 ON: attenuate -20dB by disabling RX line, switching Q5 (antenna input switch) into 100k resistence
          pinMode(AUDIO1, (att & 0x04) ? OUTPUT : INPUT); // att bit 2 ON: attenuate -40dB by terminating ADC inputs with 10R
          pinMode(AUDIO2, (att & 0x04) ? OUTPUT : INPUT);
        }
        if (menu == SIFXTAL)
        {
          change = true;
        }
#ifdef TX_ENABLE
        if ((menu == PWM_MIN) || (menu == PWM_MAX))
        {
          build_lut();
        }
#endif
        if (menu == CWTONE)
        {
          if (dsp_cap)
          {
            cw_offset = (cw_tone == 0) ? tones[0] : tones[1];
            paramAction(SAVE, CWOFF);
          }
        }
        if (menu == IQ_ADJ)
        {
          change = true;
        }
#ifdef CAL_IQ
        if (menu == CALIB)
        {
          if (dsp_cap != SDR)
            calibrate_iq();
          menu = 0;
        }
#endif
#ifdef KEYER
        if (menu == KEY_WPM)
        {
          loadWPM(keyer_speed);
        }
        if (menu == KEY_MODE)
        {
          if (keyer_mode == 0)
          {
            keyerControl = IAMBICA;
          }
          if (keyer_mode == 1)
          {
            keyerControl = IAMBICB;
          }
          if (keyer_mode == 2)
          {
            keyerControl = SINGLE;
          }
        }
#endif //KEYER
#ifdef TX_DELAY
        if (menu == TXDELAY)
        {
          semi_qsk = (txdelay > 0);
        }
#endif //TX_DELAY
      }
#ifdef DEBUG
      if (menu == SR)
      { // measure sample-rate
        numSamples = 0;
        delay(F_MCU * 500 / 16000000);  // delay 0.5s (in reality because F_CPU=20M instead of 16M, delay() is running 1.25x faster therefore we need to multiply with 1.25)
        sr = numSamples * 2;            // samples per second
        paramAction(UPDATE_MENU, menu); // refresh
      }
      if (menu == CPULOAD)
      { // measure CPU-load
        uint32_t i = 0;
        uint32_t prev_time = millis();
        for (i = 0; i != 300000; i++)
          wdt_reset(); // fixed CPU-load 132052*1.25us delay under 0% load condition; is 132052*1.25 * 20M = 3301300 CPU cycles fixed load
        cpu_load = 100 - 132 * 100 / (millis() - prev_time);
        paramAction(UPDATE_MENU, menu); // refresh
      }
      if ((menu == PARAM_A) || (menu == PARAM_B) || (menu == PARAM_C))
      {
        delay(300);
        paramAction(UPDATE_MENU, menu); // refresh
      }
#endif
    }
  }

  if (menumode == 0)
  {
    if (encoder_val)
    { // process encoder tuning steps
      process_encoder_tuning_step(encoder_val);
      encoder_val = 0;
    }
  }

  if ((change) && (!tx) && (!vox_tx))
  { // only change if TX is OFF, prevent simultaneous I2C bus access
    change = false;
    if (prev_bandval != bandval)
    {
      freq = band[bandval];
      prev_bandval = bandval;
    }
    vfo[vfosel % 2] = freq;
    save_event_time = millis() + 1000; // schedule time to save freq (no save while tuning, hence no EEPROM wear out)

    if (menumode == 0)
    {
      display_vfo(freq);
      stepsize_showcursor();
#ifdef CAT
      //Command_GETFreqA();
#endif

      // The following is a hack for SWR measurement:
      //si5351.alt_clk2(freq + 2400);
      //si5351.SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=1, CLK1_EN,CLK0_EN=1
      //digitalWrite(SIG_OUT, HIGH);  // inject CLK2 on antenna input via 120K
    }

    uint8_t f = freq / 1000000UL;
    set_lpf(f);
    bandval = (f > 32) ? 10 : (f > 26) ? 9
                          : (f > 22)   ? 8
                          : (f > 20)   ? 7
                          : (f > 16)   ? 6
                          : (f > 12)   ? 5
                          : (f > 8)    ? 4
                          : (f > 6)    ? 3
                          : (f > 4)    ? 2
                          : (f > 2)    ? 1
                                       : 0;
    prev_bandval = bandval; // align bandval with freq

    //noInterrupts();
    if (mode == CW)
    {
      si5351.freq(freq + cw_offset, rx_ph_q, 0 /*90, 0*/); // RX in CW-R (=LSB), correct for CW-tone offset
    }
    else if (mode == LSB)
      si5351.freq(freq, rx_ph_q, 0 /*90, 0*/); // RX in LSB
    else
      si5351.freq(freq, 0, rx_ph_q /*0, 90*/); // RX in USB, ...
#ifdef RIT_ENABLE
    if (rit)
    {
      si5351.freq_calc_fast(rit);
      si5351.SendPLLRegisterBulk();
    }
#endif //RIT_ENABLE
    //interrupts();
  }

  if ((save_event_time) && (millis() > save_event_time))
  {                                                  // save freq when time has reached schedule
    paramAction(SAVE, (vfosel % 2) ? FREQB : FREQA); // save vfoa/b changes
    save_event_time = 0;
    //lcd.setCursor(15, 1); lcd.print('S'); delay(100); lcd.setCursor(15, 1); lcd.print('R');
  }

#ifdef CW_MESSAGE
  if ((mode == CW) && (cw_msg_event) && (millis() > cw_msg_event))
  { // if it is time to send a CW message
    if ((cw_tx(cw_msg[cw_msg_id]) == 0) && ((cw_msg[cw_msg_id][0] == 'C') && (cw_msg[cw_msg_id][1] == 'Q')) && cw_msg_interval)
      cw_msg_event = millis() + 1000 * cw_msg_interval;
    else
      cw_msg_event = 0; // then send message, if not interrupted and its a CQ msg and there is an interval set, then schedule new event
  }
#endif //CW_MESSAGE

  wdt_reset();

  //{ lcd.setCursor(0, 0); lcd.print(freeMemory()); lcd.print(F("    ")); }
}

/* BACKLOG:
code definitions and re-use for comb, integrator, dc decoupling, arctan
refactor main()
agc based on rms256, agc/smeter after filter
noisefree integrator (rx audio out) in lower range
raised cosine tx amp for cw, 4ms tau seems enough: http://fermi.la.asu.edu/w9cf/articles/click/index.html
cw tx message/cw encoder
32 bin fft
dynamic range cw
att extended agc
single ATT
configurable F_CPU
CW-L mode
Split
undersampling, IF-offset
K2/TS480 CAT control
faster RX-TX switch to support CW
clock
qcx API demo code
scan
move last bit of arrays into flash? https://web.archive.org/web/20180324010832/https://www.microchip.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_rom_array.html
remove floats
u-law in RX path?: http://dystopiancode.blogspot.com/2012/02/pcm-law-and-u-law-companding-algorithms.html
Arduino library?
1. RX bias offset correction by measurement avg, 2. charge decoupling cap. by resetting to 0V and setting 5V for a certain amount of (charge) time
add 1K (500R?) to GND at TTL RF output to keep zero-level below BS170 threshold
additional PWM output for potential BOOST conversion
SWR measurement?
CW decoder amp thrshld restriction and noise reduction (use of certain pause amounts)
squelch gating
more buttons
s-meter offset vs DC bal.
keyer with interrupt-driven timers (to reduce jitter)

Analyse assembly:
/home/guido/Downloads/arduino-1.8.10/hardware/tools/avr/bin/avr-g++ -S -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10810 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/home/guido/Downloads/arduino-1.8.10/hardware/arduino/avr/cores/arduino -I/home/guido/Downloads/arduino-1.8.10/hardware/arduino/avr/variants/standard /tmp/arduino_build_483134/sketch/QCX-SSB.ino.cpp -o /tmp/arduino_build_483134/sketch/QCX-SSB.ino.cpp.txt

Rewire/code I/Q clk pins so that a Div/1 and Div/2 scheme is used instead of 0 and 90 degrees phase shift
10,11,13,12   10,11,12,13  (pin)
Q- I+ Q+ I-   Q- I+ Q+ I-
90 deg.shift  div/2@S1(pin2)

50MHz LSB OK, USB NOK

atmega328p signature: https://forum.arduino.cc/index.php?topic=341799.15   https://www.eevblog.com/forum/microcontrollers/bootloader-on-smd-atmega328p-au/msg268938/#msg268938 https://www.avrfreaks.net/forum/undocumented-signature-row-contents

Alain k1fm AGC sens issue:  https://groups.io/g/ucx/message/3998   https://groups.io/g/ucx/message/3999
txdelay when vox is on (disregading the tx>0 state due to ssb() overrule, instead use RX-digitalinput)
Adrian: issue #41, set cursor just after writing 'R' when smeter is off, and (menumode == 0)
Konstantinos: backup/restore vfofilt settings when changing vfo.
Bob: 2mA for clk0/1 during RX
Uli: accuracate voltages during diag

agc behind filter
vcc adc extend. power/curr measurement
swr predistort eff calc
block ptt while in vox mode

adc bias error and potential error correction
noise burst on tx
https://groups.io/g/ucx/topic/81030243#6265

*/
