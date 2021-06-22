#pragma once
#include <Arduino.h>
#include "custom_math.h"
#include "Si5351.h"

volatile uint8_t tx = 0;
volatile uint8_t filt = 0;

#define MAX_DP ((filt == 0) ? _UA : (filt == 3) ? _UA / 4 \
                                                : _UA / 2) //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define CARRIER_COMPLETELY_OFF_ON_LOW 1                    // disable oscillator on low amplitudes, to prevent potential unwanted biasing/leakage through PA circuit
#define MULTI_ADC 1                                        // multiple ADC conversions for more sensitive (+12dB) microphone input
//#define QUAD  1       // invert TX signal for phase changes > 180

uint8_t lut[256];
volatile uint8_t amp;
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);

#ifdef MORE_MIC_GAIN
  volatile uint8_t drive  = 6; // Init settings // hmm.. drive>2 impacts cpu load..why?
#else
  volatile uint8_t drive = 4;                                                                                                                                                             // Init settings
#endif




volatile uint8_t quad = 0;


inline void _vox(bool trigger)
{
  if (trigger)
  {
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  }
  else
  {
    if (tx)
      tx--;
  }
}


inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];

  for (j = 0; j != 15; j++)
    v[j] = v[j + 1];

  //dc += (in - dc) / 2;       // fast moving average
  dc = (in + dc) / 2;     // average
  int16_t ac = (in - dc); // DC decoupling
  //v[15] = ac;// - z1;        // high-pass (emphasis) filter

#ifdef MORE_MIC_GAIN
  v[15] = (ac + z1) << 1; // low-pass filter with notch at Fs/2
  z1 = ac;

  i = v[7] << 1;
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i >> 2, q >> 2);
#else  // MORE_MIC_GAIN
  v[15] = (ac + z1); // / 2;           // low-pass filter with notch at Fs/2
  z1 = ac;

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
#endif // !MORE_MIC_GAIN

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if (vox)
    _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase; // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if (dp < 0)
    dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef QUAD
  if (dp >= (_UA / 2))
  {
    dp = dp - _UA / 2;
    quad = !quad;
  }
#endif

#ifdef MAX_DP
  if (dp > MAX_DP)
  {                                     // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP); // substract restdp
    dp = MAX_DP;
  }
#endif
  if (mode == USB)
    return dp * (_F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-_F_SAMP_TX / _UA);
}

#define MIC_ATTEN 0 // 0*6dB attenuation (note that the LSB bits are quite noisy)
volatile int8_t mox = 0;
volatile int8_t volume = 12;

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx()
{                // jitter dependent things first
#ifdef MULTI_ADC // SSB with multiple ADC conversions:
  int16_t adc;   // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk(); // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
#ifdef QUAD
#ifdef TX_CLK0_CLK1
  si5351.SendRegister(16, (quad) ? 0x1f : 0x0f); // Invert/non-invert CLK0 in case of a huge phase-change
  si5351.SendRegister(17, (quad) ? 0x1f : 0x0f); // Invert/non-invert CLK1 in case of a huge phase-change
#else
  si5351.SendRegister(18, (quad) ? 0x1f : 0x0f); // Invert/non-invert CLK2 in case of a huge phase-change
#endif
#endif          //QUAD
  OCR1BL = amp; // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  adc += ADC;
  ADCSRA |= (1 << ADSC);               // causes RFI on QCX-SSB units (not on units with direct biasing); ENABLE this line when using direct biasing!!
  int16_t df = ssb(_adc >> MIC_ATTEN); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  si5351.freq_calc_fast(df); // calculate SI5351 registers based on frequency shift and carrier frequency
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  //_adc = (adc/4 - 512);
#define AF_BIAS 32
  _adc = (adc / 4 - (512 - AF_BIAS)); // now make sure that we keep a postive bias offset (to prevent the phase swapping 180 degrees and potentially causing negative feedback (RFI)
#else                                 // SSB with single ADC conversion:
  ADCSRA |= (1 << ADSC); // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                       // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ADC - 512;            // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);          // calculate SI5351 registers based on frequency shift and carrier frequency
#endif

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  if (tx == 1)
  {
    OCR1BL = 0;
    si5351.SendRegister(SI_CLK_OE, 0b11111111);
  } // disable carrier
#ifdef TX_CLK0_CLK1
  if (tx == 255)
  {
    si5351.SendRegister(SI_CLK_OE, 0b11111100);
  }    // enable carrier
#else  //TX_CLK2
  if (tx == 255)
  {
    si5351.SendRegister(SI_CLK_OE, 0b11111011);
  } // enable carrier
#endif //TX_CLK0_CLK1
#endif

#ifdef MOX_ENABLE
  if (!mox)
    return;
  OCR1AL = (adc << (mox - 1)) + 128; // TX audio monitoring
#endif
}

volatile uint16_t acc;
volatile uint32_t cw_offset;
volatile uint8_t cw_tone = 1;
const uint32_t tones[] = {F_MCU * 700ULL / 20000000, F_MCU * 600ULL / 20000000, F_MCU * 700ULL / 20000000};

volatile int8_t p_sin = 0;       // initialized with A*sin(0) = 0
volatile int8_t n_cos = 448 / 4; // initialized with A*cos(t) = A
inline void process_minsky()     // Minsky circle sample [source: https://www.cl.cam.ac.uk/~am21/hakmemc.html, ITEM 149]: p_sin+=n_cos*2*PI*f/fs; n_cos-=p_sin*2*PI*f/fs;
{
  int8_t alpha127 = tones[cw_tone] /*cw_offset*/ * 798 / _F_SAMP_TX; // alpha = f_tone * 2 * pi / fs
  p_sin += alpha127 * n_cos / 127;
  n_cos -= alpha127 * p_sin / 127;
}

// CW Key-click shaping, ramping up/down amplitude with sample-interval of 60us. Tnx: Yves HB9EWY https://groups.io/g/ucx/message/5107
const uint8_t ramp[] PROGMEM = {255, 254, 252, 249, 245, 239, 233, 226, 217, 208, 198, 187, 176, 164, 152, 139, 127, 115, 102, 90, 78, 67, 56, 46, 37, 28, 21, 15, 9, 5, 2}; // raised-cosine(i) = 255 * sq(cos(HALF_PI * i/32))

void dummy()
{
}

void dsp_tx_cw()
{ // jitter dependent things first
#ifdef KEY_CLICK
  if (OCR1BL < lut[255])
  { //check if already ramped up: ramp up of amplitude
    for (uint16_t i = 31; i != 0; i--)
    { // soft rising slope against key-clicks
      OCR1BL = lut[pgm_read_byte_near(ramp[i])];
      delayMicroseconds(60);
    }
  }
#endif // KEY_CLICK
  OCR1BL = lut[255];

  process_minsky();
  OCR1AL = (p_sin >> (16 - volume)) + 128;
}

void dsp_tx_am()
{                          // jitter dependent things first
  ADCSRA |= (1 << ADSC);   // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = amp;            // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive - 4);
//static int16_t dc;
//dc += (in - dc) / 2;
//in = in - dc;     // DC decoupling
#define AM_BASE 32
  in = max(0, min(255, (in + AM_BASE)));
  amp = in; // lut[in];
}

void dsp_tx_fm()
{                               // jitter dependent things first
  ADCSRA |= (1 << ADSC);        // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = lut[255];            // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk(); // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  int16_t adc = ADC - 512;      // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  int16_t df = in;
  si5351.freq_calc_fast(df); // calculate SI5351 registers based on frequency shift and carrier frequency
}

#define EA(y, x, one_over_alpha) (y) = (y) + ((x) - (y)) / (one_over_alpha);        // exponental averaging [Lyons 13.33.1]
#define MLEA(y, x, L, M) (y) = (y) + ((((x) - (y)) >> (L)) - (((x) - (y)) >> (M))); // multiplierless exponental averaging [Lyons 13.33.1], with alpha=1/2^L - 1/2^M


//refresh LUT based on pwm_min, pwm_max
void build_lut()
{
  for (uint16_t i = 0; i != 256; i++) // refresh LUT based on pwm_min, pwm_max
    lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
  //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
}