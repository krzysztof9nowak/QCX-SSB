#include <Arduino.h>
#include "config.h"

volatile uint8_t cat_streaming = 0;
volatile uint8_t _cat_streaming = 0;

typedef void (*func_t)(void);
volatile func_t func_ptr;
#undef  R  // Decimating 2nd Order CIC filter
#define R 4  // Rate change from 62500/2 kSPS to 7812.5SPS, providing 12dB gain

//#define SIMPLE_RX  1
#ifndef SIMPLE_RX
volatile uint8_t admux[3];
volatile int16_t ocomb, qh;
volatile uint8_t rx_state = 0;

#pragma GCC push_options
#pragma GCC optimize ("Ofast")  // compiler-optimization for speed



#define NEW_RX  1   // Faster (3rd-order) CIC stage, with simultanuous processing capability
#ifdef NEW_RX
#define AF_OUT  1   // Enables audio output stage (can be disabled in conjunction with CAT_STREAMING to safe memory)

static uint8_t tc = 0;
void process(int16_t i_ac2, int16_t q_ac2)
{
  static int16_t ac3;
#ifdef CAT_STREAMING
  //UCSR0B &= ~(TXCIE0);  // disable USART TX interrupts
  //while (!( UCSR0A & (1<<UDRE0)));  // wait for empty buffer
  if(cat_streaming){ uint8_t out = ac3 + 128; if(out == ';') out++; Serial.write(out); }  //UDR0 = (uint8_t)(ac3 + 128);   // from:  https://www.xanthium.in/how-to-avr-atmega328p-microcontroller-usart-uart-embedded-programming-avrgcc
#endif // CAT_STREAMING
#ifdef AF_OUT
  static int16_t ozd1, ozd2;  // Output stage
  if(_init){ ac3 = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
  int16_t od1 = ac3 - ozd1; // Comb section
  ocomb = od1 - ozd2;
#endif //AF_OUT
#define OUTLET  1
#ifdef OUTLET
  if(tc++ == 0)   // prevent recursion
  //if(tc++ > 16)   // prevent recursion
#endif
  interrupts();  // hack, since slow_dsp process exceeds rx sample-time, allow subsequent 7 interrupts for further rx sampling while processing, prevent nested interrupts with tc
#ifdef AF_OUT
  ozd2 = od1;
  ozd1 = ac3;
#endif  //AF_OUT
  int16_t qh;
  {
    q_ac2 >>= att2;  // digital gain control
    static int16_t v[14];  // Process Q (down-sampled) samples
    // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0, 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0, -79, 0, -21, 0, -8, 0, -2, 0) / 128;
    qh = ((v[0] - q_ac2) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS) when used in image-rejection scenario; (Hilbert transform require 4 additional bits)
    //qh = ((v[0] - q_ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
    v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7]; v[7] = v[8]; v[8] = v[9]; v[9] = v[10]; v[10] = v[11]; v[11] = v[12]; v[12] = v[13]; v[13] = q_ac2;
  }
  i_ac2 >>= att2;  // digital gain control
  static int16_t v[7];  // Post processing I and Q (down-sampled) results
  i = i_ac2; q = q_ac2;   // tbd: this can be more efficient
  int16_t i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = i_ac2;  // Delay to match Hilbert transform on Q branch
  ac3 = slow_dsp(-i - qh);  //inverting I and Q helps dampening a feedback-loop between PWM out and ADC inputs
#ifdef OUTLET
  tc--;
#endif
}

/*
// # M=3 .. = i0 + 3*(i2 + i3) + i1
int16_t i0, i1, i2, i3, i4, i5, i6, i7, i8;
int16_t q0, q1, q2, q3, q4, q5, q6, q7, q8;
#define M_SR  1

//#define EXPANDED_CIC
#ifdef EXPANDED_CIC
void sdr_rx_00(){         i0 = sdr_rx_common_i(); func_ptr = sdr_rx_01;   i4 = (i0 + (i2 + i3) * 3 + i1) >> M_SR; }
void sdr_rx_02(){         i1 = sdr_rx_common_i(); func_ptr = sdr_rx_03;   i8 = (i4 + (i6 + i7) * 3 + i5) >> M_SR; }
void sdr_rx_04(){         i2 = sdr_rx_common_i(); func_ptr = sdr_rx_05;   i5 = (i2 + (i0 + i1) * 3 + i3) >> M_SR; }
void sdr_rx_06(){         i3 = sdr_rx_common_i(); func_ptr = sdr_rx_07; }
void sdr_rx_08(){         i0 = sdr_rx_common_i(); func_ptr = sdr_rx_09;   i6 = (i0 + (i2 + i3) * 3 + i1) >> M_SR; }
void sdr_rx_10(){         i1 = sdr_rx_common_i(); func_ptr = sdr_rx_11;   i8 = (i6 + (i4 + i5) * 3 + i7) >> M_SR; }
void sdr_rx_12(){         i2 = sdr_rx_common_i(); func_ptr = sdr_rx_13;   i7 = (i2 + (i0 + i1) * 3 + i3) >> M_SR; }
void sdr_rx_14(){         i3 = sdr_rx_common_i(); func_ptr = sdr_rx_15; }
void sdr_rx_15(){         q0 = sdr_rx_common_q(); func_ptr = sdr_rx_00;   q4 = (q0 + (q2 + q3) * 3 + q1) >> M_SR; }
void sdr_rx_01(){         q1 = sdr_rx_common_q(); func_ptr = sdr_rx_02;   q8 = (q4 + (q6 + q7) * 3 + q5) >> M_SR; }
void sdr_rx_03(){         q2 = sdr_rx_common_q(); func_ptr = sdr_rx_04;   q5 = (q2 + (q0 + q1) * 3 + q3) >> M_SR; }
void sdr_rx_05(){         q3 = sdr_rx_common_q(); func_ptr = sdr_rx_06; process(i8, q8); }
void sdr_rx_07(){         q0 = sdr_rx_common_q(); func_ptr = sdr_rx_08;   q6 = (q0 + (q2 + q3) * 3 + q1) >> M_SR; }
void sdr_rx_09(){         q1 = sdr_rx_common_q(); func_ptr = sdr_rx_10;   q8 = (q6 + (q4 + q5) * 3 + q7) >> M_SR; }
void sdr_rx_11(){         q2 = sdr_rx_common_q(); func_ptr = sdr_rx_12;   q7 = (q2 + (q0 + q1) * 3 + q3) >> M_SR; }
void sdr_rx_13(){         q3 = sdr_rx_common_q(); func_ptr = sdr_rx_14; process(i8, q8); }
#else
void sdr_rx_00(){         i0 = sdr_rx_common_i(); func_ptr = sdr_rx_01;   i4 = (i0 + (i2 + i3) * 3 + i1) >> M_SR; }
void sdr_rx_02(){         i1 = sdr_rx_common_i(); func_ptr = sdr_rx_03;   i8 = (i4 + (i6 + i7) * 3 + i5) >> M_SR; }
void sdr_rx_04(){         i2 = sdr_rx_common_i(); func_ptr = sdr_rx_05;   i5 = (i2 + (i0 + i1) * 3 + i3) >> M_SR; }
void sdr_rx_06(){         i3 = sdr_rx_common_i(); func_ptr = sdr_rx_07;   i6 = i4; i7 = i5; q6 = q4; q7 = q5; }
void sdr_rx_07(){         q0 = sdr_rx_common_q(); func_ptr = sdr_rx_00;   q4 = (q0 + (q2 + q3) * 3 + q1) >> M_SR; }
void sdr_rx_01(){         q1 = sdr_rx_common_q(); func_ptr = sdr_rx_02;   q8 = (q4 + (q6 + q7) * 3 + q5) >> M_SR; }
void sdr_rx_03(){         q2 = sdr_rx_common_q(); func_ptr = sdr_rx_04;   q5 = (q2 + (q0 + q1) * 3 + q3) >> M_SR; }
void sdr_rx_05(){         q3 = sdr_rx_common_q(); func_ptr = sdr_rx_06; process(i8, q8); }
#endif
*/

// /*
static int16_t i_s0za1, i_s0zb0, i_s0zb1, i_s1za1, i_s1zb0, i_s1zb1;
static int16_t q_s0za1, q_s0zb0, q_s0zb1, q_s1za1, q_s1zb0, q_s1zb1, q_ac2;

#define M_SR  1  // CIC N=3
void sdr_rx_00(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_01;  int16_t i_s1za0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR; i_s0za1 = ac; int16_t ac2 = (i_s1za0 + (i_s1za1 + i_s1zb0) * 3 + i_s1zb1); i_s1za1 = i_s1za0; process(ac2, q_ac2); }
void sdr_rx_02(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_03;  i_s0zb1 = i_s0zb0; i_s0zb0 = ac; }
void sdr_rx_04(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_05;  i_s1zb1 = i_s1zb0; i_s1zb0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR; i_s0za1 = ac; }
void sdr_rx_06(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_07;  i_s0zb1 = i_s0zb0; i_s0zb0 = ac; }
void sdr_rx_01(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_02;  q_s0zb1 = q_s0zb0; q_s0zb0 = ac; }
void sdr_rx_03(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_04;  q_s1zb1 = q_s1zb0; q_s1zb0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR; q_s0za1 = ac; }
void sdr_rx_05(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_06;  q_s0zb1 = q_s0zb0; q_s0zb0 = ac; }
void sdr_rx_07(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_00;  int16_t q_s1za0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR; q_s0za1 = ac; q_ac2 = (q_s1za0 + (q_s1za1 + q_s1zb0) * 3 + q_s1zb1); q_s1za1 = q_s1za0; }
// */

/*
static int16_t i_s0za1, i_s0za2, i_s0zb0, i_s0zb1, i_s1za1, i_s1za2, i_s1zb0, i_s1zb1;
static int16_t q_s0za1, q_s0za2, q_s0zb0, q_s0zb1, q_s1za1, q_s1za2, q_s1zb0, q_s1zb1, q_ac2;

#define M_SR  0  // CIC N=2
void sdr_rx_00(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_01;  int16_t i_s1za0 = (ac + i_s0za1 + i_s0zb0 * 2 + i_s0zb1) >> M_SR; i_s0za1 = ac; int16_t ac2 = (i_s1za0 + i_s1za1 + i_s1zb0 * 2); i_s1za1 = i_s1za0; process(ac2, q_ac2); }
void sdr_rx_02(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_03;  i_s0zb0 = ac; }
void sdr_rx_04(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_05;  i_s1zb0 = (ac + i_s0za1 + i_s0zb0 * 2) >> M_SR; i_s0za1 = ac; }
void sdr_rx_06(){ int16_t ac = sdr_rx_common_i(); func_ptr = sdr_rx_07;  i_s0zb0 = ac; }
void sdr_rx_01(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_02;  q_s0zb0 = ac; }
void sdr_rx_03(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_04;  q_s1zb0 = (ac + q_s0za1 + q_s0zb0 * 2) >> M_SR; q_s0za1 = ac; }
void sdr_rx_05(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_06;  q_s0zb0 = ac; }
void sdr_rx_07(){ int16_t ac = sdr_rx_common_q(); func_ptr = sdr_rx_00;  int16_t q_s1za0 = (ac + q_s0za1 + q_s0zb0 * 2 + q_s0zb1) >> M_SR; q_s0za1 = ac; q_ac2 = (q_s1za0 + q_s1za1 + q_s1zb0 * 2); q_s1za1 = q_s1za0; }
*/

static int16_t ozi1, ozi2;

inline int16_t sdr_rx_common_q(){
  ADMUX = admux[0]; ADCSRA |= (1 << ADSC); int16_t ac = ADC - 511;
/*ozi2 = ozi1 + ozi2;          // Integrator section - needed?
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max(128 - (ozi2>>5) + 128, 0), 255); */
  return ac;
}

inline int16_t sdr_rx_common_i()
{
  ADMUX = admux[1]; ADCSRA |= (1 << ADSC); int16_t adc = ADC - 511; 
  static int16_t prev_adc;
  int16_t ac = (prev_adc + adc) / 2; prev_adc = adc;
#ifdef AF_OUT
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  ozi2 = ozi1 + ozi2;          // Integrator section
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max((ozi2>>5) + 128, 0), 255);
#endif // AF_OUT
  return ac;
}


#else // OLD_RX    //Orginal 2nd-order CIC:
//#define M4  1  // Enable to enable M=4 on second-stage (better alias rejection)

void sdr_rx()
{
  // process I for even samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
  ADMUX = admux[1];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 511; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  func_ptr = sdr_rx_q;    // processing function for next conversion
  sdr_rx_common();
  
  // Only for I: correct I/Q sample delay by means of linear interpolation
  static int16_t prev_adc;
  int16_t corr_adc = (prev_adc + adc) / 2;
  prev_adc = adc;
  adc = corr_adc;

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

  int16_t ac2;
  static int16_t z1;
  if(rx_state == 0 || rx_state == 4){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1 * 2;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(rx_state == 0){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1 * 2;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        ac2 >>= att2;  // digital gain control
        // post processing I and Q (down-sampled) results
        static int16_t v[7];
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = ac2;  // Delay to match Hilbert transform on Q branch
        
        int16_t ac = i + qh;
        ac = slow_dsp(ac);

        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
#define SECOND_ORDER_DUC  1
#ifdef SECOND_ORDER_DUC
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
#else
        ocomb = ac - ozd1; // Comb section
#endif
        ozd1 = ac;
      }
    } else _z1 = _ac;
  } else z1 = ac;

  rx_state++;
}

void sdr_rx_q()
{
  // process Q for odd samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
#ifdef TESTBENCH
  int16_t adc = NCO_Q();
#else
  ADMUX = admux[0];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 511; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
#endif
  func_ptr = sdr_rx;    // processing function for next conversion
#ifdef SECOND_ORDER_DUC
//  sdr_rx_common();  //necessary? YES!... Maybe NOT!
#endif

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

  int16_t ac2;
  static int16_t z1;
  if(rx_state == 3 || rx_state == 7){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1 * 2;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(rx_state == 7){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1 * 2;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        ac2 >>= att2;  // digital gain control
        // Process Q (down-sampled) samples
        static int16_t v[14];
        q = v[7];
  // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0, 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0, -79, 0, -21, 0, -8, 0, -2, 0) / 128;
        qh = ((v[0] - ac2) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS) when used in image-rejection scenario; (Hilbert transform require 4 additional bits)
        //qh = ((v[0] - ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
        for(uint8_t j = 0; j != 13; j++) v[j] = v[j + 1]; v[13] = ac2;
        //v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7]; v[7] = v[8]; v[8] = v[9]; v[9] = v[10]; v[10] = v[11]; v[11] = v[12]; v[12] = v[13]; v[13] = ac2;
      }
      rx_state = 0; return;
    } else _z1 = _ac;
  } else z1 = ac;

  rx_state++;
}

inline void sdr_rx_common()
{
  static int16_t ozi1, ozi2;
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  // Output stage [25% CPU@R=4;Fs=62.5k]
#ifdef SECOND_ORDER_DUC
  ozi2 = ozi1 + ozi2;          // Integrator section
#endif
  ozi1 = ocomb + ozi1;
#ifdef SECOND_ORDER_DUC
  OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#else
  OCR1AL = (ozi1>>5) + 128;
  OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#endif
}
#endif //OLD_RX

#endif  //!SIMPLE_RX

#ifdef SIMPLE_RX
volatile uint8_t admux[3];
static uint8_t rx_state = 0;

static struct rx {
  int16_t z1;
  int16_t za1;
  int16_t _z1;
  int16_t _za1;
} rx_inst[2];

void sdr_rx()
{
  static int16_t ocomb;
  static int16_t qh;

  uint8_t b = !(rx_state & 0x01);
  rx* p = &rx_inst[b];
  uint8_t _rx_state;
  int16_t ac;
  if(b){  // rx_state == 0, 2, 4, 6 -> I-stage
    ADMUX = admux[1];  // set MUX for next conversion
    ADCSRA |= (1 << ADSC);    // start next ADC conversion
    ac = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

    //sdr_common
    static int16_t ozi1, ozi2;
    if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
    // Output stage [25% CPU@R=4;Fs=62.5k]
    #define SECOND_ORDER_DUC 1
    #ifdef SECOND_ORDER_DUC
    ozi2 = ozi1 + ozi2;          // Integrator section
    #endif
    ozi1 = ocomb + ozi1;
    #ifdef SECOND_ORDER_DUC
    OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
    #else
    OCR1AL = (ozi1>>5) + 128;
    //OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  // OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
    #endif
    // Only for I: correct I/Q sample delay by means of linear interpolation
    static int16_t prev_adc;
    int16_t corr_adc = (prev_adc + ac) / 2;
    prev_adc = ac;
    ac = corr_adc;
    _rx_state = ~rx_state;
  } else {
    ADMUX = admux[0];  // set MUX for next conversion
    ADCSRA |= (1 << ADSC);    // start next ADC conversion
    ac = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
    _rx_state = rx_state;
  }
    
  if(_rx_state & 0x02){  // rx_state == I: 0, 4  Q: 3, 7  1st stage: down-sample by 2
    int16_t _ac = ac + p->za1 + p->z1 * 2;           // 1st stage: FA + FB
    p->za1 = ac;
    if(_rx_state & 0x04){                   // rx_state == I: 0  Q:7   2nd stage: down-sample by 2
      int16_t ac2 = _ac + p->_za1 + p->_z1 * 2;              // 2nd stage: FA + FB
      p->_za1 = _ac;
      if(b){
        // post processing I and Q (down-sampled) results
        ac2 >>= att2;  // digital gain control
        // post processing I and Q (down-sampled) results
        static int16_t v[7];
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = ac2;  // Delay to match Hilbert transform on Q branch

        int16_t ac = i + qh;
        ac = slow_dsp(ac);

        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
        #ifdef SECOND_ORDER_DUC
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
        #else
        ocomb = ac - ozd1; // Comb section
        #endif
        ozd1 = ac;
      } else {
        ac2 >>= att2;  // digital gain control
        // Process Q (down-sampled) samples
        static int16_t v[14];
        q = v[7];
        qh = ((v[0] - ac2) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
        for(uint8_t j = 0; j != 13; j++) v[j] = v[j + 1]; v[13] = ac2;
      }
    } else p->_z1 = _ac;
  } else p->z1 = ac;  // rx_state == I: 2, 6  Q: 1, 5

  rx_state++;
}
//#pragma GCC push_options
//#pragma GCC optimize ("Ofast")  // compiler-optimization for speed
//#pragma GCC pop_options  // end of DSP section
// */
#endif //SIMPLE_RX

ISR(TIMER2_COMPA_vect)  // Timer2 COMPA interrupt
{
  func_ptr();
#ifdef DEBUG
  numSamples++;
#endif
}

#pragma GCC pop_options  // end of DSP section