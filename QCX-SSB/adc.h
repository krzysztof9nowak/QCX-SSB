#pragma once
#include <Arduino.h>

void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs)
{
  DIDR0 |= (1 << adcpin); // disable digital input 
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (adcpin & 0x0f);    // set analog input pin
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // If reflvl == true, set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  //ADCSRA |= (1 << ADSC);  // start ADC measurements
#ifdef ADC_NR
//  set_sleep_mode(SLEEP_MODE_ADC);  // ADC NR sleep destroys the timer2 integrity, therefore Idle sleep is better alternative (keeping clkIO as an active clock domain)
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
#endif
}

void adc_stop()
{
  //ADCSRA &= ~(1 << ADATE); // disable auto trigger
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz
#ifdef ADC_NR
  sleep_disable();
#endif
  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
}

int analogSafeRead(uint8_t pin, bool ref1v1 = false)
{  // performs classical analogRead with default Arduino sample-rate and analog reference setting; restores previous settings
  noInterrupts();
  for(;!(ADCSRA & (1 << ADIF)););  // wait until (a potential previous) ADC conversion is completed
  uint8_t adcsra = ADCSRA;
  uint8_t admux = ADMUX;
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz
  if(ref1v1) ADMUX &= ~(1 << REFS0);  // restore reference voltage AREF (1V1)
  else ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
  delay(1);  // settle
  int val = analogRead(pin);
  ADCSRA = adcsra;
  ADMUX = admux;
  interrupts();
  return val;
}

uint16_t analogSampleMic()
{
  uint16_t adc;
  noInterrupts();
  ADCSRA = (1 << ADEN) | (((uint8_t)log2((uint8_t)(F_CPU / 13 / (192307/1)))) & 0x07);  // hack: faster conversion rate necessary for VOX

  if((dsp_cap == SDR) && (vox_thresh >= 32)) digitalWrite(RX, LOW);  // disable RF input, only for SDR mod and with low VOX threshold
  //si5351.SendRegister(SI_CLK_OE, 0b11111111); // CLK2_EN=0, CLK1_EN,CLK0_EN=0
  uint8_t oldmux = ADMUX;
  for(;!(ADCSRA & (1 << ADIF)););  // wait until (a potential previous) ADC conversion is completed
  ADMUX = admux[2];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  for(;!(ADCSRA & (1 << ADIF)););  // wait until ADC conversion is completed
  ADMUX = oldmux;
  if((dsp_cap == SDR) && (vox_thresh >= 32)) digitalWrite(RX, HIGH);  // enable RF input, only for SDR mod and with low VOX threshold
  //si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  adc = ADC;
  interrupts();
  return adc;
}