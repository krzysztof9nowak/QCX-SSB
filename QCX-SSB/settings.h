#pragma once
#include <Arduino.h>
#include "config.h"

#if !(defined(ARDUINO_ARCH_AVR))
#error "Unsupported architecture, select Arduino IDE > Tools > Board > Arduino AVR Boards > Arduino Uno."
#endif
#if (F_CPU != 16000000)
#error "Unsupported clock frequency, Arduino IDE must specify 16MHz clock; alternate crystal frequencies may be specified with F_MCU."
#endif
#undef F_CPU
#define F_CPU 20007000 // Actual crystal frequency of 20MHz XTAL1, note that this declaration is just informative and does not correct the timing in Arduino functions like delay(); hence a 1.25 factor needs to be added for correction.
#ifndef F_MCU
#define F_MCU 20000000 // 20MHz ATMEGA328P crystal
#endif


#define F_SAMP_TX 4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#if (F_MCU != 20000000)
const int16_t _F_SAMP_TX = (F_MCU * 4810LL / 20000000); // Actual ADC sample-rate; used for phase calculations
#else
#define _F_SAMP_TX F_SAMP_TX
#endif