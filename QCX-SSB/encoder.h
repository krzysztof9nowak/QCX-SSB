#include <Arduino.h>
#include "config.h"

volatile int8_t encoder_val = 0;
volatile int8_t encoder_step = 0;
static uint8_t last_state;
ISR(PCINT2_vect)
{ // Interrupt on rotary encoder turn
  //noInterrupts();
  //PCMSK2 &= ~((1 << PCINT22) | (1 << PCINT23));  // mask ROT_A, ROT_B interrupts
  switch (last_state = (last_state << 4) | (_digitalRead(ROT_B) << 1) | _digitalRead(ROT_A))
  { //transition  (see: https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/  )
//#define ENCODER_ENHANCED_RESOLUTION  1
#ifdef ENCODER_ENHANCED_RESOLUTION // Option: enhance encoder from 24 to 96 steps/revolution, see: appendix 1, https://www.sdr-kits.net/documents/PA0KLT_Manual.pdf
  case 0x31:
  case 0x10:
  case 0x02:
  case 0x23:
    encoder_val++;
    break;
  case 0x32:
  case 0x20:
  case 0x01:
  case 0x13:
    encoder_val--;
    break;
#else
    //    case 0x31: case 0x10: case 0x02: case 0x23: if(encoder_step < 0) encoder_step = 0; encoder_step++; if(encoder_step >  3){ encoder_step = 0; encoder_val++; } break;  // encoder processing for additional immunity to weared-out rotary encoders
    //    case 0x32: case 0x20: case 0x01: case 0x13: if(encoder_step > 0) encoder_step = 0; encoder_step--; if(encoder_step < -3){ encoder_step = 0; encoder_val--; } break;
  case 0x23:
    encoder_val++;
    break;
  case 0x32:
    encoder_val--;
    break;
#endif
  }
  //PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // allow ROT_A, ROT_B interrupts
  //interrupts();
}
void encoder_setup()
{
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // interrupt-enable for ROT_A, ROT_B pin changes; see https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts
  PCICR |= (1 << PCIE2);
  last_state = (_digitalRead(ROT_B) << 1) | _digitalRead(ROT_A);
  interrupts();
}
