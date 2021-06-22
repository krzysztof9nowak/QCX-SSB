#pragma once

#include <Arduino.h>
#include "config.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "AbstractDisplay.h"

uint8_t backlight = 8;
//#define RS_HIGH_ON_IDLE   1   // Experimental LCD support where RS line is high on idle periods to comply with SDA I2C standard.

class LCD : public AbstractDisplay
{                   // inspired by: http://www.technoblogy.com/show?2BET
public:             // LCD1602 display in 4-bit mode, RS is pull-up and kept low when idle to prevent potential display RFI via RS line
#define _dn 0       // PD0 to PD3 connect to D4 to D7 on the display
#define _en 4       // PD4 - MUST have pull-up resistor
#define _rs 4       // PC4 - MUST have pull-up resistor
#define RS_PULLUP 1 // Use pullup on RS line, ensures compliancy to the absolute maximum ratings for the si5351 sda input that is shared with rs pin of lcd
#ifdef RS_PULLUP
#define LCD_RS_HI()    \
  DDRC &= ~(1 << _rs); \
  asm("nop");                         // RS high (pull-up)
#define LCD_RS_LO() DDRC |= 1 << _rs; // RS low (pull-down)
#else
#define LCD_RS_LO() PORTC &= ~(1 << _rs);                                  // RS low
#define LCD_RS_HI() PORTC |= (1 << _rs);                                   // RS high
#endif                                                                     //RS_PULLUP
#define LCD_EN_LO() PORTD &= ~(1 << _en);                                  // EN low
#define LCD_EN_HI() PORTD |= (1 << _en);                                   // EN high
#define LCD_PREP_NIBBLE(b) (PORTD & ~(0xf << _dn)) | (b) << _dn | 1 << _en // Send data and enable high
  
  LCD();
  void begin(uint8_t x = 0, uint8_t y = 0);
  void pre();
  void post();
  void cmd(uint8_t b);
  size_t write(uint8_t b);
  #ifndef RS_HIGH_ON_IDLE
  void nib(uint8_t b);
  #endif

  void setCursor(uint8_t x, uint8_t y)
  {
    cmd(0x80 | (x + y * 0x40));
  }
  void cursor() { cmd(0x0e); }
  void noCursor() { cmd(0x0c); }
  void noDisplay() { cmd(0x08); }
  void createChar(uint8_t l, uint8_t glyph[])
  {
    cmd(0x40 | ((l & 0x7) << 3));
    for (int i = 0; i != 8; i++)
      write(glyph[i]);
  }
};
