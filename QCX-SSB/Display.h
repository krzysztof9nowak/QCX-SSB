#pragma once

#include <Arduino.h>
#include "LCD.h"
#include "OLED.h"
#include "Blind.h"
#include "config.h"

#ifdef LCD
LCD lcd;
#endif

#ifdef OLED
SSD1306 lcd;
#endif

#ifdef BLIND
Blind lcd;
#endif

void inline lcd_blanks() { lcd.print(F("         ")); }

void fatal(const __FlashStringHelper *msg, int value = 0, char unit = '\0')
{
  lcd.setCursor(0, 1);
  lcd.print('!');
  lcd.print('!');
  lcd.print(msg);
  if (unit != '\0')
  {
    lcd.print('=');
    lcd.print(value);
    lcd.print(unit);
  }
  lcd_blanks();
  delay(1500);
  wdt_reset();
}