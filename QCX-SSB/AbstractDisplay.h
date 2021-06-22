#pragma once

#include <Arduino.h>
#include "config.h"

// comment on virtual functions:
// https://stackoverflow.com/questions/733737/are-inline-virtual-functions-really-a-non-sense


class AbstractDisplay : public Print{
private:
    virtual void _setCursor(uint8_t x, uint8_t y) = 0;
    virtual void _noCursor() = 0;
    virtual void _cursor() = 0;
    virtual size_t _write(uint8_t b) = 0;
    virtual void noDisplay() = 0;
public:
#ifdef CAT_EXT
  uint8_t x, y;
  bool curs;
  char text[2 * 16 + 1];
#endif
    AbstractDisplay();
    size_t write(uint8_t b);
    void setCursor(uint8_t x, uint8_t y);
    void cursor();
    void noCursor();
    void clear();  
};

volatile uint8_t cat_active = 0;
volatile uint32_t rxend_event = 0;
volatile uint8_t vox = 0;