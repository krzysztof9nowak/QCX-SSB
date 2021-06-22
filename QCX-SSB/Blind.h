#include "AbstractDisplay.h"
#include "config.h"

// This class is a dummy LCD replacement
class Blind : public AbstractDisplay
{ 
public:
  size_t _write(uint8_t b) {}
  void _setCursor(uint8_t _x, uint8_t _y) {}
  void _cursor() {}
  void _noCursor() {}
  void _begin(uint8_t x, uint8_t y) {}
  void noDisplay() {}
  //void _createChar(uint8_t l, uint8_t glyph[]) {}
};