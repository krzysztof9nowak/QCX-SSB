#include <Arduino.h>
#include "OLED_fonts.h"
#include "AbstractDisplay.h"



#define BRIGHT 1
//#define CONDENSED 1
//#define INVERSE  1
static const uint8_t ssd1306_init_sequence[] PROGMEM = {
    // Initialization Sequence
    //  0xAE,     // Display OFF (sleep mode)
    0x20, 0b10, // Set Memory Addressing Mode
                // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
                // 10=Page Addressing Mode (RESET); 11=Invalid
    0xB0,       // Set Page Start Address for Page Addressing Mode, 0-7
    0xC8,       // Set COM Output Scan Direction.  Flip Veritically.
    0x00,       // Set low nibble of column address
    0x10,       // Set high nibble of column address
    0x40,       // Set display start line address
#ifdef BRIGHT
    0x81, /*32*/ 0x7F, // Set contrast control register
#else
    0x81, 32,   // Set contrast control register
#endif
    0xA1, // Set Segment Re-map. A0=column 0 mapped to SEG0; A1=column 127 mapped to SEG0. Flip Horizontally
#ifdef INVERSE
    0xA7, // Set display mode. A6=Normal; A7=Inverse
#else
    0xA6,       // Set display mode. A6=Normal; A7=Inverse
#endif
    0xA8, 0x1F, // Set multiplex ratio(1 to 64)
    0xA4,       // Output RAM to Display
                // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
    0xD3, 0x00, // Set display offset. 00 = no offset
    0xD5, 0x01, // 0x80--set display clock divide ratio/oscillator frequency
#ifdef BRIGHT
    0xD9, 0xF1, // 0xF1=brighter //0x22 Set pre-charge period
#else
    0xD9, 0x03, // 0x22 Set pre-charge period
#endif
#ifdef CONDENSED
    0xDA, 0x12, // Set com pins hardware configuration
#else
    0xDA, 0x02, // Set com pins hardware configuration
#endif
    0xDB, 0x05, //0x20, --set vcomh 0x20 = 0.77xVcc
    0x8D, 0x14, // Set DC-DC enable
    0xAF,       // Display ON
};


#define SSD1306_ADDR 0x3C // Slave address
#define SSD1306_PAGES 4
#define SSD1306_COMMAND 0x00
#define SSD1306_DATA 0x40

class SSD1306 : public AbstractDisplay
{ // https://www.buydisplay.com/download/manual/ER-OLED0.91-3_Series_Datasheet.pdf
private:
  uint8_t oledX = 0, oledY = 0;
  uint8_t renderingFrame = 0xB0;
  bool wrap = false;
  bool curs = false;
  void setCursorPixels(uint8_t x, uint8_t y);
  void cmd(uint8_t b);
  void drawCursor(bool en);
  

public:
  size_t _write(byte c);
  void _noCursor() { curs = false; }
  void _cursor() { curs = true; }
  void _setCursor(uint8_t x, uint8_t y);
  void _clear(){};
  
  void noDisplay() { cmd(0xAE); }
  void createChar(uint8_t l, uint8_t glyph[]) {}
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = 0);
  void newLine();
  void bitmap(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[]);
};
