#include "OLED.h"
#include "config.h"

// Enables I2C OLED communications that is not using the standard I/O pull-down approach with pull-up resistors, instead I/O is directly driven with 0V/5V
// I2C class used by SSD1306 driver; you may connect a SSD1306 (128x32) display on LCD header pins: 1 (GND); 2 (VCC); 13 (SDA); 14 (SCL)
class BitBangI2C
{
public:
#if (F_MCU > 20900000)
#define _DELAY()                     \
    for (uint8_t i = 0; i != 6; i++) \
        asm("nop");
#else
#define _DELAY()                     \
    for (uint8_t i = 0; i != 4; i++) \
        asm("nop"); // 4=731kb/s
#endif
#define _I2C_SDA (1 << 2) // PD2
#define _I2C_SCL (1 << 3) // PD3
#ifdef OLED_I2C_DIRECT_IO
#define _I2C_INIT() \
    _I2C_SDA_HI();  \
    _I2C_SCL_HI();  \
    DDRD |= (_I2C_SDA | _I2C_SCL); // direct I/O (no need for pull-ups)
#define _I2C_SDA_HI()  \
    PORTD |= _I2C_SDA; \
    _DELAY();
#define _I2C_SDA_LO()   \
    PORTD &= ~_I2C_SDA; \
    _DELAY();
#define _I2C_SCL_HI()  \
    PORTD |= _I2C_SCL; \
    _DELAY();
#define _I2C_SCL_LO()   \
    PORTD &= ~_I2C_SCL; \
    _DELAY();
#else // !OLED_I2C_DIRECT_IO
#define _I2C_INIT()     \
    PORTD &= ~_I2C_SDA; \
    PORTD &= ~_I2C_SCL; \
    _I2C_SDA_HI();      \
    _I2C_SCL_HI(); // open-drain
#define _I2C_SDA_HI()  \
    DDRD &= ~_I2C_SDA; \
    _DELAY();
#define _I2C_SDA_LO() \
    DDRD |= _I2C_SDA; \
    _DELAY();
#define _I2C_SCL_HI()  \
    DDRD &= ~_I2C_SCL; \
    _DELAY();
#define _I2C_SCL_LO() \
    DDRD |= _I2C_SCL; \
    _DELAY();
#endif // !OLED_I2C_DIRECT_IO
#define _I2C_START() \
    _I2C_SDA_LO();   \
    _I2C_SCL_LO(); // _I2C_SDA_HI();
#define _I2C_STOP() \
    _I2C_SDA_LO();  \
    _I2C_SCL_HI();  \
    _I2C_SDA_HI();
#define _I2C_SUSPEND() //_I2C_SDA_LO(); // SDA_LO to allow re-use as output port
#define _SendBit(data, bit) \
    if (data & 1 << bit)    \
    {                       \
        _I2C_SDA_HI();      \
    }                       \
    else                    \
    {                       \
        _I2C_SDA_LO();      \
    }                       \
    _I2C_SCL_HI();          \
    _I2C_SCL_LO();
    inline void start()
    {
        _I2C_INIT();
        _I2C_START();
    };
    inline void stop()
    {
        _I2C_STOP();
        _I2C_SUSPEND();
    };
    inline void SendByte(uint8_t data)
    {
        _SendBit(data, 7);
        _SendBit(data, 6);
        _SendBit(data, 5);
        _SendBit(data, 4);
        _SendBit(data, 3);
        _SendBit(data, 2);
        _SendBit(data, 1);
        _SendBit(data, 0);
        _I2C_SDA_HI(); // recv ACK
        _DELAY();      //
        _I2C_SCL_HI();
        _I2C_SCL_LO();
    }
    void SendRegister(uint8_t addr, uint8_t *data, uint8_t n)
    {
        start();
        SendByte(addr << 1);
        while (n--)
            SendByte(*data++);
        stop();
    }
    //void SendRegister(uint8_t addr, uint8_t val){ SendRegister(addr, &val, 1); }

    void begin(){};
    void beginTransmission(uint8_t addr)
    {
        start();
        SendByte(addr << 1);
    };
    bool write(uint8_t byte)
    {
        SendByte(byte);
        return 1;
    };
    uint8_t endTransmission()
    {
        stop();
        return 0;
    };
};

#ifdef OLED
#ifdef BITBANG_OLED
BitBangI2C Wire;
#else
#include <Wire.h>
#endif
#endif

void SSD1306::cmd(uint8_t b)
{
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(SSD1306_COMMAND);
    Wire.write(b);
    Wire.endTransmission();
}
void SSD1306::begin(uint8_t cols, uint8_t rows, uint8_t charsize = 0)
{
    Wire.begin();
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(SSD1306_COMMAND);
    for (uint8_t i = 0; i < sizeof(ssd1306_init_sequence); i++)
    {
        Wire.write(pgm_read_byte(&ssd1306_init_sequence[i]));
    }
    Wire.endTransmission();
    delayMicroseconds(100);
}

void SSD1306::setCursorPixels(uint8_t x, uint8_t y)
{
    oledX = x;
    oledY = y;
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(SSD1306_COMMAND);
    Wire.write(renderingFrame | (oledY & 0x07));
    Wire.write(0x10 | ((oledX & 0xf0) >> 4));
    Wire.write(oledX & 0x0f);
    Wire.endTransmission();
}
void SSD1306::drawCursor(bool en)
{
    //setCursorPixels(oledX, oledY + (FONT_W/(FONT_STRETCHH+1)));
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(SSD1306_DATA);
    Wire.write((en) ? 0xf0 : 0x00); // horizontal line
    Wire.endTransmission();
}
void SSD1306::_setCursor(uint8_t x, uint8_t y)
{
    if (curs)
    {
        drawCursor(false);
    }
    setCursorPixels(x * FONT_W, y * FONT_H);
    if (curs)
    {
        drawCursor(true);
        setCursorPixels(oledX, oledY);
    }
}

void SSD1306::newLine()
{
    oledY += FONT_H;
    if (oledY > SSD1306_PAGES - FONT_H)
    {
        oledY = SSD1306_PAGES - FONT_H;
    }
    setCursorPixels(0, oledY);
}

size_t SSD1306::_write(byte c)
{
    if ((c == '\n') || (oledX > ((uint8_t)128 - FONT_W)))
    {
        if (wrap)
            newLine();
        return 1;
    }
    //if(oledY > SSD1306_PAGES - FONT_H) return; //needed?
    c = ((c < 9) ? (c + '~') : c) - ' ';

    uint16_t offset = ((uint16_t)c) * FONT_W / (FONT_STRETCHH + 1) * FONT_H;
    uint8_t line = FONT_H;
    do
    {
        if (FONT_STRETCHV)
            offset = ((uint16_t)c) * FONT_W / (FONT_STRETCHH + 1) * FONT_H / (2 * FONT_STRETCHV);
        Wire.beginTransmission(SSD1306_ADDR);
        Wire.write(SSD1306_DATA);
        for (uint8_t i = 0; i < (FONT_W / (FONT_STRETCHH + 1)); i++)
        {
            uint8_t b = pgm_read_byte(&(font[offset++]));
            if (FONT_STRETCHV)
            {
                uint8_t b2 = 0;
                if (line > 1)
                    for (int i = 0; i != 4; i++)
                        b2 |= /* ! */ (b & (1 << i)) ? (1 << (i * 2)) | (1 << ((i * 2) + 1)) : 0x00;
                else
                    for (int i = 0; i != 4; i++)
                        b2 |= /* ! */ (b & (1 << (i + 4))) ? (1 << (i * 2)) | (1 << ((i * 2) + 1)) : 0x00;
                Wire.write(b2);
                if (FONT_STRETCHH)
                    Wire.write(b2);
            }
            else
            {
                Wire.write(b);
                if (FONT_STRETCHH)
                    Wire.write(b);
            }
        }
        Wire.endTransmission();
        if (FONT_H == 1)
        {
            oledX += FONT_W;
        }
        else
        {
            if (line > 1)
            {
                setCursorPixels(oledX, oledY + 1);
            }
            else
            {
                setCursorPixels(oledX + FONT_W, oledY - (FONT_H - 1));
            }
        }
    } while (--line);
    return 1;
}

void SSD1306::bitmap(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[])
{
    uint16_t j = 0;
    for (uint8_t y = y0; y < y1; y++)
    {
        setCursorPixels(x0, y);
        Wire.beginTransmission(SSD1306_ADDR);
        Wire.write(SSD1306_DATA);
        for (uint8_t x = x0; x < x1; x++)
        {
            Wire.write(pgm_read_byte(&bitmap[j++]));
        }
        Wire.endTransmission();
    }
    _setCursor(0, 0);
}