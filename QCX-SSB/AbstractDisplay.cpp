#include "AbstractDisplay.h"

AbstractDisplay::AbstractDisplay() { clear(); }; // TODO: inicjalizuj parenta w klasach pochodnych

#ifdef CAT_EXT
size_t AbstractDisplay::write(uint8_t b)
{
    if ((x < 16) && (y < 2))
    {
        text[y * 16 + x] = ((b < 9) ? "> :*#AB"[b - 1] /*(b + 0x80 - 1)*/ /*(uint8_t[]){ 0xAF, 0x20, 0xB0, 0xB1, 0xB2, 0xA6, 0xE1, 0x20 }[b-1]*/ : b);
        x++;
    }
    return _write(b);
}
void AbstractDisplay::setCursor(uint8_t x, uint8_t y)
{
    this->x = x;
    this->y = y;
    _setCursor(x, y);
}
void AbstractDisplay::cursor()
{
    curs = true;
    _cursor();
}
void AbstractDisplay::noCursor()
{
    curs = false;
    _noCursor();
}
void AbstractDisplay::clear()
{
    for (uint8_t i = 0; i != 2 * 16; i++)
        text[i] = ' ';
    text[2 * 16] = '\0';
    x = 0;
    y = 0;
}
#else
size_t AbstractDisplay::write(uint8_t b)
{
    return _write(b);
}
void AbstractDisplay::setCursor(uint8_t x, uint8_t y)
{
    _setCursor(x, y);
}
void AbstractDisplay::cursor()
{
    _cursor();
}
void AbstractDisplay::noCursor()
{
    _noCursor();
}
void AbstractDisplay::clear()
{
}
#endif