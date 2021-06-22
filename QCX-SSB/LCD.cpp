#include "LCD.h"
#include "LCD_fonts.h"
#include <Arduino.h>

#define pgm_cache_item(addr, sz) byte _item[sz]; memcpy_P(_item, addr, sz);  // copy array item from PROGMEM to SRAM

LCD::LCD(){
  for (uint8_t i = 0; i != N_FONTS; i++)
  { // Init fonts
    pgm_cache_item(fonts[i], 8);
    createChar(0x01 + i, /*fonts[i]*/ _item);
  }
}

void LCD::begin(uint8_t x = 0, uint8_t y = 0)
{ // Send command , make sure at least 40ms after power-up before sending commands
    bool reinit = (x == 0) && (y == 0);
    DDRD |= 0xf << _dn | 1 << _en; // Make data, EN outputs
    DDRC |= 1 << _rs;
    //PORTC &= ~(1 << _rs);                          // Set RS low in case to support pull-down when DDRC is output
    delayMicroseconds(50000); // *
    LCD_RS_LO();
    LCD_EN_LO();
    cmd(0x33); // Ensures display is in 8-bit mode
    delayMicroseconds(4500);
    cmd(0x33);
    delayMicroseconds(4500);
    cmd(0x33);
    delayMicroseconds(150); // * Ensures display is in 8-bit mode
    cmd(0x32);              // Puts display in 4-bit mode
    cmd(0x28);              // * Function set: 2-line, 5x8
    cmd(0x0c);              // Display on
    if (reinit)
        return;
    cmd(0x01); // Clear display
    delay(3);  // Allow to execute Clear on display [https://www.sparkfun.com/datasheets/LCD/HD44780.pdf, p.49, p58]
    cmd(0x06); // * Entrymode: left, shift-dec
}

// Since LCD is using PD0(RXD), PD1(TXD) pins in the data-path, some co-existence feature is required when using the serial port.
// The following functions are temporarily disabling the serial port when LCD writes happen, and make sure that serial transmission is ended.
// To prevent that LCD writes are received by the serial receiver, PC2 is made HIGH during writes to pull-up TXD via a diode.
// The RXD, TXD lines are connected to the host via 1k resistors, a 1N4148 is placed between PC2 (anode) and the TXD resistor.
// There are two drawbacks when continuous LCD writes happen: 1. noise is leaking via the AREF pull-ups into the receiver 2. serial data cannot be received.
void LCD::pre()
{
#ifdef _SERIAL
    if (!vox && cat_active)
    {
        Serial.flush();
        while (millis() < rxend_event)
            wdt_reset();
        PORTC |= 1 << 2;
        DDRC |= 1 << 2;
    }
    UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0)); // Complete serial TX and RX; mask PD1 LCD data-exchange by pulling-up TXD via PC2 HIGH; enable PD0/PD1, disable serial port
#endif
    noInterrupts(); // do not allow LCD tranfer to be interrupted, to prevent backlight to lighten-up
}

void LCD::post()
{
    if (backlight)
        PORTD |= 0x08;
    else
        PORTD &= ~0x08; // Backlight control
#ifdef _SERIAL
    //UCSR0B |= (1<<RXEN0)|(1<<TXEN0); if(!vox) if(cat_active){ DDRC &= ~(1<<2); } // Enable serial port, disable PD0, PD1; disable PC2
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
    if (!vox && cat_active)
    {
        PORTC &= ~(1 << 2);
    } // Enable serial port, disable PD0, PD1; PC2 LOW to prevent CAT TX disruption via MIC input
#endif
    interrupts();
}


#ifdef RS_HIGH_ON_IDLE
  void LCD::cmd(uint8_t b)
  {
    pre();
    uint8_t nibh = LCD_PREP_NIBBLE(b >> 4);  // Prepare high nibble data and enable high
    PORTD = nibh;                            // Send high nibble data and enable high
    uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf); // Prepare low nibble data and enable high
    LCD_RS_LO();
    LCD_EN_LO();
    PORTD = nibl; // Send low nibble data and enable high
    asm("nop");
    asm("nop"); // Keep RS low, but complete enable cycle (should be 500ns)
    LCD_EN_LO();
    LCD_RS_HI();
    post();
    delayMicroseconds(60); // Execution time  (37+4)*1.25 us
  }
  size_t LCD::write(uint8_t b)
  { // Write data:    send nibbles while RS high
    pre();
    uint8_t nibh = LCD_PREP_NIBBLE(b >> 4);  // Prepare high nibble data and enable high
    PORTD = nibh;                            // Send high nibble data and enable high
    uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf); // Prepare low nibble data and enable high
    LCD_RS_HI();
    LCD_EN_LO();
    PORTD = nibl; // Send low nibble data and enable high
    asm("nop");
    asm("nop"); // Keep RS high, but complete enable cycle (should be 500ns)
    LCD_EN_LO();
    post();
    delayMicroseconds(60); // Execution time  (37+4)*1.25 us
    return 1;
  }
#else  //!RS_HIGH_ON_IDLE
  void LCD::nib(uint8_t b)
  { // Send four bit nibble to display
    pre();
    PORTD = LCD_PREP_NIBBLE(b); // Send data and enable high
    //asm("nop");                                    // Enable high pulse width must be at least 230ns high, data-setup time 80ns
    delayMicroseconds(4);
    LCD_EN_LO();
    post();
    //delayMicroseconds(52);                         // Execution time
    delayMicroseconds(60); // Execution time
  }
  void LCD::cmd(uint8_t b)
  {
    nib(b >> 4);
    nib(b & 0xf);
  } // Write command: send nibbles while RS low
  size_t LCD::write(uint8_t b)
  { // Write data:    send nibbles while RS high
    pre();
    //LCD_EN_HI();                                   // Complete Enable cycle must be at least 500ns (so start early)
    uint8_t nibh = LCD_PREP_NIBBLE(b >> 4);  // Prepare high nibble data and enable high
    PORTD = nibh;                            // Send high nibble data and enable high
    uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf); // Prepare low nibble data and enable high
    //asm("nop");                                    // Enable high pulse width must be at least 230ns high, data-setup time 80ns; ATMEGA clock-cycle is 50ns (so at least 5 cycles)
    LCD_RS_HI();
    LCD_EN_LO();
    PORTD = nibl; // Send low nibble data and enable high
    LCD_RS_LO();
    //asm("nop"); asm("nop");                        // Complete Enable cycle must be at least 500ns
    //PORTD = nibl;                                  // Send low nibble data and enable high
    //asm("nop");                                    // Enable high pulse width must be at least 230ns high, data-setup time 80ns; ATMEGA clock-cycle is 50ns (so at least 5 cycles)
    LCD_RS_HI();
    LCD_EN_LO();
    LCD_RS_LO();
    post();
    delayMicroseconds(60); // Execution time  (37+4)*1.25 us
    return 1;
  }
#endif // RS_HIGH_ON_IDLE