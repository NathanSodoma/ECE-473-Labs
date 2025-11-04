#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>

#include "lcd_config.h"

#include "lcd.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof *(x))

#ifndef A0_PORT
#  define A0_PORT PORTF
#  define A0_DDR  DDRF
#  define A0      PF1
#endif

#ifndef RST_PORT
#  define RST_PORT PORTF
#  define RST_DDR  DDRF
#  define RST      PF0
#endif

#ifndef SS_PORT
#  define SS_PORT  PORTB
#  define SS_DDR   DDRB
#  define SS       PB0
#endif

#ifndef MOSI_DDR
#  define MOSI_DDR DDRB
#  define MOSI     PB2
#  define SCK_DDR  DDRB
#  define SCK      PB1
#endif

#ifndef BL_PORT
#  define BL_PORT  PORTC
#  define BL_DDR   DDRC
#  define BL_PIN   PC7      // Backlight pin (also OC4A on ATmega32U4)
#endif



static inline void ss_low(void)  { SS_PORT &= ~_BV(SS); }
static inline void ss_high(void) { SS_PORT |=  _BV(SS); }

/* Initialize SPI interface
 *
 * Read the Atmel spec sheet and LCD driver spec sheet
 */
static void
spi_init(void)
{
  MOSI_DDR |= _BV(MOSI);
  SCK_DDR  |= _BV(SCK);
  SS_DDR   |= _BV(SS);
  ss_high(); // inactive (active-low)

  // A0 and RST as outputs
  A0_DDR  |= _BV(A0);
  RST_DDR |= _BV(RST);

  // SPI: enable, master, mode 3 (CPOL=1, CPHA=1), fosc/2 (SPI2X=1)
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA);
  SPSR = _BV(SPI2X);
  // mirrors LCDDriver.asm setup. :contentReference[oaicite:9]{index=9}
}

/* Initialize the LED backlight.
 *
 * Look at Timer/Counter4 and the schematic */
static void
led_init(void)
{
  BL_DDR |= _BV(BL_PIN); // output
  // default off
  BL_PORT &= ~_BV(BL_PIN);
  // (Optional) You can swap to Timer4 PWM later for smooth brightness.
  // See lcd_led_set() below.
}

/* Write to the LCD via spi */
static void
spi_write(uint8_t c)
{
  SPDR = c;
  while (!(SPSR & _BV(SPIF))) { }
}

/* Interpret SPI bytes as commands */
static void
lcd_cmd(void)
{
  A0_PORT &= ~_BV(A0);
}

/* Interpret SPI bytes as image data */
static void
lcd_data(void)
{
  A0_PORT |= _BV(A0);  // A0=1 -> data
}

/* Initializes the LCD and Backlight,
 * but does not turn it on yet
 *
 * Review the LCD spec sheet for correct
 * initialization routine.
 */
void
lcd_init(void)
{
  spi_init();
  led_init();

  lcd_reset_pulse();

  ss_low();
  lcd_cmd();
  // The same init sequence as LCDDriver.asm:
  spi_write(0xA2);              // set bias (1/6)
  spi_write(0xC0);              // set scan direction (normal)
  spi_write(0x81); spi_write(15); // contrast command then value (1..63)
  spi_write(0x22);              // regulator resistor ratio
  spi_write(0x2F);              // booster/regulator/follower on
  spi_write(0xAF);              // display ON
  ss_high();
  // Sequence mirrors asm exactly. :contentReference[oaicite:10]{index=10}
}

/* Turn the display on */
void
lcd_on()
{
   ss_low(); 
   lcd_cmd(); 
   spi_write(0xAF); 
   ss_high();
}

/* Turn the display off */
void
lcd_off()
{
  ss_low(); lcd_cmd(); spi_write(0xAE); ss_high(); // OFF
  // base command with LSB=0. :contentReference[oaicite:12]{index=12}
}

/* Set the backlight (LED) PWM duty cycle
 *
 * Review schematics... :) 
 */
void
lcd_led_set(uint8_t level)
{
  // Simple version (like asm): treat nonzero as ON, zero as OFF.
  if (level) { BL_PORT |= _BV(BL_PIN); } else { BL_PORT &= ~_BV(BL_PIN); }
  // If you want “real” brightness: set up Timer4 FastPWM on OC4A (PC7) and write OCR4A=level.
  // The skeleton hints Timer/Counter4; you can upgrade later without touching callsites. :contentReference[oaicite:13]{index=13}
}

/* Sets LCD volume (contrast) */
void
lcd_volume_set(uint8_t level)
{
  if (level < 1) level = 1;
  if (level > 63) level = 63;
  ss_low(); lcd_cmd(); spi_write(0x81); spi_write(level); ss_high();
}

/* Retets LCD; different from blanking/clearing */
void
lcd_reset(void)
{
  lcd_off();
  lcd_reset_pulse();
  lcd_on();
}

void
lcd_sleep(void)
{
  lcd_off();
}

void
lcd_wake(void)
{
  lcd_on();
}

/* Scrolls the LCD by setting the display start line */
void
lcd_scroll(uint8_t y)
{
   y &= 0x3F; // 0..63
  ss_low(); lcd_cmd(); spi_write(0x40 | y); ss_high();
}

/* Fills the LCD with the specified byte value */
void
lcd_fill(uint8_t c)
{
  for (uint8_t page = 0; page < LCD_PAGE_COUNT; ++page) {
    lcd_cmd();
    ss_low();
    spi_write(PAGE_ADDR_SET(page));
    spi_write(COL_ADDR_SET_UPPER(0));
    spi_write(COL_ADDR_SET_LOWER(0));
    ss_high();

    lcd_data();
    ss_low();
    for (uint8_t col = 0; col < LCD_COLUMN_COUNT; ++col) {
      spi_write(c);
    }
    ss_high();
}

/* Clears the LCD */
void
lcd_clear(void)
{
  lcd_fill(0x00);
}

/* Dumps a text buffer to the LCD
 *
 * Text buffer = array of 8 strings
 *
 * Converts each character to a glyph for
 * printing
 */
void
lcd_flush_text(lcd_text_buffer_t const buf)
{
  /* ... */
  glyph_t const *g = ascii_to_glyph(c);
  for (uint8_t col = 0; col < 8; ++col) {
    if (col < GLYPH_WIDTH) spi_write(g->cols[col]);
    else spi_write(0);
  }
}

/* Dumps a pixel buffer to the LCD
 *
 * Pixel buffer = raw pixel data
 */
void
lcd_flush_pixels(lcd_pixel_buffer_t const buf)
{
  for (size_t y = 0; y < LCD_PAGE_COUNT; ++y) {
    lcd_cmd();
    spi_write(PAGE_ADDR_SET(y));
    spi_write(COL_ADDR_SET_UPPER(0));
    spi_write(COL_ADDR_SET_LOWER(0));

    lcd_data();
    for (size_t x = 0; x < LCD_COLUMN_COUNT; ++x) {
      uint8_t c = buf[y][x];
      spi_write(c);
    }
  }
}
