#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

#include "lcd_config.h"
#include "lcd.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof *(x))
#define LCD_TEXT_COLUMNS 16
#define LCD_PAGE_COUNT 8          // 8 pages vertically
#define LCD_COLUMN_COUNT 128      // 128 pixels horizontally



static inline void cs_low(void)  { PORTB &= ~_BV(PB0); }  /* SS active low */
static inline void cs_high(void) { PORTB |=  _BV(PB0); }

/* Initialize SPI interface
 *
 * Read the Atmel spec sheet and LCD driver spec sheet
 */
static void
spi_init(void)
{
  /* Pins: MOSI=PB2, SCK=PB1, SS=PB0, A0=PF1, RST=PF0, BL=PC7 */
  DDRB |= _BV(DDB2) | _BV(DDB1) | _BV(DDB0);
  PORTB |= _BV(PB0);              /* SS high (inactive) */

  DDRF |= _BV(DDF1) | _BV(DDF0);  /* A0, RST outputs */
  DDRC |= _BV(DDC7);              /* Backlight (OC4A) output */

  /* Hardware reset pulse (≈1ms) */
  PORTF &= ~_BV(PF0);
  _delay_ms(1);
  PORTF |= _BV(PF0);

  /* SPI: mode 3, master, fck/2 */
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA);
  SPSR = _BV(SPI2X);
}

/* Initialize the LED backlight.
 *
 * Look at Timer/Counter4 and the schematic */
static void
led_init(void)
{
  // PC7 (OC4A) as output
  DDRC |= _BV(DDC7);

  // Timer4: enable PWM on channel A and connect OC4A to the pin
  // COM4A1: non-inverting PWM on OC4A
  // PWM4A:  enable PWM channel A (required on ATmega32U4 Timer4)
  TCCR4A = _BV(PWM4A) | _BV(COM4A1);

  // Prescaler: clk/8 is a good default (adjust to taste)
  TCCR4B = _BV(CS41);

  // Fast PWM mode (Timer4 uses WGM4x in TCCR4D)
  TCCR4D = _BV(WGM40);     // 8-bit Fast PWM

  // Start off (0 = off, 0xFF = full on)
  OCR4A = 0x00;
}

/* Write to the LCD via spi */
static void
spi_write(uint8_t c)
{
  SPDR = c;
  while (!(SPSR & _BV(SPIF))) { /* wait */ }
}

/* Interpret SPI bytes as commands */
static void
lcd_cmd(void)
{
  A0_PORT &= ~_BV(A0);  /* command mode */
}

/* Interpret SPI bytes as image data */
static void
lcd_data(void)
{
  A0_PORT |= _BV(A0);   /* data mode */
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

  /* Mirror ASM init sequence (bias, scan dir, contrast, res ratio, vreg, ON) */
  cs_low();
  lcd_cmd();
  spi_write(0xA2);                  /* lcd_c_disp_set_bias */
  spi_write(0xC0);                  /* lcd_c_disp_set_scandir */
  spi_write(0x81);                  /* lcd_c_disp_set_contrast_1 */
  spi_write(15);                    /* lcd_c_disp_set_contrast_2 (1..63) */
  spi_write(0x22);                  /* lcd_c_disp_set_res_ratio */
  spi_write(0x2F);                  /* lcd_c_disp_vreg_on */
  spi_write(0xAF);                  /* lcd_c_disp_en | 1 -> display ON */
  cs_high();
}

/* Turn the display on */
void
lcd_on()
{
  cs_low();
  lcd_cmd();
  spi_write(0xAF); /* display on */
  cs_high();
}

/* Turn the display off */
void
lcd_off()
{
  cs_low();
  lcd_cmd();
  spi_write(0xAE); /* display off */
  cs_high();
}

/* Set the backlight (LED) PWM duty cycle
 *
 * Review schematics... :)
 */
void
lcd_led_set(uint8_t level)
{
  /* 0x00 = off ... 0xFF = max */
  OCR4A = level;   // 0..255
}

/* Sets LCD volume (contrast) */
void
lcd_volume_set(uint8_t level)
{
  /* Contrast 1..63; clamp then send 0x81,value */
  if (level > 63) level = 63;
  if (level == 0) level = 1;
  cs_low();
  lcd_cmd();
  spi_write(0x81);
  spi_write(level);
  cs_high();
}

/* Retets LCD; different from blanking/clearing */
void
lcd_reset(void)
{
  /* Use controller’s soft reset (does not clear GRAM) */
  cs_low();
  lcd_cmd();
  spi_write(0xE2); /* lcd_c_disp_reset */
  cs_high();

  /* Small settle delay */
  _delay_ms(2);
}

void
lcd_sleep(void)
{
  lcd_off();
  lcd_led_set(0);
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
  /* Vertical scroll: display start line (0x40 | (y&0x3F)) */
  cs_low();
  lcd_cmd();
  spi_write(0x40 | (y & 0x3F));
  cs_high();
}

/* Fills the LCD with the specified byte value */
void
lcd_fill(uint8_t c)
{
  for (uint8_t page = 0; page < LCD_PAGE_COUNT; ++page) {
    cs_low();
    lcd_cmd();
    spi_write(PAGE_ADDR_SET(page));
    spi_write(COL_ADDR_SET_UPPER(0));
    spi_write(COL_ADDR_SET_LOWER(0));

    lcd_data();
    for (uint16_t x = 0; x < LCD_COLUMN_COUNT; ++x) {
      spi_write(c);
    }
    cs_high();
  }
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
  for (uint8_t page = 0; page < LCD_PAGE_COUNT; ++page) {
    const char *s = buf[page] ? buf[page] : "";
    uint16_t written_cols = 0;

    cs_low();
    lcd_cmd();
    spi_write(PAGE_ADDR_SET(page));
    spi_write(COL_ADDR_SET_UPPER(0));
    spi_write(COL_ADDR_SET_LOWER(0));

    lcd_data();

    /* Write characters left-to-right from column 0 */
    for (uint8_t i = 0; s[i] && i < LCD_TEXT_COLUMNS; ++i) {
      uint8_t c = (uint8_t)s[i];
      glyph_t const *g = ascii_to_glyph(c);

      /* Always output 8 columns per character: glyph + pad */
      for (uint8_t col = 0; col < 8; ++col) {
        if (col < GLYPH_WIDTH) spi_write(g->cols[col]); else spi_write(0x00);
      }
      written_cols += 8;
      if (written_cols >= LCD_COLUMN_COUNT) break;
    }

    /* Clear any remaining columns on the line to avoid garbage */
    while (written_cols < LCD_COLUMN_COUNT) {
      spi_write(0x00);
      ++written_cols;
    }

    cs_high();
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
