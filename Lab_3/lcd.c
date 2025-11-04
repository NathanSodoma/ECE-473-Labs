#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>

#include "lcd_config.h"

#include "lcd.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof *(x))

#define TOP_PAGE (LCD_PAGE_COUNT - 1)

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
  DDRB |= _BV(PB2) | _BV(PB1) | _BV(PB0);
  PORTB |= _BV(PB0);               // SS idle high (inactive)

  // A0 (data/command) and RST
  DDRF |= _BV(PF1) | _BV(PF0);

  // SPI: enable, master, MODE 3, fosc/2
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
  DDRC |= _BV(PC7);

  // --- Enable PLL and feed it to Timer4 ---
  // Enable PLL
  PLLCSR = _BV(PLLE);
  // Wait for lock
  while (!(PLLCSR & _BV(PLOCK))) { }
  // Route PLL clock to Timer4
  PLLCSR |= _BV(PCKE);

  // --- Timer4: non-inverting PWM on OC4A ---
  // Clear on compare match (COM4A1), enable PWM channel A (PWM4A)
  TCCR4A = _BV(COM4A1) | _BV(PWM4A);

  // Fast PWM mode (WGM bits live in TCCR4D on 32U4). Set WGM to Fast PWM.
  TCCR4D = 0; // WGM4[1:0] = 00 -> Fast PWM

  // Choose a prescaler; 1/32 is a nice quiet PWM with PLL clock
  // CS4[3:0] = 0b1001 (datasheet mapping) → experiment if needed.
  // Safe choice many boards use is /128:
  TCCR4B = _BV(CS43);  // clk/128 from PLL

  TCCR4C = 0;
  TCCR4E = 0;

  // Start at 0% duty
  TC4H = 0;
  OCR4A = 0;
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

static void lcd_reset_pulse(void)
{
  RST_PORT &= ~_BV(RST);
  _delay_ms(1);
  RST_PORT |= _BV(RST);
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

  // Hardware reset pulse (RST low ~1ms)
  PORTF &= ~_BV(PF0);
  _delay_ms(1);
  PORTF |= _BV(PF0);

  // Begin init command sequence
  PORTB &= ~_BV(PB0);   // SS low
  lcd_cmd();
  spi_write(0xA0);                // bias 1/6
  spi_write(0xC0);                // scan dir (normal)  :contentReference[oaicite:5]{index=5}
  spi_write(0x81); spi_write(15); // contrast command + value (1..63)  :contentReference[oaicite:6]{index=6}
  spi_write(0x22);                // resistor ratio                     :contentReference[oaicite:7]{index=7}
  spi_write(0x2F);                // booster/reg/follower ON            :contentReference[oaicite:8]{index=8}
  spi_write(0xAF);                // display ON                         :contentReference[oaicite:9]{index=9}
  PORTB |= _BV(PB0);    // SS high
}

/* Turn the display on */
void
lcd_on()
{
   PORTB &= ~_BV(PB0); 
   lcd_cmd(); 
   spi_write(0xAF); 
   PORTB |= _BV(PB0);
}

/* Turn the display off */
void
lcd_off()
{
  PORTB &= ~_BV(PB0); 
  lcd_cmd(); 
  spi_write(0xAE); 
  PORTB |= _BV(PB0);
  // base command with LSB=0. :contentReference[oaicite:12]{index=12}
}

/* Set the backlight (LED) PWM duty cycle
 *
 * Review schematics... :) 
 */
void
lcd_led_set(uint8_t level)
{
  // Map 0..255 to ~0..1023 (10-bit). Simple upscale by 4.
  uint16_t duty = (uint16_t)level * 4; // 0..1020
  if (duty > 1023) duty = 1023;

  // IMPORTANT: write high bits to TC4H before OCR4A
  TC4H = (duty >> 8) & 0x03;
  OCR4A = duty & 0xFF;
}

/* Sets LCD volume (contrast) */
void
lcd_volume_set(uint8_t level)
{
  if (level < 1) level = 1;
  if (level > 63) level = 63;
  PORTB &= ~_BV(PB0);
  lcd_cmd();
  spi_write(0x81);
  spi_write(level);
  PORTB |= _BV(PB0);
}

/* Retets LCD; different from blanking/clearing */
void
lcd_reset(void)
{
 lcd_off();
  PORTF &= ~_BV(PF0);
  _delay_ms(1);
  PORTF |= _BV(PF0);
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
  y &= 0x3F;
  PORTB &= ~_BV(PB0);
  lcd_cmd();
  spi_write(0x40 | y);
  PORTB |= _BV(PB0);
}

/* Fills the LCD with the specified byte value */
void
lcd_fill(uint8_t c)
{
 for (uint8_t page = 0; page < LCD_PAGE_COUNT; ++page) {
    PORTB &= ~_BV(PB0);
    lcd_cmd();
    spi_write(PAGE_ADDR_SET(page));
    spi_write(COL_ADDR_SET_UPPER(0));
    spi_write(COL_ADDR_SET_LOWER(0));
    PORTB |= _BV(PB0);

    PORTB &= ~_BV(PB0);
    lcd_data();
    for (uint8_t x = 0; x < LCD_COLUMN_COUNT; ++x) spi_write(c);
    PORTB |= _BV(PB0);
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
   // --- Row 0 (top page) ---
  // Set cursor to page 0, column 0
  ss_low();
  lcd_cmd();
  spi_write(PAGE_ADDR_SET(TOP_PAGE));   // <-- was 0; now the physical top page
  spi_write(COL_ADDR_SET_UPPER(0));
  spi_write(COL_ADDR_SET_LOWER(0));
  ss_high();

  // Stream exactly 128 bytes: 16 cells * 8 columns
  ss_low();
  lcd_data();

  // Safe, bounded fetch of row-0 string
  char const *s = buf ? buf[0] : NULL;
  uint8_t len = 0;
  if (s) {
    while (len < (LCD_COLUMN_COUNT / 8) && s[len] != '\0') ++len;  // max 16 chars
  }

  for (uint8_t ch = 0; ch < (LCD_COLUMN_COUNT / 8); ++ch) {
    char c = (ch < len) ? s[ch] : ' ';              // pad with spaces
    if (c < ' ' || c > '~') c = ' ';               // sanitize to printable ASCII
    glyph_t const *g = ascii_to_glyph(c);
    // Overwrite full 8-pixel cell (5 glyph cols + 3 zeros)
    for (uint8_t col = 0; col < 8; ++col) {
      spi_write(col < GLYPH_WIDTH ? g->cols[col] : 0x00);
    }
  }
  ss_high();

  // --- Clear the other pages so you never see duplicates/ghosting ---
  for (uint8_t page = 1; page < LCD_PAGE_COUNT; ++page) {
    ss_low();
    lcd_cmd();
    spi_write(PAGE_ADDR_SET(page));
    spi_write(COL_ADDR_SET_UPPER(0));
    spi_write(COL_ADDR_SET_LOWER(0));
    ss_high();

    ss_low();
    lcd_data();
    for (uint8_t i = 0; i < LCD_COLUMN_COUNT; ++i) spi_write(0x00);
    ss_high();
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
