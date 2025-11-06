#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <avr/io.h>
#include <util/delay.h>

#include "lcd/lcd.h"

lcd_text_buffer_t text_buffer = {
  "hello world!", "1", "2", "3", "4", "5", "6", "Goodbye!"};

  int
  main()
  {
    /* Initialize LCD */
    lcd_init();

    /* Turn on LCD */
    lcd_on();

    /* Turn on LCD backlight to full brightness */
    lcd_led_set(0xff);

    /* Flush test message to LCD RAM */
    lcd_clear();
    lcd_flush_text(text_buffer);
    _delay_ms(1000);

    /* Scroll through test message */
    for (uint8_t i = 0; i <= 0x20; ++i) {
      _delay_ms(100);
      lcd_scroll(i);
    }
    _delay_ms(1000);

    /* Reset the LCD */
    lcd_scroll(0);
    lcd_clear();

    /* --- ASCII animal demo (before counting) --- */
    lcd_clear();

    lcd_text_buffer_t ascii_animal = {

      "", "", "", "",
      " (_(_/-(_/",
      " `_/      )",
      "o'')}____//",
      "  __      _"
    };
    lcd_flush_text(ascii_animal);
    _delay_ms(1200);

    /* gentle vertical scroll to show stability */
    for (uint8_t s = 0; s <= 0x20; ++s) {
      lcd_scroll(s);
      _delay_ms(60);
    }
    _delay_ms(5000);
    lcd_scroll(0);
    _delay_ms(1000);

    /* clean up before continuing */
    lcd_clear();

    /* Reduce LCD backlight to 25% */
    lcd_led_set(64);

    /* Erase the text buffer */
    for (uint8_t i = 0; i < 8; ++i) {
      text_buffer[i] = 0;
    }

    /* Allocate a new buffer for storing counter string */
    char buf[6] = {0};
    text_buffer[0] = buf;
    lcd_clear();

    for (uint16_t count = 0;; ++count) {
      /* Convert count to a string */
      uint16_t tmp = count;
      for (uint8_t i = 0; i < 6; ++i) {
        buf[5 - i] = tmp % 10 + '0';
        tmp /= 10;
      }
      /* Write string to LCD */
      lcd_flush_text(text_buffer);
      lcd_led_set(count % 255);
      _delay_ms(100);
    }
  }
