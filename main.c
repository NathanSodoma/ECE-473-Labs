#include <avr/io.h>

#include "delay_debounce.h"
#include "shift_debounce.h"
#include "async_debounce.h"

int
main()
{
  /* Set up Port B for LED output */
  DDRB = 0xff;
  PORTB = 0x00;

  /* Set up Port D for Input */
  DDRD = 0x00;
  PORTD = 0xff; /* Pull-up pins */

  int8_t pd6_cnt = 0;
  uint8_t pd6_state = 0;
  for (;;) {
    delay_debounce(&PORTD, 4, 100);

    shift_debounce(&PORTD, 5);

    async_debounce(&PORTD, 6, &pd6_cnt);
    if (pd6_cnt > 32 && pd6_state == 0) {
      pd6_state = 1;
      PORTB = ~PORTB;
    } else if (pd6_cnt < 32 && pd6_state == 1) {
      pd6_state = 0;
    }
  }
}
