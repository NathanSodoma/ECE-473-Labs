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
  uint8_t pd4_state = 0;
  uint8_t pd5_state = 0;

  for (;;) {
    int d = delay_debounce(&PIND, 4, 100);
    if(d == 0) {
      pd4_state = 0;
    }else if(d == 1 && pd4_state == 0) {
      pd4_state = 1;
      PORTB = ~PORTB;
    }

    int s = shift_debounce(&PIND, 5);
    if (s == 0) {
      pd5_state = 0;
    } else if (s == 1 && pd5_state == 0) {
      pd5_state = 1;
      PORTB = ~PORTB;
    }

    async_debounce(&PIND, 6, &pd6_cnt);
    if (pd6_cnt > 32 && pd6_state == 0) {
      pd6_state = 1;
      PORTB = ~PORTB;
    } else if (pd6_cnt < 32 && pd6_state == 1) {
      pd6_state = 0;
    }
  }
}
