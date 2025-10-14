#include <stdint.h>

#include "delay_cycles.h"

int
delay_debounce(uint8_t volatile *port, uint8_t pin, uint16_t cycles)
{
  for (;;) {
    int first  = ((*port) & (1u << pin)) ? 1 : 0;
    delay_cycles(cycles);
    int second = ((*port) & (1u << pin)) ? 1 : 0;
    if (first == second) {
      return second; // debounced value
    }
    // else: loop until two consecutive reads match
  }
}
