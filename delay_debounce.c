#include <stdint.h>

#include "delay_cycles.h"

int
delay_debounce(uint8_t volatile *port, uint8_t pin, uint16_t cycles)
{
  if ((*port) & (1 << pin)) {
  } else {
  }
  delay_cycles(cycles);
  return 0;
}
