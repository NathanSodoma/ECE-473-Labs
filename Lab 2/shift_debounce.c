#include <stdint.h>

int
shift_debounce(uint8_t volatile *port, uint8_t pin)
{
  return !!((*port) & (1 << pin));
}
