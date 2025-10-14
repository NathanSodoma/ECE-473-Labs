#include <stdint.h>
void async_debounce(uint8_t volatile *port, uint8_t pin, int8_t *counter)
{
  int val = !!((*port) & (1 << pin));
  *counter += 2*val - 1;
}
