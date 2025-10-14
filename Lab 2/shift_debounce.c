#include <stdint.h>

int shift_debounce(uint8_t volatile *port, uint8_t pin)
{
  // Seed with an alternating pattern chosen by the first observed bit.
  int first = ((*port) & (1u << pin)) ? 1 : 0;
  uint16_t shift_reg = first ? 0xAAAAu : 0x5555u;

  for (;;) {
    int bit = ((*port) & (1u << pin)) ? 1 : 0;
    shift_reg = (uint16_t)((shift_reg << 1) | (uint16_t)bit);

    if (shift_reg == 0xFFFFu) return 1;
    if (shift_reg == 0x0000u) return 0;
  }
}
