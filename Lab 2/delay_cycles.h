#ifndef DELAY_CYCLES_H
#define DELAY_CYCLES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// wait for N + 8*cycles CPU cycles, where N is a fixed overhead (9 cycles)
// inside this function (not including the caller's CALL instruction).
void delay_cycles(uint16_t cycles);

#ifdef __cplusplus
}
#endif

#endif // DELAY_CYCLES_H