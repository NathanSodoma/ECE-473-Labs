#include <stdint.h>



void async_debounce(uint8_t volatile *port, uint8_t pin, int8_t *counter)
{


  int val = ((*port) & (1u << pin)) ? 1 : 0;

    // Reset counter when signal direction changes
    if ((*counter < 0) && (val == 1)) {
        *counter = 0;
    } else if ((*counter > 0) && (val == 0)) {
        *counter = 0;
    }

    // Move toward saturation limits (+64 or -64)
    if (val == 1) {
        if (*counter < 64)
            (*counter)++;
    } else {
        if (*counter > -64)
            (*counter)--;
    }
  
}
