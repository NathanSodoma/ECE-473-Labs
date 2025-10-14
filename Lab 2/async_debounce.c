#include <stdint.h>



void async_debounce(uint8_t volatile *port, uint8_t pin, int8_t *counter)
{


  int val = !!((*port) & (1 << pin));
  *counter += 2*val - 1;

  if((*counter < 0) && (val == 1)){
    *counter = 0;
  }

  if((*counter > 0) && (val == 0)){
    *counter = 0;
  }

  if((val ==1) && (*counter < 64)){
    *counter += 1;
    return;
  }

  if((val == 0) && (*counter > -64)){
    *counter -= 1;
  }

  
}
