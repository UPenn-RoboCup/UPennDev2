#include "config.h"

#include "DebugInterface.h"

int DebugInit()
{
  DEBUG_COM_PORT_INIT();
  DEBUG_COM_PORT_SETBAUD(DEBUG_BAUD_RATE);

  return 0;
}
