#ifndef __LUAJOYSTICK_H__
#define __LUAJOYSTICK_H__

#ifdef __cplusplus
extern "C"
{
#endif
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#ifdef __cplusplus
}
#endif

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <libusb.h>

#include <sys/time.h>
#include <sys/types.h>

int xbox360_thread_init();

int xbox360_thread_cleanup();

void *xbox360_thread_func(void *);

#endif
