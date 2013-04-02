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

#include <usb.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>

#include <sys/time.h>
#include <sys/types.h>

// TODO: switch for mac driver
#include <linux/joystick.h>


int joystick_thread_init(char *dev);

int joystick_thread_cleanup();

void *joystick_thread_func(void *);

#endif
