/**
 * Lua module for usb joystick interface
 */

#include "luajoystick.h"

// initialized flag
int init = 0;
int running = 0;
int stopRequest = 0;
int jsFD;

// number of buttons and axes
char nbutton;
char naxis;
// arrays for button and axis data
int16_t *buttons;
int16_t *axes;
// time of last event (in milliseconds)
uint32_t *tbutton; 
uint32_t *taxis; 

// thread variables
static pthread_t jsThread;


int joystick_thread_init(char *dev) {
  if (init) {
    // a joystick is already open, close it first
    joystick_thread_cleanup();
  } 
  int ret;

  // open joystick device
  printf("opening joystick: %s...", dev); fflush(stdout);
  jsFD = open(dev, O_RDONLY);
  printf("fd: %d...done\n", jsFD); fflush(stdout);

  // query the number of buttons and axes
  char name[256];
  ret = ioctl(jsFD, JSIOCGNAME(256), name);
  if (ret < 0) {
    fprintf(stderr, "error querying device identifier string: %d\n", ret);
    return ret;
  }
  ret = ioctl(jsFD, JSIOCGBUTTONS, &nbutton);
  if (ret < 0) {
    fprintf(stderr, "error querying number of buttons: %d\n", ret);
    return -1;
  }
  ret = ioctl(jsFD, JSIOCGAXES, &naxis);
  if (ret < 0) {
    fprintf(stderr, "error querying number of axes: %d\n", ret);
  }

  printf("opened %s with %d buttons and %d axes\n", name, nbutton, naxis);

  // allocate array data
  buttons = (int16_t *)malloc(nbutton * sizeof(int16_t));
  if (!buttons) {
    fprintf(stderr, "unable to allocate button data array\n");
    return -1;
  }
  axes = (int16_t *)malloc(naxis * sizeof(int16_t));
  if (!axes) {
    fprintf(stderr, "unable to allocate axes data array\n");
    return -1;
  }
  tbutton = (uint32_t *)malloc(nbutton * sizeof(uint32_t));
  if (!tbutton) {
    fprintf(stderr, "unable to allocate button time array\n");
    return -1;
  }
  taxis = (uint32_t *)malloc(naxis * sizeof(uint32_t));
  if (!taxis) {
    fprintf(stderr, "unable to allocate axes time array\n");
    return -1;
  }

  // start receiver thread
  printf("creating joystick thread\n");
  running = 1;
  ret = pthread_create(&jsThread, NULL, joystick_thread_func, NULL);
  if (ret != 0) {
    fprintf(stderr, "error creating joystick thread: %d\n", ret);
    return -1;
  }

  init = 1;

  return 0;
}

int joystick_thread_cleanup() {
  printf("cleaning up joystick thread\n");
  if (init) {
    // set initialized to false
    init = 0;

    // close the fd
    close(jsFD);
    // free arrays
    nbutton = 0;
    naxis = 0;
    free(buttons);
    free(axes);
    free(tbutton);
    free(taxis);
  }

  return 0;
}


void *joystick_thread_func(void *) {
  printf("starting joystick thread\n");

  /*
  sigset_t sigs;
  sigfillset(&sigs);
  pthread_sigmask(SIG_BLOCK, &sigs, NULL);
  */

  while (!stopRequest) {
    struct js_event event;
    int nrd = read(jsFD, &event, sizeof(struct js_event));
    if (nrd != sizeof(struct js_event)) {
      fprintf(stderr, "read %d bytes but expected %lu bytes\n", nrd, sizeof(struct js_event));
      break;
    }

    if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
      buttons[event.number] = event.value;
      tbutton[event.number] = event.time;
    }
    if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
      axes[event.number] = event.value;
      taxis[event.number] = event.time;
    }

    // sleep for 1ms
    usleep(1000);
  }

  joystick_thread_cleanup();
  running = 0;
}


static int lua_joystick_open(lua_State *L) {
  const char *dev = luaL_optstring(L, 1, "/dev/input/js0");

  return joystick_thread_init((char *)dev);
}


static int lua_joystick_close(lua_State *L) {
  // stop thread
  stopRequest = 1;

  // wait for it to actually stop
  while (running) {
    usleep(1000);
  }
  stopRequest = 0;

  return 0;
}


static int lua_joystick_num_buttons(lua_State *L) {
  lua_pushinteger(L, nbutton);

  return 1;
}


static int lua_joystick_num_axes(lua_State *L) {
  lua_pushinteger(L, naxis);

  return 1;
}


static int lua_joystick_button(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind == -1) {
    // return array will all button states
    lua_createtable(L, nbutton, 0);
    for (int i = 0; i < (int)nbutton; i++) {
      lua_pushinteger(L, buttons[i]);
      lua_rawseti(L, -2, i+1);
    }
  } else {
    // return only indexed button state
    lua_pushinteger(L, buttons[ind-1]);
  }
  return 1;
}


static int lua_joystick_axis(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind == -1) {
    // return array will all axes states
    lua_createtable(L, naxis, 0);
    for (int i = 0; i < (int)naxis; i++) {
      lua_pushinteger(L, axes[i]);
      lua_rawseti(L, -2, i+1);
    }
  } else {
    // return only indexed axes state
    lua_pushinteger(L, axes[ind-1]);
  }
  return 1;
}


static int lua_joystick_button_time(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind == -1) {
    // return array will all button states
    lua_createtable(L, nbutton, 0);
    for (int i = 0; i < (int)nbutton; i++) {
      lua_pushinteger(L, tbutton[i]);
      lua_rawseti(L, -2, i+1);
    }
  } else {
    // return only indexed button state
    lua_pushinteger(L, tbutton[ind-1]);
  }
  return 1;
}


static int lua_joystick_axis_time(lua_State *L) {
  int ind = luaL_optinteger(L, 1, -1);
  if (ind == -1) {
    // return array will all axes states
    lua_createtable(L, naxis, 0);
    for (int i = 0; i < (int)naxis; i++) {
      lua_pushinteger(L, taxis[i]);
      lua_rawseti(L, -2, i+1);
    }
  } else {
    // return only indexed axes state
    lua_pushinteger(L, taxis[ind-1]);
  }
  return 1;
}


static const struct luaL_reg joystick_lib [] = {
  {"open", lua_joystick_open},
  {"close", lua_joystick_close},
  {"button", lua_joystick_button},
  {"axis", lua_joystick_axis},
  {"button_time", lua_joystick_button_time},
  {"axis_time", lua_joystick_axis_time},
  {"num_buttons", lua_joystick_num_buttons},
  {"num_axes", lua_joystick_num_axes},

  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_joystick (lua_State *L) {
  luaL_register(L, "joystick", joystick_lib);

  return 0;
}

