/*
 * generic_hid.c
 *
 *  Created on: Apr 22, 2011
 *      Author: Jan Axelson
 *
 * Demonstrates communicating with a device designed for use with a generic HID-class USB device.
 * Sends and receives 2-byte reports.
 * Requires: an attached HID-class device that supports 2-byte
 * Input, Output, and Feature reports.
 * The device firmware should respond to a received report by sending a report.
 * Change VENDOR_ID and PRODUCT_ID to match your device's Vendor ID and Product ID.
 * See Lvr.com/winusb.htm for example device firmware.
 * This firmware is adapted from code provided by Xiaofan.
 * Note: libusb error codes are negative numbers.

 The application uses the libusb 1.0 API from libusb.org.
 Compile the application with the -lusb-1.0 option. 
 Use the -I option if needed to specify the path to the libusb.h header file. For example:
 -I/usr/local/angstrom/arm/arm-angstrom-linux-gnueabi/usr/include/libusb-1.0 

*/


/**
 * Lua module for usb joystick interface
 */

#include "luaxbox360.h"

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
static pthread_t xbThread;

/////////////////////////////

// For XBOX controller
static const int VENDOR_ID = 0x045e;
static const int PRODUCT_ID = 0x028e;

static const int INTERRUPT_IN_ENDPOINT = 0x81;
static const int INTERRUPT_OUT_ENDPOINT = 0x02;

// With firmware support, transfers can be > the endpoint's max packet size.

static const int MAX_INTERRUPT_IN_TRANSFER_SIZE = 40;
static const int MAX_INTERRUPT_OUT_TRANSFER_SIZE = 32; // Seems not to matter how large this is...

// Values for bmRequestType in the Setup transaction's Data packet.

static const int CONTROL_REQUEST_TYPE_IN = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE;
static const int CONTROL_REQUEST_TYPE_OUT = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE;

// From the HID spec:

static const int HID_GET_REPORT = 0x01;
static const int HID_SET_REPORT = 0x09;
static const int HID_REPORT_TYPE_INPUT = 0x01;
static const int HID_REPORT_TYPE_OUTPUT = 0x02;
static const int HID_REPORT_TYPE_FEATURE = 0x03;

// With firmware support, transfers can be > the endpoint's max packet size.

static const int MAX_CONTROL_IN_TRANSFER_SIZE = 2;
static const int MAX_CONTROL_OUT_TRANSFER_SIZE = 2;

static const int INTERFACE_NUMBER = 0;
static const int TIMEOUT_MS = 5000;

// Dev handle
struct libusb_device_handle *devh = NULL;

int xbox360_thread_init() {
  if (init) {
    // a joystick is already open, close it first
    xbox360_thread_cleanup();
  } 
  ///////////////////
  // INIT

  int result;

  result = libusb_init(NULL);
  if (result < 0)
  {
    fprintf(stderr, "Unable to initialize libusb.\n");
    return -1;
  }
  devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);

  if (devh != NULL)
  {
    // The HID has been detected.
    // Detach the hidusb driver from the HID to enable using libusb.

    libusb_detach_kernel_driver(devh, INTERFACE_NUMBER);
    result = libusb_claim_interface(devh, INTERFACE_NUMBER);
    if (result < 0) {
      fprintf(stderr, "libusb_claim_interface error %d\n", result);
      return -1;
    }
  }
  else
  {
    fprintf(stderr, "Unable to find the device.\n");
    return -1;
  }
  ///////////////




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
  printf("creating xbox360 thread\n");
  running = 1;
	int ret;
  ret = pthread_create(&xbThread, NULL, xbox360_thread_func, NULL);
  if (ret != 0) {
    fprintf(stderr, "error creating joystick thread: %d\n", ret);
    return -1;
  }

  init = 1;

  return 0;
}

int xbox360_thread_cleanup() {
  printf("cleaning up xbox360 thread\n");
  if (init) {
    // set initialized to false
    init = 0;

    libusb_release_interface(devh, 0);
    libusb_close(devh);
    libusb_exit(NULL);

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


void *xbox360_thread_func(void *) {
  printf("starting xbox360 thread\n");
	
	int bytes_transferred;
	int result = 0;
	int i = 0;

 	unsigned char data_in[20]; // Reports are 20 bytes for 360

  /*
     sigset_t sigs;
     sigfillset(&sigs);
     pthread_sigmask(SIG_BLOCK, &sigs, NULL);
     */

  while (!stopRequest) {
    // Read data from the device.
    result = libusb_interrupt_transfer(
        devh,
        INTERRUPT_IN_ENDPOINT,
        data_in,
        MAX_INTERRUPT_OUT_TRANSFER_SIZE,
        &bytes_transferred,
        0); // Timeout of 0 waits forever...

    if (result >= 0)
    {
      if (bytes_transferred > 0)
      {
        printf("Data received via interrupt transfer:\n");
        for(i = 0; i < bytes_transferred; i++)
        {
          printf("%02x ",data_in[i]);
        }
        printf("\n");
      }
      else
      {
        fprintf(stderr, "No data received in interrupt transfer (%d)\n", result);
      }
    }
    else
    {
			// Timeout occurs -7
      //fprintf(stderr, "Error receiving data via interrupt transfer %d\n", result);
    }
		/*
    buttons[event.number] = event.value;
    tbutton[event.number] = event.time;
    axes[event.number] = event.value;
    taxis[event.number] = event.time;
		*/
  }

  xbox360_thread_cleanup();
  running = 0;
}


static int lua_joystick_open(lua_State *L) {
  return xbox360_thread_init();
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


static const struct luaL_reg xbox360_lib [] = {
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
int luaopen_xbox360 (lua_State *L) {
  luaL_register(L, "xbox360", xbox360_lib);

  return 0;
}
