#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#include <wchar.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "spnav.h"

#define MT_NAME "spnav_mt"

static structSpnav * lua_checkspnav(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid spnav");

  return (structSpnav *)ud;
}

static int lua_spnav_delete(lua_State *L) {
  structSpnav *ud = lua_checkspnav(L, 1);

  free(ud->buf);
  hid_close(ud->handle);
  hid_exit();

  return 1;
}

static int lua_spnav_index(lua_State *L) {
  /* Get index through metatable: */
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} /* push metatable */
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2); /* get metatable function */
  lua_remove(L, -2); /* delete metatable */
  return 1;
}

#define MAX_STR 255
#define MAX_BUF 256

static int lua_spnav_init(lua_State *L) {
  structSpnav *ud = (structSpnav *)lua_newuserdata(L, sizeof(structSpnav));
  int res = 0;
  int vendorID = luaL_checknumber(L, 1);
  int productID = luaL_checknumber(L, 2);


  /* SpaceMouse 0x046D, 0xC626 */
  /* SpaceMouse Pro 0x046D, 0xC62b */
  ud->handle = hid_open(vendorID, productID, NULL); 
  if (!ud->handle) {
    luaL_error(L, "Unable to open SpaceMouse\n");
    return 1;
  }

  /* init buffer */
  ud->buf = (unsigned char *)malloc(MAX_BUF * sizeof(unsigned char));

  /* Read the Manufacturer String */
  wchar_t wstr[MAX_STR];
  wstr[0] = 0x0000;
  res = hid_get_manufacturer_string(ud->handle, wstr, MAX_STR);
  if (res < 0)
    fprintf(stdout, "Unable to read manufacturer string\n");
  fprintf(stdout, "SpaceMouse Manufacturer: %ls\n", wstr);

  /* read Pruduct String */
  wstr[0] = 0x0000;
  res = hid_get_product_string(ud->handle, wstr, MAX_STR);
  if (res < 0)
    fprintf(stdout, "Unable to read product string\n");
  fprintf(stdout, "SpaceMouse Product String: %ls\n", wstr);

  /* Read Serial Number */
  wstr[0] = 0x0000;
  res = hid_get_serial_number_string(ud->handle, wstr, MAX_STR);
  if (res < 0)
    fprintf(stdout, "Unable to read serial number string\n");
  fprintf(stdout, "SpaceMouse Serial Number: %ls\n", wstr);

	/* Set the hid_read() function to be non-blocking. */
	hid_set_nonblocking(ud->handle, 1);
	
	/* Try to read from the device. There shoud be no
	   data here, but execution should not block.
  */
	res = hid_read(ud->handle, ud->buf, 17);

	// Send a Feature Report to the device
	ud->buf[0] = 0x2;
	ud->buf[1] = 0xa0;
	ud->buf[2] = 0x0a;
	ud->buf[3] = 0x00;
	ud->buf[4] = 0x00;
	res = hid_send_feature_report(ud->handle, ud->buf, 17);
	if (res < 0) {
		printf("Unable to send a feature report.\n");
	}

	memset(ud->buf, 0, MAX_BUF * sizeof(unsigned char));

	// Read a Feature Report from the device
  int i = 0;
	ud->buf[0] = 0x2;
	res = hid_get_feature_report(ud->handle, ud->buf, MAX_BUF * sizeof(unsigned char));
	if (res < 0) {
		printf("Unable to get a feature report.\n");
		printf("%ls", hid_error(ud->handle));
	}
	else {
		// Print out the returned buffer.
		printf("Feature Report\n   ");
		for (i = 0; i < res; i++)
			printf("%02hhx ", ud->buf[i]);
		printf("\n");
	}

	memset(ud->buf, 0, MAX_BUF * sizeof(unsigned char));

	// Toggle LED (cmd 0x80). The first byte is the report number (0x1).
	ud->buf[0] = 0x1;
	ud->buf[1] = 0x80;
	res = hid_write(ud->handle, ud->buf, 17);
	if (res < 0) {
		printf("Unable to write()\n");
		printf("Error: %ls\n", hid_error(ud->handle));
	}
	
	// Request state (cmd 0x81). The first byte is the report number (0x1).
	ud->buf[0] = 0x1;
	ud->buf[1] = 0x81;
	hid_write(ud->handle, ud->buf, 17);
	if (res < 0)
		printf("Unable to write() (2)\n");


  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_spnav_get_raw(lua_State *L) {
  structSpnav *ud = lua_checkspnav(L, 1);

  int res = hid_read(ud->handle, ud->buf, MAX_BUF * sizeof(unsigned char));
  if (res < 0)
    luaL_error(L ,"Unable to read request\n");
  if (res==0)
    return 0;

  char * buffer = (char *)malloc(MAX_BUF * 2 * sizeof(char));
  char * bufferp = buffer;

  int i = 0, size = 0;
  for (i = 0; i < res; i++) {
    size = sprintf(bufferp, "%02hhx ", ud->buf[i]); 
    bufferp += 3;
  }

  lua_pushlstring(L, buffer, res * 3);
  free(buffer);

  return 1;
}

static int lua_spnav_get(lua_State *L) {
  structSpnav *ud = lua_checkspnav(L, 1);

  int res = hid_read(ud->handle, ud->buf, MAX_BUF * sizeof(unsigned char));
  if (res < 0)
    luaL_error(L ,"Unable to read request\n");
  if (res==0)
    return 0;

  //int event_type = ud->buf[0];
  int16_t * payload = (int16_t *)(&ud->buf[1]);
  switch (ud->buf[0]) {
    case SPNAV_EVENT_EMPTY:
      return 0;
    case SPNAV_EVENT_ROTATE:
      lua_pushstring(L, "rotate");
      lua_createtable(L, 0, 3);
    	lua_pushnumber(L, -1*payload[0] );
    	lua_setfield(L, -2, "wy");
    	lua_pushnumber(L, -1*payload[1] );
    	lua_setfield(L, -2, "wx");
    	lua_pushnumber(L, -1*payload[2] );
    	lua_setfield(L, -2, "wz");
      break;
    case SPNAV_EVENT_TRANSLATE:
      lua_pushstring(L, "translate");
      lua_createtable(L, 0, 3);
    	lua_pushnumber(L,-1*payload[0]);
      lua_setfield(L, -2, "y");
    	lua_pushnumber(L,-1*payload[1]);
      lua_setfield(L, -2, "x");
    	lua_pushnumber(L,-1*payload[2]);
      lua_setfield(L, -2, "z");
      break;
    case SPNAV_EVENT_BUTTON:
      lua_pushstring(L, "button");
      // Push the button number
    	lua_pushnumber(L,payload[0]);
      break;
    default:
      return 0;
  }
  return 2;
}

static int lua_spnav_lsusb(lua_State *L) {
  struct hid_device_info *devs, *cur_dev;
	devs = hid_enumerate(0x0, 0x0);
	cur_dev = devs;	
	while (cur_dev) {
		printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
		printf("\n");
		printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
		printf("  Product:      %ls\n", cur_dev->product_string);
		printf("  Release:      %hx\n", cur_dev->release_number);
		printf("  Interface:    %d\n",  cur_dev->interface_number);
		printf("\n");
		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);

  return 1;
}

static const struct luaL_reg spnav_functions [] = {
  {"init", lua_spnav_init},
  {"lsusb", lua_spnav_lsusb},
  {NULL, NULL}
};

static const struct luaL_reg spnav_methods [] = {
  {"get_raw", lua_spnav_get_raw},
  {"get", lua_spnav_get},
  {"__gc", lua_spnav_delete},
	{"__index", lua_spnav_index},
  {NULL, NULL}
};

int luaopen_spacemouse(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, spnav_methods, 0);
  luaL_newlib(L, spnav_functions);
#else
  luaL_register(L, NULL, spnav_methods);
  luaL_register(L, "spnav", spnav_functions);
#endif

  return 1;
}
