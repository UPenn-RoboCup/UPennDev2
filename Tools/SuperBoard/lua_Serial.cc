#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include "SerialDevice.hh"
#include <stdint.h>
#include <vector>
#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

static SerialDevice * pDev = NULL;

std::vector<uint8_t> charBuf;

static int lua_Serial_connect(lua_State *L) {
	if (pDev != NULL) {
		std::cout << "serialDeviceAPI: Port is already open" << std::endl;
    lua_pushinteger(L, 0);
		return 1;
	}
  
  if (lua_isnoneornil(L, 2)) 
    luaL_error(L, "SerialDevice: Please enter correct arguments: <device>, <baud rate>");

  string deviceName(luaL_checkstring(L, 1)); 

	if (deviceName.length() < 1) {
		luaL_error(L, "serialDeviceAPI: Could not read string while reading the device name");
	}

  int baud = luaL_checkint(L, 2);
	pDev = new SerialDevice();

	//connect to the device and set IO mode (see SerialDevice.hh for modes)
	if (pDev->Connect(deviceName.c_str(),baud) || pDev->Set_IO_BLOCK_W_TIMEOUT())
  {
		delete pDev;
		pDev=NULL;
		luaL_error(L, "Could not open device");
	}
	
	std::cout << "serialDeviceAPI: Connected to device: "<<deviceName << std::endl;
  lua_pushinteger(L, 0);
	return 1;
}

static int lua_Serial_read(lua_State *L) {
		//how many chars to read?		
    int len = luaL_checkint(L, 1);
		charBuf.resize(len); //make sure there is enough space for data

		//timeout in microseconds
    int timeout = luaL_checkint(L, 2);

		int numRead=pDev->ReadChars((char*)&(charBuf[0]),len,timeout);

		if (numRead >= 0){
//			std::cout << "serialDeviceAPI: Read "<<numRead<<" chars"<< std::endl;
      lua_pushlightuserdata(L, &charBuf[0]);
      lua_pushstring(L, "uint8_t");
      lua_pushinteger(L, numRead);
      return 3;
		}
		else{
			std::cout << "serialDeviceAPI: ERROR: could not read chars"<< std::endl;
      lua_pushinteger(L, 0);
      return 1;
		}
		return 1;
}

static int lua_Serial_write(lua_State *L) {
    if (!pDev)
      luaL_error(L, "SerialDevice: not connected to device");

    if (lua_isnoneornil(L, 2))
      luaL_error(L, "SerialDevice: need two argumets");

    uint8_t *data = (uint8_t *) lua_touserdata(L, 1); 
    if ((data == NULL) || !lua_islightuserdata(L, 1)) {
      return luaL_error(L, "Input string not light user data");
    }
    int len = sizeof(data) / sizeof(uint8_t);

    if (pDev->WriteChars((char*)data,len) == len){
      //std::cout << "serialDeviceAPI: Wrote "<<len<<" chars"<< std::endl;
      lua_pushinteger(L, 1);
    }
    else {
      std::cout << "serialDeviceAPI: ERROR: Could not write "<<len<<" chars"<< std::endl;
      lua_pushinteger(L, 0);
    }
    return 1;
}

static int lua_Serial_shutdown(lua_State *L) {
	printf("Exiting SerialDevice\n"); fflush(stdout);
	if (pDev != NULL) delete pDev;
  exit(0);
  return 1;
}

static const struct luaL_reg Serial_lib [] = {
  {"connect", lua_Serial_connect},
  {"read", lua_Serial_read},
  {"write", lua_Serial_write},
  {"shutdown", lua_Serial_shutdown},
  {NULL, NULL}
};

extern "C"
int luaopen_Serial(lua_State *L) {
  luaL_register(L, "Serial", Serial_lib);
  return 1;
}
