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

static int lua_Serial_imuparser(lua_State *L) {
  uint8_t *data = (uint8_t *) lua_touserdata(L, 1); 
  if ((data == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input string not light user data");
  }
  int len = luaL_checkint(L, 2);  
////////////////////////
//cout << len << endl;
//for (int cnt = 0; cnt < len; cnt ++) 
//  cout << (unsigned int)data[cnt] << ' ';
//cout << endl;
////////////////////////
  uint32_t tuc = data[8] << 24 | data[7] << 16 | data[6] << 8 | data[5];
  double id = static_cast<double>(data[9]);
  double cntr = static_cast<double>(data[10]);

  int16_t rpy = 0;
  double rpy_gain = 5000;
  rpy = data[12] << 8 | data[11];
  double r = static_cast<double>(rpy) / rpy_gain;
  rpy = data[14] << 8 | data[13];
  double p = static_cast<double>(rpy) / rpy_gain;
  rpy = data[16] << 8 | data[15];
  double y = static_cast<double>(rpy) / rpy_gain;

  int16_t wrpy = 0;
  double wrpy_gain = 500;
  wrpy = data[18] << 8 | data[17]; 
  double wr = static_cast<double>(wrpy) / wrpy_gain;
  wrpy = data[20] << 8 | data[19];
  double wp = static_cast<double>(wrpy) / wrpy_gain;
  wrpy = data[22] << 8 | data[21];
  double wy = static_cast<double>(wrpy) / wrpy_gain;

  int16_t acc = 0;
  double acc_gain = 5000;
  acc = data[24] << 8 | data[23];
  double ax = static_cast<double>(acc) / acc_gain;
  acc = data[26] << 8 | data[25];
  double ay = static_cast<double>(acc) / acc_gain;
  acc = data[28] << 8 | data[27];
  double az = static_cast<double>(acc) / acc_gain;

//  cout << tuc << ' ' << id << ' ' << cntr << endl;
//  cout << r << ' ' << p << ' ' << y << endl;
//  cout << wr << ' ' << wp << ' ' << wy << endl;
//  cout << ax << ' ' << ay << ' ' << az << endl;

  lua_createtable(L, 0, 1);
  lua_pushstring(L, "tuc");
  lua_pushinteger(L, tuc);
  lua_settable(L, -3);

  lua_pushstring(L, "id");
  lua_pushnumber(L, id);
  lua_settable(L, -3);

  lua_pushstring(L, "cntr");
  lua_pushnumber(L, cntr);
  lua_settable(L, -3);

  lua_pushstring(L, "rpy");
  lua_createtable(L, 3, 0);
  lua_pushnumber(L, r);
  lua_rawseti(L, -2, 1);
  lua_pushnumber(L, p);
  lua_rawseti(L, -2, 2);
  lua_pushnumber(L, y);
  lua_rawseti(L, -2, 3);
  lua_settable(L, -3);

  lua_pushstring(L, "wrpy");
  lua_createtable(L, 3, 0);
  lua_pushnumber(L, wr);
  lua_rawseti(L, -2, 1);
  lua_pushnumber(L, wp);
  lua_rawseti(L, -2, 2);
  lua_pushnumber(L, wy);
  lua_rawseti(L, -2, 3);
  lua_settable(L, -3);

  lua_pushstring(L, "acc");
  lua_createtable(L, 3, 0);
  lua_pushnumber(L, ax);
  lua_rawseti(L, -2, 1);
  lua_pushnumber(L, ay);
  lua_rawseti(L, -2, 2);
  lua_pushnumber(L, az);
  lua_rawseti(L, -2, 3);
  lua_settable(L, -3);

  return 1;
}

static const struct luaL_reg Serial_lib [] = {
  {"connect", lua_Serial_connect},
  {"read", lua_Serial_read},
  {"write", lua_Serial_write},
  {"shutdown", lua_Serial_shutdown},
  {"imuparser", lua_Serial_imuparser},
  {NULL, NULL}
};

extern "C"
int luaopen_Serial(lua_State *L) {
  luaL_register(L, "Serial", Serial_lib);
  return 1;
}
