#include "timeScalar.h"
#include "string.h"
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "RoboCupGameControlData.h"

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

static int sock_fd = 0;
static struct RoboCupGameControlData gameControlData;
static int nGameControlData = 0;
static double recvTime = 0;

static int lua_gamecontrolpacket_parse(lua_State *L, RoboCupGameControlData *data) {
  if (data == NULL) {
    return 0;
  }

  if (strncmp(data->header, GAMECONTROLLER_STRUCT_HEADER, 4) != 0) {
    return 0;
  }

  lua_createtable(L, 0, 3);  
	// time
  lua_pushstring(L, "time");
  lua_pushnumber(L, recvTime);
  lua_settable(L, -3);
  // version 
  lua_pushstring(L, "version");
  lua_pushnumber(L, data->version);
  lua_settable(L, -3);
	// state
  lua_pushstring(L, "state");
  lua_pushnumber(L, data->state);
  lua_settable(L, -3);

  return 1;
}

static int lua_gamecontrolpacket_receive(lua_State *L) {
  const int MAX_LENGTH = 4096;
  static char data[MAX_LENGTH];
  static bool init = false;

  if (!init) {
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
      return luaL_error(L, "Could not open datagram socket\n");
    }

    struct sockaddr_in local_addr;
    bzero((char *) &local_addr, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(GAMECONTROLLER_PORT);
    if (bind(sock_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) < 0) {
      return luaL_error(L, "Could not bind to port\n");
    }

    // Nonblocking receive:
    int flags  = fcntl(sock_fd, F_GETFL, 0);
    if (flags == -1)
      flags = 0;
    if (fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      return luaL_error(L, "Could not set nonblocking mode\n");
    }

    init = true;
  }


  // Process incoming game controller messages:
  static sockaddr_in source_addr;
  socklen_t source_addr_len = sizeof(source_addr);
  int len = recvfrom(sock_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);

  while (len > 0) {
    //printf("Packet: %d bytes\n", len);

    // Verify game controller header:
    if (memcmp(data, GAMECONTROLLER_STRUCT_HEADER, sizeof(GAMECONTROLLER_STRUCT_HEADER) - 1) == 0) {
      memcpy(&gameControlData, data, sizeof(RoboCupGameControlData));    
      nGameControlData++;
      //printf("Game control: %d received.\n", nGameControlData);
    }
    len = recvfrom(sock_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);

		recvTime = time_scalar();
  }

  if (nGameControlData == 0) {
    // no messages received yet
    return 0;
  } else {
    return lua_gamecontrolpacket_parse(L, &gameControlData); 
  }

}

static const struct luaL_reg gamecontrol_receiver_lib [] = {
  {"receive", lua_gamecontrolpacket_receive},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_GameControlReceiver (lua_State *L) {
  luaL_register(L, "GameControlReceiver", gamecontrol_receiver_lib);

  return 1;
}
