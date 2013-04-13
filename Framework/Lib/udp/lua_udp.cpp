#include <iostream>
#include <string>
#include <deque>
#include "string.h"
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <assert.h>

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

//#include <common.h>

#include <vector>
#include <algorithm>
#include <stdint.h>

#define MDELAY 2
#define TTL 16
#define MAX_LENGTH 160000 //Size for sending 640*480 yuyv data without resampling

const int maxQueueSize = 12;
static std::deque<std::string> recvQueue;

// TODO: Make more file descriptors, which are saved in the Lua metatable
static bool init = false;
static int send_fd, recv_fd;
static std::string IP;
static int PORT = 0;

// TODO: Close file descriptors
/*
   void mexExit(void)
   {
   if (send_fd > 0)
   close(send_fd);
   if (recv_fd > 0)
   close(recv_fd);
   }
   */

static int lua_comm_init(lua_State *L) {

  if( init ){
    printf("Already initialized!\n");
    lua_pushnil(L);
    return 1;
  }

  const char *ip = luaL_checkstring(L, 1);
  int port = luaL_checkint(L,2);
  IP = ip;
  PORT = port;

  struct hostent *hostptr = gethostbyname(IP.c_str());
  if (hostptr == NULL) {
    printf("Could not get hostname\n");
    lua_pushnil(L);
    return 1;
  }

  send_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_fd < 0) {
    printf("Could not open datagram send socket\n");
    lua_pushnil(L);
    return 1;
  }

  int i = 1;
  if (setsockopt(send_fd, SOL_SOCKET, SO_BROADCAST, (const char *) &i, sizeof(i)) < 0) {
    printf("Could not set broadcast option\n");
    lua_pushnil(L);
    return 1;
  }

  struct sockaddr_in dest_addr;
  bzero((char *) &dest_addr, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  bcopy(hostptr->h_addr, (char *) &dest_addr.sin_addr, hostptr->h_length);
  dest_addr.sin_port = htons(PORT);

  if (connect(send_fd, (struct sockaddr *) &dest_addr, sizeof(dest_addr)) < 0) {
    printf("Could not connect to destination address\n");
    lua_pushnil(L);
    return 1;
  }

  recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (recv_fd < 0) {
    printf("Could not open datagram recv socket\n");
    lua_pushnil(L);
    return 1;
  }

  struct sockaddr_in local_addr;
  bzero((char *) &local_addr, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(PORT);
  if (bind(recv_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) < 0) {
    printf("Could not bind to port\n");
    lua_pushnil(L);
    return 1;
  }

  // Nonblocking receive:
  int flags  = fcntl(recv_fd, F_GETFL, 0);
  if (flags == -1) 
    flags = 0;
  if (fcntl(recv_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    printf("Could not set nonblocking mode\n");
    lua_pushnil(L);
    return 1;
  }

  // TODO: set at exit
  init = true;
  lua_pushinteger(L, 1);
  return 1;
}

static int lua_comm_update(lua_State *L) {
  static sockaddr_in source_addr;
  static char data[MAX_LENGTH];

  // Check whether initiated
  // TODO: associate checks with the metatable
  assert(init);

  // Process incoming messages:
  socklen_t source_addr_len = sizeof(source_addr);
  int len = recvfrom(recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
  while (len > 0) {
    std::string msg((const char *) data, len);
    recvQueue.push_back(msg);
    len = recvfrom(recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
  }

  // Remove older messages
  while (recvQueue.size() > maxQueueSize)
    recvQueue.pop_front();

  return 1;
}

static int lua_comm_size(lua_State *L) {
  int updateRet = lua_comm_update(L);
  lua_pushinteger(L, recvQueue.size());
  return 1;
}

static int lua_comm_receive(lua_State *L) {
  // Perform an update to receive data
  int updateRet = lua_comm_update(L);

  // If empty, then return nil
  if (recvQueue.empty()) {
    lua_pushnil(L);
    return 1;
  }

  // Push a light string with the incoming data from the queue
  lua_pushlstring(L, recvQueue.front().c_str(), recvQueue.front().size());

  // Remove this object from the receiving buffer queue
  recvQueue.pop_front();

  return 1;
}


static int lua_comm_send(lua_State *L) {
  // Grab the arguments
  const char *data = luaL_checkstring(L, 1);
  int size = luaL_optint(L, 2, 0);
  if(size==0)
    size = strlen(data);
  else if(size<0) {
    // Cannot send a negative number of bytes
    // TODO: add a maximum limit
    lua_pushnil(L);
    return 1;
  }
  // Process the update
  int updateRet = lua_comm_update(L);
  // Send the data
  int ret = send(send_fd, data, size, 0);
  // Push the return value
  lua_pushinteger(L, ret);
  return 1;
}

static const struct luaL_reg udp [] = {
  {"init", lua_comm_init},
  {"size", lua_comm_size},
  {"receive", lua_comm_receive},
  {"send", lua_comm_send},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_udp (lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, udp);
#else
  luaL_register(L, "udp", udp);
#endif  
  return 1;
}
