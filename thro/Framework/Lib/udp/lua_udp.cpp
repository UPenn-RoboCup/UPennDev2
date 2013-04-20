/*
Lua file to send and receive UDP messages.
Daniel D. Lee copyright 2009 <ddlee@seas.upenn.edu>
Stephen G. McGill copyright 2013 <smcgill3@seas.upenn.edu>
*/

#include <iostream>
#include <sstream>
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
#include <vector>
#include <algorithm>
#include <stdint.h>

// Error printing
#include <errno.h>

// C++ include
#include "lua.hpp"

#define MDELAY 2
#define TTL 16
//Size for sending 640*480 yuyv data without resampling
#define MAX_LENGTH 160000

#define MT_NAME "udp_mt"

typedef struct {
  bool init_recv;
  int recv_fd;
  int recv_port;
  bool init_send;
  int send_fd;
  int send_port;
  const char * send_ip;
  std::deque<std::string> *recvQueue;
} structUdp;

const int maxQueueSize = 12;

static structUdp * lua_checkudp(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid Udp udata");
  return (structUdp *)ud;
}

static int lua_udp_close(lua_State *L) {
  structUdp *p = lua_checkudp(L, 1);

  if (p->recv_fd > 0 && p->init_recv) {
    close(p->recv_fd);
    p->init_recv = false;
    p->recvQueue->clear();
    delete p->recvQueue;
    lua_pushboolean(L, 1);
    return 1;
  }

  if (p->send_fd > 0 && p->init_send) {
    close(p->send_fd);
    p->init_send = false;
    lua_pushboolean(L, 1);
    return 1;
  }

	lua_pushboolean(L,0);
	return 1;
}

static int lua_udp_init_recv(lua_State *L) {

  structUdp *ud = (structUdp *)lua_newuserdata(L, sizeof(structUdp)); 
  ud->init_recv = false;
  ud->init_send = false;

	if( ud->init_recv )
		return luaL_error(L,"Already initialized receiver!\n");

	// Grab port to listen on (all interfaces)
	ud->recv_port = luaL_checkint(L,1);

	ud->recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (ud->recv_fd < 0)
		return luaL_error(L,"Could not open datagram recv socket!\n");
  ud->recvQueue = new std::deque<std::string>;

	struct sockaddr_in local_addr;
	bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	local_addr.sin_port = htons(ud->recv_port);
	if (bind(ud->recv_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) < 0)
		return luaL_error(L,"Could not bind to port\n");

	// Nonblocking receive
	int flags  = fcntl(ud->recv_fd, F_GETFL, 0);
	if (flags == -1) 
		flags = 0;
	if ( fcntl(ud->recv_fd, F_SETFL, flags|O_NONBLOCK)<0 )
		return luaL_error(L,"Could not set nonblocking mode\n");

	// TODO: Store in an array, so we can have multiple ports
	ud->init_recv = true;

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	return 1;
}

static int lua_udp_init_send(lua_State *L) {

  structUdp *ud = (structUdp *)lua_newuserdata(L, sizeof(structUdp)); 
  ud->init_send = false;
  ud->init_recv = false;

	if (ud->init_send)
		return luaL_error(L,"Already initialized sender!\n");
	
	// TODO: check if already assigned an ip/port combo?
	ud->send_ip = luaL_checkstring(L, 1);//std::string
	ud->send_port = luaL_checkint(L,2);

	// Where to send the data
	struct hostent *hostptr = gethostbyname( ud->send_ip );
	if (hostptr == NULL)
		return luaL_error(L,"Could not get hostname\n");

	ud->send_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (ud->send_fd < 0)
		return luaL_error(L,"Could not open datagram send socket\n");
  else
    ud->init_send = true;

	int i = 1;
	if (setsockopt(ud->send_fd, SOL_SOCKET, SO_BROADCAST, 
	(const char *) &i, sizeof(i)) < 0)
		return luaL_error(L,"Could not set broadcast option\n");
	i = 1;
	if (setsockopt(ud->send_fd, SOL_SOCKET, SO_REUSEADDR,
	(const char *) &i, sizeof(i)) < 0)
		return luaL_error(L,"Could not set reuse option");

	struct sockaddr_in dest_addr;
	bzero((char *) &dest_addr, sizeof(dest_addr));
	dest_addr.sin_family = AF_INET;
	bcopy(hostptr->h_addr, (char *) &dest_addr.sin_addr, hostptr->h_length);
	dest_addr.sin_port = htons(ud->send_port);

	if (connect(ud->send_fd, (struct sockaddr *) &dest_addr, sizeof(dest_addr)) < 0)
		return luaL_error(L,"Could not connect to destination address\n");

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	return 1;
}

static int comm_update(structUdp *p) {

	static sockaddr_in source_addr;
	static char data[MAX_LENGTH];

	// Check whether initiated
	// TODO: associate checks with the metatable
	assert( p->init_recv );
	// Process incoming messages:
	socklen_t source_addr_len = sizeof(source_addr);
	int len = 
		recvfrom(p->recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
	while (len > 0) {
		std::string msg((const char *) data, len);
		p->recvQueue->push_back(msg);
		len = recvfrom(p->recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
	}

	// Remove older messages
	while (p->recvQueue->size() > maxQueueSize){
    p->recvQueue->pop_front();
  }

	return p->recvQueue->size();
}

static int lua_udp_size(lua_State *L) {
  structUdp *p = lua_checkudp(L, 1);
	int updateRet = comm_update(p);
	lua_pushinteger( L, updateRet );
	return 1;
}

static int lua_udp_receive(lua_State *L) {

  structUdp *p = lua_checkudp(L, 1);

	// Perform an update to receive data
	int updateRet = comm_update(p);

	// If empty, then return nil
	if (p->recvQueue->empty()) {
		lua_pushnil(L);
		return 1;
	}

	// Push a light string with the incoming data from the queue
	lua_pushlstring(L, p->recvQueue->front().c_str(), p->recvQueue->front().size());
	// Remove this object from the receiving buffer queue
	p->recvQueue->pop_front();

	return 1;
}

static int lua_udp_send(lua_State *L) {

  structUdp *p = lua_checkudp(L, 1);

	// Grab the data
	const char *data = luaL_checkstring(L, 2);
	// Grab the size of the data, if given
	const int size = luaL_optint(L, 3, strlen(data));
	
	if( p->send_fd == p->recv_fd || p->send_fd<3 )
		return luaL_error(L,"Bad file descriptor (%d)!\n", p->send_fd);
	
	// TODO: add a maximum send limit
	if(size<0)
		return luaL_error(L,"Negative number of bytes to send!\n");

	// Send the data
	int ret = send(p->send_fd, data, size, 0);
	// Debugging message
	if(ret==-1){
		printf("\n\tUDP send fail:\n\t%s\n\n", strerror(errno));
		fflush(stdout);
	}
	// Push the return value
	lua_pushinteger(L, ret);
	return 1;
}

static int lua_udp_index(lua_State *L) {
  structUdp *p = lua_checkudp(L, 1);
  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

static int lua_udp_string(lua_State *L) {
  structUdp *p = lua_checkudp(L, 1);
  std::stringstream ss; 
  if (p->init_send) {
    ss << "Sender file descripter " << p->send_fd << ' ';
    ss << "IP: " << p->send_ip << " PORT: " << p->send_port << std::endl;
  } else if (p->init_recv) {
    ss << "Receiver file descripter " << p->recv_fd << 
      " at port " << p->recv_port << std::endl;
  } else {
    luaL_error(L, "not init");
  }
  lua_pushstring(L, ss.str().c_str());
  return 1;
}

static const struct luaL_Reg udp_function [] = {
	{"new_receiver", lua_udp_init_recv},
	{"new_sender", lua_udp_init_send},
	{NULL, NULL}
};

static const luaL_Reg udp_methods[] = {
  {"send", lua_udp_send},
  {"receive", lua_udp_receive},
	{"size", lua_udp_size},
  {"__tostring", lua_udp_string},
  {"__gc", lua_udp_close},
  {"__index", lua_udp_index},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_udp (lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
	luaL_newlib(L, udp_function);
  int i = 0;
  lua_pushvalue(L, -1);
  while (udp_methods[i].name) {
    lua_setfield(L, -2, udp_methods[i].name);
    lua_pushcfunction(L, udp_methods[i].func);
  }
#else
  luaL_register(L, NULL, udp_methods);
	luaL_register(L, "udp", udp_function);
#endif  
	return 1;
}
