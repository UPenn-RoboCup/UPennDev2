/*
Lua file to send and receive UDP messages.
Daniel D. Lee copyright 2009 <ddlee@seas.upenn.edu>
Stephen G. McGill copyright 2013 <smcgill3@seas.upenn.edu>
*/
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
#include <vector>
#include <algorithm>
#include <stdint.h>

// C++ include
#include "lua.hpp"

#define MDELAY 2
#define TTL 16
//Size for sending 640*480 yuyv data without resampling
#define MAX_LENGTH 160000

// TODO: Make more recv file descriptors, 
// which are saved in the Lua metatable
static bool init_recv = false;
static int recv_fd;
const int maxQueueSize = 12;
static std::deque<std::string> recvQueue;
static int comm_update();

// TODO: Close send descriptors as well
static int lua_comm_close(lua_State *L) {
	const int fd = luaL_checkint(L,1);
	if (fd == recv_fd){
		if( recv_fd > 0 && init_recv ){
			close(recv_fd);
			init_recv = false;
			recvQueue.clear();
			lua_pushboolean(L,1);
			return 1;
		}
	}
	else if(fd>2 && fd!=recv_fd){
		close(fd);
		lua_pushboolean(L,1);
		return 1;
	}
	
	lua_pushboolean(L,0);
	return 1;
}

static int lua_comm_init_recv(lua_State *L) {

	if( init_recv )
		return luaL_error(L,"Already initialized receiver!\n");

	// Grab port to listen on (all interfaces)
	const int RECV_PORT = luaL_checkint(L,1);

	recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (recv_fd < 0)
		return luaL_error(L,"Could not open datagram recv socket!\n");

	struct sockaddr_in local_addr;
	bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	local_addr.sin_port = htons(RECV_PORT);
	if (bind(recv_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) < 0)
		return luaL_error(L,"Could not bind to port\n");

	// Nonblocking receive
	int flags  = fcntl(recv_fd, F_GETFL, 0);
	if (flags == -1) 
		flags = 0;
	if ( fcntl(recv_fd, F_SETFL, flags|O_NONBLOCK)<0 )
		return luaL_error(L,"Could not set nonblocking mode\n");

	// TODO: Store in an array, so we can have multiple ports
	init_recv = true;
	lua_pushinteger(L, recv_fd);
	return 1;
}

static int lua_comm_init_send(lua_State *L) {
	
	// TODO: check if already assigned an ip/port combo?
	const char* SEND_IP = luaL_checkstring(L, 1);//std::string
	const int SEND_PORT = luaL_checkint(L,2);

	// Where to send the data
	struct hostent *hostptr = gethostbyname( SEND_IP );
	if (hostptr == NULL)
		return luaL_error(L,"Could not get hostname\n");

	const int send_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (send_fd < 0)
		return luaL_error(L,"Could not open datagram send socket\n");

	int i = 1;
	if (setsockopt(send_fd, SOL_SOCKET, SO_BROADCAST, 
	(const char *) &i, sizeof(i)) < 0)
		return luaL_error(L,"Could not set broadcast option\n");
	i = 1;
	if (setsockopt(send_fd, SOL_SOCKET, SO_REUSEADDR,
	(const char *) &i, sizeof(i)) < 0)
		return luaL_error(L,"Could not set reuse option");

	struct sockaddr_in dest_addr;
	bzero((char *) &dest_addr, sizeof(dest_addr));
	dest_addr.sin_family = AF_INET;
	bcopy(hostptr->h_addr, (char *) &dest_addr.sin_addr, hostptr->h_length);
	dest_addr.sin_port = htons(SEND_PORT);

	if (connect(send_fd, (struct sockaddr *) &dest_addr, sizeof(dest_addr)) < 0)
		return luaL_error(L,"Could not connect to destination address\n");

	// Return the file descriptor
	lua_pushinteger(L, send_fd);
	return 1;
}

static int comm_update() {
	static sockaddr_in source_addr;
	static char data[MAX_LENGTH];

	// Check whether initiated
	// TODO: associate checks with the metatable
	assert( init_recv );

	// Process incoming messages:
	socklen_t source_addr_len = sizeof(source_addr);
	int len = 
		recvfrom(recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
	while (len > 0) {
		std::string msg((const char *) data, len);
		recvQueue.push_back(msg);
		len = recvfrom(recv_fd, data, MAX_LENGTH, 0, (struct sockaddr *) &source_addr, &source_addr_len);
	}

	// Remove older messages
	while (recvQueue.size() > maxQueueSize)
		recvQueue.pop_front();

	return recvQueue.size();
}

static int lua_comm_size(lua_State *L) {
	int updateRet = comm_update();
	lua_pushinteger( L, updateRet );
	return 1;
}

static int lua_comm_receive(lua_State *L) {
	// Perform an update to receive data
	int updateRet = comm_update();

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
	// Grab the file descriptor
	const int send_fd = luaL_checkint(L, 1);
	// Grab the data
	const char *data = luaL_checkstring(L, 2);
	// Grab the size of the data, if given
	const int size = luaL_optint(L, 3, strlen(data));
	
	if( send_fd==recv_fd || send_fd<3 )
		return luaL_error(L,"Bad file descriptor (%d)!\n", send_fd);
	
	// TODO: add a maximum send limit
	if(size<0)
		return luaL_error(L,"Negative number of bytes to send!\n");

	// Send the data
	int ret = send(send_fd, data, size, 0);
	// Push the return value
	lua_pushinteger(L, ret);
	return 1;
}

static const struct luaL_Reg udp [] = {
	{"new_receiver", lua_comm_init_recv},
	{"new_sender", lua_comm_init_send},
	{"size", lua_comm_size},
	{"receive", lua_comm_receive},
	{"send", lua_comm_send},
	{"close", lua_comm_close},
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