/*
Lua file to send and receive UDP messages.
Daniel D. Lee copyright 2009 <ddlee@seas.upenn.edu>
Stephen G. McGill copyright 2013 <smcgill3@seas.upenn.edu>
Yida Zhang copyright 2013 <yida@seas.upenn.edu>
*/

#include <string>
#include <cstring>
#include <deque>
#include <map>
#include <vector>
#include <algorithm>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <assert.h>

#include <lua.hpp>

//#define MAX_LENGTH 65536
//#define MAX_BODY_LENGTH 60000
//#define UUID_LENGTH 8
// DARPA Robotics Challenge
// 1500 MTU - 8 UDP header - 20 IP header = 1472
//#define MAX_LENGTH 1500
#define MAX_LENGTH 1472
#define UUID_LENGTH 5
// Extra is UUID+5
#define EXTRA_LENGTH 10
#define MAX_BODY_LENGTH 1462

#define MT_NAME "udp_mt"

using namespace std;

typedef struct {
  uint8_t uuid[UUID_LENGTH];
  uint8_t order;
  uint8_t number;
  uint8_t size0;
	uint8_t size1;
  uint8_t checksum;
  int8_t data[MAX_BODY_LENGTH];
} packet;

typedef struct {
  bool init_recv;
  int recv_fd;
  int recv_port;
  bool init_send;
  int send_fd;
  int send_port;
  const char * send_ip;
  std::deque<std::string> *recv_queue;
  std::deque<std::string> *raw_recv_queue;
} structUdp;

const int maxQueueSize = 12;

// receiver buffer
static std::map<std::string, std::map<char, std::string> > uuid_buf;

inline uint8_t checksum_gen(const char *header) {
  char sum = 0;
  for (int i = 0; i < (UUID_LENGTH+4); i++) {
    sum ^= header[i];
  }
  return sum;
}

static structUdp * lua_checkudp(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid Udp udata");
  return (structUdp *)ud;
}

// http://stackoverflow.com/questions/440133/how-do-i-create-a-random-alpha-numeric-string-in-c
static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
char rand_uuid_buf[UUID_LENGTH];
void gen_random() {
    for (int i = 0; i < UUID_LENGTH; ++i) {
        rand_uuid_buf[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }
}

static int lua_udp_close(lua_State *L) {
  structUdp *p = lua_checkudp(L, 1);

  if (p->recv_fd > 0 && p->init_recv) {
    close(p->recv_fd);
    p->init_recv = false;
    p->recv_queue->clear();
    delete p->recv_queue;
    p->raw_recv_queue->clear();
    delete p->raw_recv_queue;
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

	// Grab port to listen on (all interfaces)
	ud->recv_port = luaL_checkint(L,1);

	ud->recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (ud->recv_fd < 0)
		return luaL_error(L,"Could not open datagram recv socket!\n");
  ud->recv_queue = new std::deque<std::string>;
  ud->raw_recv_queue = new std::deque<std::string>;

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

	if ( connect(ud->send_fd, 
				(struct sockaddr *) &dest_addr, 
				sizeof(dest_addr))
			< 0 )
		return luaL_error(L,"Could not connect to destination address\n");

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	return 1;
}

static int packet_update(structUdp *p) {
  std::string pkt_msg = p->raw_recv_queue->front();
  std::string uuid_str = pkt_msg.substr(0, UUID_LENGTH);
  uint8_t order = pkt_msg[UUID_LENGTH];
  uint8_t number = pkt_msg[UUID_LENGTH + 1];
	uint8_t size0 = pkt_msg[UUID_LENGTH + 2];
	uint8_t size1 = pkt_msg[UUID_LENGTH + 3];
  uint16_t size = (size1 << 8 & 0xFF00) | size0;
  uint8_t checksum = pkt_msg[UUID_LENGTH + 4];
	
/*
	printf("UUID: %s\n", uuid_str.c_str());
	printf("order %u\n", order);
	printf("number %u\n", number);
	printf("size: %u [%u|%u]\n", size, size0, size1);
  printf("checksum %u %u\n",
				 checksum, checksum_gen(pkt_msg.c_str())
				);
*/
  if (checksum != checksum_gen(pkt_msg.c_str())) {
		//printf("BAD CHECKSUM\n");
    p->recv_queue->push_back(pkt_msg);
    return 0;
  }
	
	
  int8_t * data = (int8_t *)pkt_msg.c_str() + UUID_LENGTH + 5;
	
  if (number == 1) {
		//printf("ONLY ONE\n");
    p->recv_queue->push_back(std::string((char *)data, size));
    return 0;
  }

  // update packets
  std::map<std::string, std::map<char, std::string> >::iterator uuid_it = uuid_buf.find(uuid_str);
  std::map<char, std::string> * pkt_buf = NULL;
  if (uuid_it == uuid_buf.end()) {
    std::map<char, std::string> new_pkt_buf;
    uuid_buf[uuid_str] = new_pkt_buf;
    pkt_buf = &uuid_buf[uuid_str];
  } else {
    pkt_buf = &uuid_it->second;
  }

  std::map<char, std::string>::iterator pkt_it = pkt_buf->find(order);
  std::string msg_buf;
  if (pkt_it == pkt_buf->end()) {
    msg_buf.assign((char *)data, size);
    pkt_buf->insert(std::pair<char, std::string>(order, msg_buf));
  }

  std::string out_msg;
  std::map<char, std::string> pkt_inst = *pkt_buf;
  if (pkt_buf->size() == number) {
    for (int i = 0; i < number; i++) {
      out_msg.append(pkt_inst[i]);
    }
    p->recv_queue->push_back(out_msg);
    pkt_buf->clear();
  }

  return 0;
}

static int comm_update(structUdp *p) {

	static sockaddr_in source_addr;
	static char data[MAX_LENGTH];

	// Check whether initiated
	assert( p->init_recv );
	// Process incoming messages:
	socklen_t source_addr_len = sizeof(source_addr);
	int len = recvfrom(
		p->recv_fd,
		data,
		MAX_LENGTH,
		0,
		(struct sockaddr *) &source_addr,
		&source_addr_len
	);

	while (len > 0) {
		std::string msg((const char *) data, len);
    p->raw_recv_queue->push_back(msg);
		len = recvfrom(
			p->recv_fd,
			data,
			MAX_LENGTH,
			0,
			(struct sockaddr *) &source_addr,
			&source_addr_len
		);
	}

  while (p->raw_recv_queue->size() > 0) {
    packet_update(p);
    p->raw_recv_queue->pop_front();
  }

	// Remove older messages
  while (p->recv_queue->size() > maxQueueSize)
    p->recv_queue->pop_front();

	return p->recv_queue->size();
}

static int lua_udp_size(lua_State *L) {
  structUdp *p = lua_checkudp(L, 1);
	if( !p->init_recv ){
		/*
		lua_pushinteger( L, 0 );
		return 1;
		*/
		return luaL_error(L,"Did not initialize the UDP as a receiver!\n");
	}
	/*
	fprintf(stdout,"udp size: %d (%d)\n",p->init_recv,p->recv_fd);
	fflush(stdout);
	*/
	int updateRet = comm_update(p);
	lua_pushinteger( L, updateRet );
	return 1;
}

static int lua_udp_receive(lua_State *L) {

  structUdp *p = lua_checkudp(L, 1);

	// Perform an update to receive data
	//fprintf(stdout,"udp size: %d (%d)\n",p->init_recv,p->recv_fd);
	//fflush(stdout);
	//int updateRet = comm_update(p);
	comm_update(p);

	// If empty, then return nil
  if (p->recv_queue->empty()) {
    lua_pushnil(L);
    return 1;
  }

	// Push a light string with the incoming data from the queue
	lua_pushlstring(L, p->recv_queue->front().c_str(), 
                      p->recv_queue->front().size());
	// Remove this object from the receiving buffer queue
	p->recv_queue->pop_front();

	return 1;
}

static int lua_udp_send(lua_State *L) {

  structUdp *p = lua_checkudp(L, 1);
	/* Src pointer and length */
	const char *data = NULL;
	size_t size;
	if( lua_isstring(L,2) ){
		data = lua_tolstring( L, 2, &size );
		/* Grab the size of the data, if given */
		size = luaL_optinteger( L, 3, size );
	} else if(lua_islightuserdata(L,2)) {
		if( (data=(const char *)lua_touserdata(L, 2))==NULL ){
			return luaL_error(L, "Input src is NULL");
		}
		/* Grab the size of the data; required for userdata*/
		size = luaL_checkinteger( L, 3 );
	}
	
	if( p->send_fd == p->recv_fd || p->send_fd<3 )
		return luaL_error(L,"Bad send file descriptor (%d)!\n", p->send_fd);
	
	// Send the data
	int ret = send(p->send_fd, data, size, 0);
	// Debugging message
	// Push the return value
	lua_pushinteger(L, ret);
	if(ret==-1){
		lua_pushstring( L, strerror(errno) );
		return 2;
	}
	return 1;
}

// for sending data without size limit
static int lua_udp_send_all(lua_State *L) {
  structUdp *ud = lua_checkudp(L, 1);
  if( ud->send_fd == ud->recv_fd || ud->send_fd<3 )
	  return luaL_error(L,"Bad send all file descriptor (%d)!\n", ud->send_fd);

  size_t size = 0;
  const char *data = NULL;
  const char * uuid = NULL;
  if (lua_isstring(L, 2)) {
    data = lua_tolstring(L, 2, &size);
    // Grab the size of the data if given
    //size = luaL_optinteger(L, 3, size);
    uuid = luaL_optstring(L, 3, NULL);
  } else if (lua_islightuserdata(L, 2)) {
    if ((data = (const char *)lua_touserdata(L, 2)) == NULL) {
      return luaL_error(L, "Input is NULL");
    }
    size = luaL_checkinteger(L, 3);
    uuid = luaL_optstring(L, 4, NULL);
  }
  // if no uuid given, send with packing
	/*
  if (uuid == NULL) {
	  // Send the data
	  int ret = send(ud->send_fd, data, size, 0);
	  // Debugging message
	  // Push the return value
	  lua_pushinteger(L, ret);
	  if(ret==-1){
	  	lua_pushstring( L, strerror(errno) );
	  	return 2;
	  }
	  return 1;
  }
	*/
	if (uuid == NULL) {
		gen_random();
		uuid = rand_uuid_buf;
	}

  size_t uuid_len = strlen(uuid);
  if (uuid_len > UUID_LENGTH){
		uuid_len = UUID_LENGTH;
	}
  
  uint8_t num_packets = size / MAX_BODY_LENGTH;
  uint16_t size_last_packets = size % MAX_BODY_LENGTH;
	//printf("num_packets: %d\n", num_packets);
  if (size_last_packets > 0) num_packets ++;
	//printf("num_packets*: %d\n", num_packets);
	
  std::vector<packet> packets;
  for (int i = 0; i < num_packets; i++) {
    int packet_len = (i < (num_packets - 1)) ? MAX_BODY_LENGTH : size_last_packets;
		//printf("packet_len %d %d\n", packet_len, UUID_LENGTH);
    packet new_packet;
    memset(new_packet.uuid, 0x0, UUID_LENGTH * sizeof(uint8_t));
    memcpy(new_packet.uuid, uuid, uuid_len * sizeof(uint8_t));
    new_packet.order = i;
    new_packet.number = num_packets;
    new_packet.size0 = packet_len & 0xFF;
		new_packet.size1 = (packet_len>>8) & 0xFF;
    new_packet.checksum = checksum_gen((const char *)&new_packet);
    memset(new_packet.data, 0x0, MAX_BODY_LENGTH * sizeof(uint8_t));
    memcpy(new_packet.data, (uint8_t *)data, packet_len * sizeof(uint8_t));
    data += packet_len;
    packets.push_back(new_packet);
  }
  
//  useconds_t usec = 10000;
  size_t raw_len = 0, ret = 0;
	/*
  size_t extra_bytes = 3 * sizeof(uint8_t) + sizeof(uint16_t) +
                          (UUID_LENGTH + 1) * sizeof(uint8_t);
													*/

	//fprintf(stdout, "npkt: %lu\n", packets.size());
	lua_createtable(L, packets.size(), 0);
  for (int i = 0; i < packets.size(); i++) {
    raw_len = EXTRA_LENGTH + ((packets[i].size1<<8 & 0xFF00) | packets[i].size0) * sizeof(uint8_t);
	  ret = send(ud->send_fd, &packets[i], raw_len, 0);
		lua_pushnumber(L, ret);
		lua_rawseti(L, -2, i+1);
//		printf("Sending %lu %lu\n", raw_len, ret);
//		printf("Sending... sum: %u | size: %d %d\n", packets[i].checksum, packets[i].size0, packets[i].size1);
//    usleep((useconds_t) usec);
  }
	lua_pushlstring(L, uuid, UUID_LENGTH);
  return 2;
}

static int lua_udp_index(lua_State *L) {
  //structUdp *p = lua_checkudp(L, 1);
  lua_checkudp(L, 1);
  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

static int lua_udp_string(lua_State *L) {
	structUdp *p = lua_checkudp(L, 1);
	char msg_buf[255];
  if (p->init_send)
		sprintf(msg_buf,"UDP Sender <%s:%d>",p->send_ip,p->send_port);
  else if (p->init_recv)
		sprintf(msg_buf,"UDP Receiver <%d>",p->recv_port);
  else
		sprintf(msg_buf,"UDP Uninitialized");
  lua_pushstring(L, msg_buf);
  return 1;
}

static int lua_udp_fd(lua_State *L) {

  structUdp *p = lua_checkudp(L, 1);
  if (p->init_recv) {
    lua_pushinteger(L, p->recv_fd);
  } else if (p->init_send) {
    lua_pushinteger(L, p->send_fd);
  } else {
    luaL_error(L, "udp not init");
  }
  return 1;
}

static const struct luaL_Reg udp_function [] = {
	{"new_receiver", lua_udp_init_recv},
	{"new_sender", lua_udp_init_send},
	{NULL, NULL}
};

static const luaL_Reg udp_methods[] = {
  {"send_all", lua_udp_send_all},
  {"send", lua_udp_send},
	{"size", lua_udp_size},
  {"receive", lua_udp_receive},
  {"descriptor", lua_udp_fd},
  {"close", lua_udp_close},
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
	luaL_setfuncs( L, udp_methods, 0 );
	luaL_newlib(L, udp_function);
#else
  luaL_register(L, NULL, udp_methods);
	luaL_register(L, "udp", udp_function);
#endif  
	return 1;
}
