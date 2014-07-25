-- UNIX FFI library
-- (c) Stephen McGill 2014
local udp = {}
--
local ffi = require'ffi'
local C = ffi.C

local bit = require'bit'
local bor = bit.bor

-- Constants
local AF_INET = 2
local INADDR_ANY = 0
local SOCK_DGRAM = 2
local O_NONBLOCK = 4
local F_GETFL = 3
local F_SETFL = 4

ffi.cdef[[
typedef struct hostent {
  char    *h_name;        /* official name of host */
  char    **h_aliases;    /* alias list */
  int     h_addrtype;     /* host address type */
  int     h_length;       /* length of address */
  char    **h_addr_list;  /* list of addresses from name server */
} hostent;

typedef struct in_addr {
  uint32_t s_addr;
} in_addr;

typedef struct sockaddr_in {
  uint8_t sin_len;
  uint8_t sin_family;
  uint16_t sin_port;
  struct in_addr sin_addr;
  char sin_zero[8];
} sockaddr_in;

]]

ffi.cdef[[
int socket(int domain, int type, int protocol);
uint32_t htonl(uint32_t hostlong);
uint16_t htons(uint16_t hostshort);
int bind(int socket, const struct sockaddr *address, uint32_t address_len);
int fcntl(int fildes, int cmd, ...);
//
struct hostent * gethostbyname(const char *name);
//
long recvfrom(int socket, void *restrict buffer, size_t length, int flags,
struct sockaddr *restrict address, uint32_t *restrict address_len);
]]

local function receive()
end

function udp.new_receiver(port)
  local fd = assert(C.socket(AF_INET, SOCK_DGRAM, 0), "Could not open datagram recv socket!")
  -- Should be zero'd by default in LuaJIT
  local local_addr = ffi.new('sockaddr_in')
	--C.bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET
	local_addr.sin_addr.s_addr = C.htonl(INADDR_ANY)
	local_addr.sin_port = C.htons(port)
  --local ret = C.bind(ud->recv_fd, (struct sockaddr *) &local_addr, sizeof(local_addr))
  local ret = C.bind(fd, ffi.cast('const struct sockaddr *', local_addr), ffi.sizeof(local_addr))
  assert(ret==0, "Could not bind to port!")
  
	-- Nonblocking receive
	local flags = C.fcntl(fd, F_GETFL, 0)
	if flags == -1 then flags = 0 end
  
  ret = C.fcntl(fd, F_SETFL, bor(flags, O_NONBLOCK))
  assert(ret>=0, "Could not set non-blocking mode")
  
  local obj = {
    kind = 'receiver',
    port = port,
    fd = fd,
    receive = receive,
    --std::deque<std::string> *recv_queue;
    --std::deque<std::string> *raw_recv_queue;
  }
  return obj
end

--]]
return udp
