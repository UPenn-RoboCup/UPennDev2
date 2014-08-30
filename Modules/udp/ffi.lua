-- UNIX FFI library
-- (c) Stephen McGill 2014
local ok, ffi = pcall(require, 'ffi')
if not ok then return require'udp' end
if ffi.os=='Linux' then return require'udp' end

local udp = {ffi=true}
local C = ffi.C
local bit = require'bit'
local bor = bit.bor
local tinsert = table.insert

-- Constants
local AF_INET = 2
local INADDR_ANY = 0
local SOCK_DGRAM = 2
local O_NONBLOCK = 4
local F_GETFL = 3
local F_SETFL = 4
-- Jumbo UDP packet
local MAX_LENGTH = 65536
-- OSX
local SOL_SOCKET = 0xffff
local SO_REUSEADDR = 0x0004
local SO_BROADCAST = 0x0020
if ffi.os=='Linux' then
  SOL_SOCKET = 1
  SO_REUSEADDR = 2
  SO_BROADCAST = 6
end

local function strerror()
  return ffi.string(C.strerror(ffi.errno()))
end

ffi.cdef[[
typedef struct hostent {
  char    *h_name;        /* official name of host */
  char    **h_aliases;    /* alias list */
  int     h_addrtype;     /* host address type */
  int     h_length;       /* length of address */
  char    **h_addr_list;  /* list of addresses from name server */
} hostent;
]]

if ffi.os=='Linux' and ffi.arch=='x86' then
	-- http://www.gta.ufrj.br/ensino/eel878/sockets/sockaddr_inman.html
  ffi.cdef[[
	typedef struct in_addr {
		unsigned long s_addr;  // load with inet_aton()
	};
  typedef struct sockaddr_in {
    short sin_family;
    unsigned short sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
  } sockaddr_in;
  ]]
elseif ffi.os=='Linux' then 
  ffi.cdef[[
	typedef struct in_addr {
		uint32_t s_addr;
	} in_addr;
  typedef struct sockaddr_in {
    uint8_t sin_len;
    uint16_t sin_family;
    uint16_t sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
  } sockaddr_in;
  ]]
elseif ffi.os=='OSX' then
  ffi.cdef[[
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
end

ffi.cdef[[
int socket(int domain, int type, int protocol);
uint32_t htonl(uint32_t hostlong);
uint16_t htons(uint16_t hostshort);
int bind(int socket, const struct sockaddr *address, uint32_t address_len);
int fcntl(int fildes, int cmd, ...);
struct hostent * gethostbyname(const char *name);
long recv(int socket, void *buffer, size_t length, int flags);
int setsockopt(int socket, int level, int option_name, const void *option_value, uint32_t option_len);
int connect(int socket, const struct sockaddr *address, uint32_t address_len);
long send(int socket, const void *buffer, size_t length, int flags);
int close(int fildes);
]]

-- TODO: Set the metatable correctly for Garbage collection
-- For now, this is OK - our open sockets are long lived
local function close(self) return C.close(self.fd)==0 end

-- Return a table of all the data received (Each element is a packet)
local recv_buffer = ffi.new('uint8_t[?]', MAX_LENGTH)
local function receive(self)
  local len = tonumber(C.recv(self.fd, recv_buffer, MAX_LENGTH, 0))
  if len > 0 then return ffi.string(recv_buffer, len) end
end

local function recv_all(self)
  local len = tonumber(C.recv(self.fd, recv_buffer, MAX_LENGTH, 0))
  -- Insert into the buffer
  local skt_buffer = {}
  while len>0 do
    tinsert(skt_buffer, ffi.string(recv_buffer, len))
    len = tonumber(C.recv(self.fd, recv_buffer, MAX_LENGTH, 0))
  end
  -- Remove the first packet
  return skt_buffer
end

function udp.new_receiver(port)
  local fd = assert(C.socket(AF_INET, SOCK_DGRAM, 0), "Could not open datagram recv socket!")
  -- Should be zero'd by default in LuaJIT
  local local_addr = ffi.new('sockaddr_in')
	--C.bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET
	local_addr.sin_addr.s_addr = C.htonl(INADDR_ANY)
	local_addr.sin_port = C.htons(port)
  local ret = C.bind(fd, ffi.cast('const struct sockaddr *', local_addr), ffi.sizeof(local_addr))
  assert(ret==0, "Bind: "..strerror())
	-- Nonblocking receive
	local flags = C.fcntl(fd, F_GETFL, 0)
	if flags == -1 then flags = 0 end
  ret = C.fcntl(fd, F_SETFL, ffi.new('int', bor(flags, O_NONBLOCK)))
  assert(ret==0, "Could not set non-blocking mode"..strerror())
  -- Object
  local obj = {
    port = port,
    fd = fd,
    receive = receive,
    recv_all = recv_all,
  }
  return obj
end

local function send(self, data, len)
  local sz = len or #data
  local ret = C.send(self.fd, data, sz, 0)
  if ret==sz then return ret end
  return ret, strerror()
end

function udp.new_sender(ip, port)
  local fd = C.socket(AF_INET, SOCK_DGRAM, 0)
  assert(fd > 0, "Could not open datagram send socket")
  local i, ret = ffi.new('int[1]', 1)
  ret = C.setsockopt(fd, SOL_SOCKET, SO_BROADCAST, ffi.cast('const char *', i), ffi.sizeof(i))
  assert(ret==0, "Bad set broadcast "..strerror())
  i[0] = 1
  ret = C.setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, ffi.cast('const char *', i), ffi.sizeof(i))
  assert(ret==0, "Re-use address"..strerror())
  local dest_addr = ffi.new('struct sockaddr_in')
  dest_addr.sin_family = AF_INET
  local hostptr = assert(C.gethostbyname(ip), "Could not get hostname")
  ffi.copy(hostptr.h_addr_list[0], ffi.cast('char*', dest_addr.sin_addr), hostptr.h_length)
  dest_addr.sin_port = C.htons(port)
  ret = C.connect(fd, ffi.cast('struct sockaddr *', dest_addr), ffi.sizeof(dest_addr))
  assert(ret==0,"Could not connect to destination address: "..strerror())
  return {
    ip = ip,
    port = port,
    fd = fd,
    send = send,
    close = close,
  }
end

return udp
