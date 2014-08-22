-- UNIX FFI library
-- (c) Stephen McGill 2014

local ok, ffi = pcall(require, 'ffi')
if not ok then return require'unix' end

local unix = {}
local C = ffi.C
local bit    = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local band   = bit.band
local bor    = bit.bor
local floor  = math.floor

ffi.cdef[[
static const int O_RDONLY = 0x0000; /* open for reading only */
static const int O_WRONLY = 0x0001; /* open for writing only */
static const int O_RDWR = 0x0002; /* open for reading and writing */
static const int O_ACCMODE = 0x0003; /* mask for above modes */
static const int O_NONBLOCK = 0x0004; /* no delay */
static const int O_APPEND = 0x0008; /* set append mode */
static const int O_CREAT = 0x0200; /* create if nonexistant */
static const int O_NOCTTY = 0x20000; /* don't assign controlling terminal */
static const int F_GETFL = 3; /* get file status flags */
static const int F_SETFL = 4; /* set file status flags */
static const int F_DUPFD = 0; /* duplicate file descriptor */
]]
unix.O_RDONLY = C.O_RDONLY
unix.O_WRONLY = C.O_WRONLY
unix.O_RDWR = C.O_RDWR
unix.O_ACCMODE = C.O_ACCMODE
unix.O_NONBLOCK = C.O_NONBLOCK
unix.O_APPEND = C.O_APPEND
unix.O_CREAT = C.O_CREAT
unix.O_NOCTTY = C.O_NOCTTY
unix.F_GETFL = C.F_GETFL
unix.F_SETFL = C.F_SETFL
unix.F_DUPFD = C.F_DUPFD

ffi.cdef[[
typedef uint16_t mode_t;
static const int O_NDELAY = O_NONBLOCK; /* compat */
long read(int fildes, void *buf, size_t nbyte);
long write(int fildes, const void *buf, size_t nbyte);
]]

ffi.cdef[[
typedef struct timeval {
	long tv_sec;
	int32_t tv_usec;
} timeval;

typedef struct utsname {
  char sysname[256];  /* [XSI] Name of OS */
  char nodename[256]; /* [XSI] Name of this network node */
  char release[256];  /* [XSI] Release level */
  char version[256];  /* [XSI] Version level */
  char machine[256];  /* [XSI] Hardware type */
} utsname;

void free(void *ptr);
]]

ffi.cdef[[
int usleep(uint32_t useconds);
int gettimeofday(struct timeval *restrict tp, void *restrict tzp);
unsigned int sleep(unsigned int seconds);
int uname(struct utsname *name);
int gethostname(char *name, size_t namelen);
int open(const char *path, int oflag, ...);
int close(int fildes);
char * strerror(int errnum);
char * getcwd(char *buf, size_t size);
int chdir(const char *path);
int mkfifo(const char *path, mode_t mode);
]]

ffi.cdef[[
typedef struct fd_set {
	int32_t fds_bits[32];
} fd_set;
int select(int nfds, fd_set *restrict readfds, fd_set *restrict writefds, fd_set *restrict errorfds, struct timeval *restrict timeout);
]]

-- Sleep in microseconds
unix.usleep = C.usleep
unix.sleep = C.sleep

-- Grab the time in seconds
local t = ffi.new"timeval"
function unix.time()
  C.gettimeofday(t, nil)
	return tonumber(t.tv_sec) + 1E-6 * t.tv_usec
end

function unix.uname()
	local un = ffi.new"utsname"
	C.uname(un)
	return ffi.string(un.sysname)
end

function unix.gethostname()
	local mlen = 128
	local hn = ffi.new("char[?]", mlen)
	C.gethostname(hn, mlen)
	return ffi.string(hn)
end

function unix.open(path, flags)
	assert(path)
	flags = flags or C.O_RDONLY
  return C.open(path, flags)
end
unix.close = C.close

function unix.read(fd, nbyte)
	local read_buf_sz = nbyte or 1024
	local read_buf = ffi.new("char[?]", read_buf_sz)
	local ret = C.read(fd, read_buf, read_buf_sz)
	if ret > 0 then
		return ffi.string(read_buf, ret)
	elseif ret < 0 then
		return ret
	end
end

-- TODO: have multiple items
-- NOTE: We don't actually do multiple items just yet...
function unix.write(fd, item)
  local buf = tostring(item)
	local len = #buf
	return C.write(fd, buf, len)
end
unix.chdir = C.chdir
function unix.getcwd()
	local raw_path = C.getcwd(NULL, 0)
	local path = ffi.string(raw_path)
	C.free(raw_path)
	return path
end

function unix.readdir()
	local f = io.popen'ls -f1'
	local entries, entry = {}
	repeat
		entry = f:read()
		table.insert(entries, entry)
	until not entry
	return entries
end

local to = ffi.new'timeval'
function unix.select(fds, timeout)
	-- Zero the struct
  local fdset = ffi.new'fd_set'
	local nfds = #fds
	local maxfd = 0
	-- Setup the file descriptor set
	for _, fd in ipairs(fds) do
    maxfd = fd > maxfd and fd or maxfd
		--FD_SET(fd, fds)
		fdset.fds_bits[fd / 32] = bor(fdset.fds_bits[fd / 32], lshift(1, fd % 32))
  end
	-- Add a timeout and run the syscall
	local status
	if timeout then
		local integral = floor(timeout)
    to.tv_sec = integral
		to.tv_usec = (timeout - integral) * 1E6
    status = C.select(maxfd + 1, fdset, nil, nil, to)
  else
    status = C.select(maxfd + 1, fdset, nil, nil, nil)
  end
	-- Parse the status
	local set = {}
	for _, fd in ipairs(fds) do
    --table.insert(set, C.FD_ISSET(fd, fds)~=0)
		--table.insert(set, band(fdset.fds_bits[fd / 32], lshift(1, fd % 32)) ~= 0)
		set[fd] = band(fdset.fds_bits[fd / 32], lshift(1, fd % 32)) ~= 0
  end
	return status, set
end

unix.system = os.execute

unix.mkfifo = C.mkfifo

--[[
local mt = {}
-- Provide raw access to all of our functions and variables
-- TODO: Should really just save the constants ourselves...
mt.__index = function(t, k)
	return C[k]
end
return setmetatable(unix, mt)
--]]
return unix
