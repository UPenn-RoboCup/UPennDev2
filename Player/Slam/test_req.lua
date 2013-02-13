-- Add the required paths
cwd = '.';
uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/../?.so;"..package.cpath;
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd.."/../Config/?.lua;"..package.path;
package.path = cwd.."/../Vision/?.lua;"..package.path;

require 'torch'
torch.Tensor = torch.DoubleTensor
dofile('../Util/File.lua')

local ffi = require( "ffi" )
local zmq = require( "zmq" )
ffi.cdef[[
int poll(struct pollfd *fds, unsigned long nfds, int timeout);
]]
function sleep(s)
    ffi.C.poll(nil, 0, s*1000)
end

local tensor2send = torch.rand(5);
print('s:',tensor2send[1])
local buf2send = torch.serialize( tensor2send )

--[[
print(buf2send)
for i=1,5 do
  print( tensor2send[i] )
end
--]]
local send, recv = zmq.zmq_send, zmq.zmq_recv

content = {}

for i=0,65535 do
  content[#content + 1] = string.char(math.floor(math.random(255)))
end

content = table.concat(content)
print(#content)

local buf  = ffi.new( "char[?]", #content, content )
local buf1 = ffi.new( "char[?]", #content )
ffi.copy(buf1, buf2send)

local buf2 = ffi.new( "char[?]", #content )
local size, size1, size2 = ffi.sizeof(buf), ffi.sizeof(buf1), ffi.sizeof(buf2)
print('Size of serialized:',#buf2send,size1);

local connection = { }

local function zmq_error()
   local errno = zmq.zmq_errno()
   local strerror = ffi.string( zmq.zmq_strerror( errno ) )
   error( "ZMQ ERROR: " .. errno .. ": " .. strerror )
end

function connection.send( self, buf, len )
   local r = send( self.handle, buf, len, 0 )
   if r ~= len then zmq_error() end
   return r
end

function connection.recv( self, buf, len )
   local r = recv( self.handle, buf, len, 0 )
   if r ~= len then zmq_error() end
   return r
end

function new_connection( handle )
   local c = {}
   c.send = connection.send
   c.recv = connection.recv
   c.handle = handle
   return c
end

local function bounce( sc )
   local r1, r4
	 r1 = sc:send( buf1, size1 )
	 print('Sending',size1)
   r4 = sc:recv( buf2, #content )
	 print('Received',buf2[1])
   assert( r1 == size )
   assert( r4 == size2 )
end

-- Init the context
local ctx = zmq.zmq_init (1);
assert (ctx);

-- Init the Socket type
local sc = zmq.zmq_socket (ctx, zmq.ZMQ_REQ);
assert (sc);

-- Bind to the socket at an address
local rc = zmq.zmq_connect (sc, "ipc:///tmp/rpcserver" )
assert (rc == 0);

print('Bound to the Socket!')

-- Establish connection helpers...?
local sc = new_connection( sc )

for i=1,5 do
	print('Sending msg...')
	bounce(sc)
  sleep(1)
end

-- Debug
print(1024*1024/64)

-- Close everything
rc = zmq.zmq_close (sb);
assert (rc == 0);

rc = zmq.zmq_term (ctx);
assert (rc == 0);

print("DONE")
