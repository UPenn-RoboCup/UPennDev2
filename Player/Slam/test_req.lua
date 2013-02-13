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
-- Alias
local send, recv = zmq.zmq_send, zmq.zmq_recv

-- Data to send
local send_buf_sz = 720000;
local send_buf = ffi.new( "char[?]", send_buf_sz )
local tensor2send = torch.rand(320,240);
local serialized2send = torch.serialize( tensor2send )
--local buf1 = ffi.new( "char[?]", #serialized2send, serialized2send )
--local size1 = ffi.sizeof(buf1)
-- Data to receive
local recv_buf_sz = 1;
local recv_buf = ffi.new( "char[?]", recv_buf_sz )

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

require 'unix'
t0 = unix.time()
for i=1,5 do
	-- Manipulate a new tensor
	tensor2send = tensor2send:rand(320,240);
	serialized2send = torch.serialize( tensor2send )
	ffi.copy(send_buf, serialized2send)
	local r1 = sc:send( send_buf, send_buf_sz )
	assert( r1 == send_buf_sz )
	-- Receive an ACK
	-- TODO: Remove the need...
  local r2 = sc:recv( recv_buf, recv_buf_sz )
  assert( r2 == recv_buf_sz )
end
t1 = unix.time()
print('FPS:',5/(t1-t0))

-- Close everything
sc = sc.handle;
rc = zmq.zmq_close (sc);
assert (rc == 0);

rc = zmq.zmq_term (ctx);
assert (rc == 0);

print("DONE")
