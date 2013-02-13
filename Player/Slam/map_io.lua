-- Add the required paths
cwd = '.';
package.cpath = cwd.."/../?.so;"..package.cpath;
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;
package.path = cwd.."/../Config/?.lua;"..package.path;
package.path = cwd.."/../Vision/?.lua;"..package.path;

-- Add torch and the Serialization utilities
require 'torch'
torch.Tensor = torch.DoubleTensor
dofile('../Util/File.lua')

-- Add the FFI sugar
local ffi = require( "ffi" )
local zmq = require( "zmq" )
ffi.cdef[[
int poll(struct pollfd *fds, unsigned long nfds, int timeout);
]]
function sleep(s)
    ffi.C.poll(nil, 0, s*1000)
end

-- Alias the send and receive calls
local send, recv = zmq.zmq_send, zmq.zmq_recv
local addr = "ipc:///tmp/mapserver"
local map_buf_sz = 720000;
local map_buf = ffi.new( "char[?]", map_buf_sz );
local ack_buf_sz = 1;
local ack_buf = ffi.new( "char[?]", ack_buf_sz );

-- If the primary role is receiving
function init_receive()
	-- Init the context
local ctx = zmq.zmq_init(1);
assert (ctx);
-- Init the Socket type
local skt = zmq.zmq_socket (ctx, zmq.ZMQ_REP);
assert (skt);
-- Bind to the socket at an address
local rc = zmq.zmq_bind (skt, addr )
assert (rc == 0);
-- Make a connection wrapper to the socket
skt_conn = new_connection( skt )
end

-- If the primary role is the send
function init_send()
-- Init the context
local ctx = zmq.zmq_init (1);
assert (ctx);
-- Init the Socket type
local skt = zmq.zmq_socket (ctx, zmq.ZMQ_REQ);
assert (skt);
-- Bind to the socket at an address
local rc = zmq.zmq_connect (skt, addr )
assert (rc == 0);
-- Make a connection wrapper to the socket
skt_conn = new_connection( skt )
end

local function zmq_error()
   local errno = zmq.zmq_errno()
   local strerror = ffi.string( zmq.zmq_strerror( errno ) )
   error( "ZMQ ERROR: " .. errno .. ": " .. strerror )
end

-- Connection Wrapper
local connection = { }
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

function send_map( my_map )
local serialized2send = torch.serialize(  my_map  )
ffi.copy(map_buf, serialized2send)
local r1 = skt_conn:send( map_buf, map_buf_sz )
assert( r1 == map_buf_sz )
-- Receive an ACK
-- TODO: Remove the need...
local r2 = skt_conn:recv( ack_buf, ack_buf_sz )
assert( r2 == ack_buf_sz )
end

function receive_map()
	local r1 = skt_conn:recv( map_buf, map_buf_sz )
	assert( r1 == map_buf_sz )
	-- Send an ACK
	-- TODO: Remove this need
    local r2 = skt_conn:send(ack_buf,ack_buf_sz)
	assert( r2 == ack_buf_sz )
	local ser_str = ffi.string(map_buf, map_buf_sz);
	local recv_map = torch.deserialize( ser_str )	
	return recv_map
end

-- Close everything
function close_map()
	local rc = zmq.zmq_close (skt);
	assert (rc == 0);
	rc = zmq.zmq_term (ctx);
	assert (rc == 0);
end