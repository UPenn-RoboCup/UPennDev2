-- Include the proper directories
cwd = '.';
package.path = cwd.."/../../../Tools/?.lua;"..package.path;

-- Require the ffi modules
local ffi = require 'ffi'
local zmq = require 'ffi/zmq'

-- Setup the millisecond sleep function
ffi.cdef[[
int poll(struct pollfd *fds, unsigned long nfds, int timeout);
]]
local function msleep(ms)
  ffi.C.poll(nil, 0, ms)
end

-- Initialize the context
local n_zmq_threads = 1;
local context_handle = zmq.zmq_init( n_zmq_threads );
assert (context_handle);

-- Set the socket type
local test_socket = zmq.zmq_socket (context_handle, zmq.ZMQ_SUB);
assert (test_socket);

-- Connect to a message pipeline
local rc = zmq.zmq_bind( test_socket, "ipc:///tmp/test_pubsub" )
assert (rc == 0);
zmq.zmq_setsockopt(test_socket, zmq.ZMQ_SUBSCRIBE, '', 0)

print('Sucessfully subscribed to the message pipeline!')

-- Set up the message buffer
local msg_buf_sz = 5;
local msg_buf = ffi.new('uint8_t[?]',msg_buf_sz)

-- Listen for messages
while true do
  local n_bytes_recv = zmq.zmq_recv( test_socket, msg_buf, msg_buf_sz, 0);
  print('Received '..n_bytes_recv.." bytes")
end
