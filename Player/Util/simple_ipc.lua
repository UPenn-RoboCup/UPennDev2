local ffi = require 'ffi'
local zmq = require 'ffi/zmq'

-- Based on ZMQ
local simple_ipc = {}
-- Simple number of threads
simple_ipc.n_zmq_threads = 1

function simple_ipc.setup_publisher( channel )
  local channel_obj = {}
	channel_obj.name = "ipc:///tmp/"..channel;
  channel_obj.context_handle = zmq.zmq_init( simple_ipc.n_zmq_threads )
  assert( channel_obj.context_handle )

  -- Set the socket type
  channel_obj.socket_handle = zmq.zmq_socket( channel_obj.context_handle, zmq.ZMQ_PUB );
  assert( channel_obj.socket_handle );

  -- Bind to a message pipeline
  local rc = zmq.zmq_connect( channel_obj.socket_handle, channel_obj.name )
  assert (rc == 0);

  -- Set up the sending object
  function channel_obj.send( self, msg_buf, msg_buf_sz, has_more )
		local msg_sz = msg_buf_sz;
		if not msg_sz then
			local msg_type = type(msg_buf)
			if msg_type=="userdata" then
				error_msg = string.format("SimpleIPC (%s): Specified userdata with no size");
				return error_msg;
			elseif msg_type=="number" then
				msg_buf = ffi.new("double[?]",1,msg_buf)
				msg_sz = ffi.sizeof("double")
			elseif msg_type=="string"	then
				msg_sz = #msg_buf
			else
				error_msg = string.format("SimpleIPC (%s): Type (%s) not implemented",msg_type);
				return error_msg;
			end
		end
		
		local send_flags = 0;
		if has_more then 
			send_flags = zmq.ZMQ_SNDMORE 
		end
    local n_bytes_sent = zmq.zmq_send( 
			self.socket_handle, msg_buf, msg_sz, send_flags
    )
		if n_bytes_sent==-1 then
			local error_msg = ffi.string( zmq.zmq_strerror( zmq.zmq_errno() ) );
			error_msg = string.format("SimpleIPC (%s): %s",self.name ,error_msg)
			return error_msg
		elseif n_bytes_sent~=msg_sz then
			error_msg = string.format("SimpleIPC (%s): Sent: %d | Requested: ",
				n_bytes_sent, msg_buf_sz)
			return error_msg
		end
    return n_bytes_sent
  end

  return channel_obj;
end

function simple_ipc.setup_subscriber( channel )
  local channel_obj = {}
	channel_obj.name = "ipc:///tmp/"..channel;
  channel_obj.context_handle = zmq.zmq_init( simple_ipc.n_zmq_threads )
  assert( channel_obj.context_handle )

  -- Set the socket type
  channel_obj.socket_handle = zmq.zmq_socket( channel_obj.context_handle, zmq.ZMQ_SUB );
  assert( channel_obj.socket_handle );

  -- Bind to a message pipeline
  local rc = zmq.zmq_bind( channel_obj.socket_handle, channel_obj.name )
  assert (rc == 0);
  zmq.zmq_setsockopt( channel_obj.socket_handle, zmq.ZMQ_SUBSCRIBE, '', 0 )

	-- Set up receiving object
	function channel_obj.receive( self, msg_buf, msg_buf_sz )
		-- If a size is not given, then it is evident to ffi
		local msg_sz = msg_buf_sz;
		if not msg_sz then
			msg_sz = ffi.sizeof(msg_buf)
		end
		
	  local n_bytes_recv = zmq.zmq_recv( 
	  	self.socket_handle, msg_buf, msg_sz, 0
	  )
		if n_bytes_recv==-1 then
			local error_msg = ffi.string( zmq.zmq_strerror( zmq.zmq_errno() ) );
			error_msg = string.format("SimpleIPC (%s): %s",self.name ,error_msg)
			return error_msg
		end
		local option_value = ffi.new('int[1]',0)
		local option_len = ffi.new('size_t[1]',ffi.sizeof('int'))
		local has_more = zmq.zmq_getsockopt(
			self.socket_handle,zmq.ZMQ_RCVMORE,
			option_value,option_len)
		if option_value[0]==1 then
			return n_bytes_recv, true;
		end
  	return n_bytes_recv;
	end

  return channel_obj;
end

return simple_ipc