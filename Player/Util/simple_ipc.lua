local ffi = require 'ffi'
local zmq = require 'ffi/zmq' -- Based on ZMQ
local simple_ipc = {} -- Our module

-- On the require, find the interfaces
local f_ifconfig = io.popen( 'ifconfig -l' )
local interface_list = f_ifconfig:read()
f_ifconfig:close()
for interface in string.gmatch(interface_list, "[%a|%d]+") do 
	local f_ifconfig = io.popen( "ifconfig "..interface.." | grep 'inet ' | cut -d ' ' -f 2" )
	local interface_ip = f_ifconfig:read()
	if interface_ip then
		local subnet_search = string.gmatch(interface_ip, "192.168.123.%d+")
		local addr = subnet_search()
		if addr then
			simple_ipc.intercom_interface = interface
			simple_ipc.intercom_interface_ip = interface_ip
		end
	end
	f_ifconfig:close()
end

-- Simple number of threads
simple_ipc.n_zmq_threads = 1
simple_ipc.local_prefix = 'ipc:///tmp/'
-- Set the intercomputer interface
if simple_ipc.intercom_interface then
	print( string.format('Selecting (%s) as the inter-pc interface\nUsing address (%s)',
	simple_ipc.intercom_interface, simple_ipc.intercom_interface_ip) );
	simple_ipc.intercom_prefix = 'epgm://'..simple_ipc.intercom_interface_ip..';239.192.1.1:'
else
	print( 'There is no inter-pc interface, using TCP' )
	simple_ipc.intercom_prefix = 'tcp://*:'
end

-- If channel is a number, then use tcp
local function setup_publisher( channel )
	local channel_obj = {}
	local channel_type = type(channel)
	if channel_type=="string" then
		channel_obj.name = simple_ipc.local_prefix..channel
	elseif channel_type=="number" then
		channel_obj.name = simple_ipc.intercom_prefix..channel
	end
	assert(channel_obj.name)
	print('Publishing on',channel_obj.name)
	
  channel_obj.context_handle = zmq.zmq_init( simple_ipc.n_zmq_threads )
  assert( channel_obj.context_handle )

  -- Set the socket type
  channel_obj.socket_handle = zmq.zmq_socket( channel_obj.context_handle, zmq.ZMQ_PUB );
  assert( channel_obj.socket_handle );

  -- Bind to a message pipeline
--  local rc = zmq.zmq_connect( channel_obj.socket_handle, channel_obj.name )
  local rc = zmq.zmq_bind( channel_obj.socket_handle, channel_obj.name )
  assert (rc == 0, print(ffi.string( zmq.zmq_strerror( zmq.zmq_errno() ) )));

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
simple_ipc.setup_publisher = setup_publisher

local function setup_subscriber( channel )
	local channel_obj = {}
	local channel_type = type(channel)
	if channel_type=="string" then
		channel_obj.name = simple_ipc.local_prefix..channel
	elseif channel_type=="number" then
		channel_obj.name = simple_ipc.intercom_prefix..channel
	end
	assert(channel_obj.name)
	print('Subscribing on',channel_obj.name)

  channel_obj.context_handle = zmq.zmq_init( simple_ipc.n_zmq_threads )
  assert( channel_obj.context_handle )

  -- Set the socket type
  channel_obj.socket_handle = zmq.zmq_socket( channel_obj.context_handle, zmq.ZMQ_SUB );
  assert( channel_obj.socket_handle );

  -- Bind to a message pipeline
--  local rc = zmq.zmq_bind( channel_obj.socket_handle, channel_obj.name )
  local rc = zmq.zmq_connect( channel_obj.socket_handle, channel_obj.name )
  assert (rc == 0, print(ffi.string( zmq.zmq_strerror( zmq.zmq_errno() ) )));
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
simple_ipc.setup_subscriber = setup_subscriber

local function wait_on_channels( channels )
	local poll_obj = {}
	local poll_items = ffi.new('zmq_pollitem_t[?]',#channels)
	for i=1,#channels do
		poll_items[i-1].socket = channels[i].socket_handle
		poll_items[i-1].events = zmq.ZMQ_POLLIN;
	end
	poll_obj.poll_items = poll_items
	poll_obj.nitems = #channels;
	function poll_obj.wait_on_any( self, timeout )
		local nevents = zmq.zmq_poll(self.poll_items, self.nitems, timeout);
		-- TODO: return which channels have been updated
		event_ids = {}
		if nevents>0 then
			for i=1,self.nitems do
				if poll_items[i-1].revents>0 then
					table.insert(event_ids,i)
				end
			end
		end
		return nevents, event_ids
	end
	return poll_obj;
end
simple_ipc.wait_on_channels = wait_on_channels

return simple_ipc