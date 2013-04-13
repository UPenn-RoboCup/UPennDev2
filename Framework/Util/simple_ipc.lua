local zmq = require 'zmq' -- Based on ZMQ
local poller = require 'zmq/poller'
local simple_ipc = {} -- Our module

--[[
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
--]]

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
	simple_ipc.intercom_prefix = 'tcp://127.0.0.1:'
end

-- Make a new publisher
-- Publish with a filter prefix on a (possibly) pre-existing channel
simple_ipc.new_publisher = function( channel, filter )
	local channel_obj = {}
	local channel_type = type(channel)
	if channel_type=="string" then
		channel_obj.name = simple_ipc.local_prefix..channel
	elseif channel_type=="number" then
		channel_obj.name = simple_ipc.intercom_prefix..channel
	end
	assert(channel_obj.name)
	print('Publishing on',channel_obj.name,'with filter',filter)
	
  channel_obj.context_handle = zmq.init( simple_ipc.n_zmq_threads )
  assert( channel_obj.context_handle )

  -- Set the socket type
  channel_obj.socket_handle = channel_obj.context_handle:socket( zmq.PUB );
  assert( channel_obj.socket_handle );

  -- Bind to a message pipeline
	-- TODO: connect?
  channel_obj.socket_handle:bind( channel_obj.name )
  
  -- Set the filter for sending messages
  channel_obj.filter = filter or '';

  -- Set up the sending object
  function channel_obj.send( self, messages )
    if type(messages) == "string" then
      return self.socket_handle:send( self.filter..messages );
    end
    local nmessages = #messages;
    local filter = self.filter;
    for i=1,nmessages do
      local msg = messages[i];
      -- TODO: Does this slow the process by a noticeable margin?
      assert( type(msg)=="string", 
        string.format("SimpleIPC (%s): Type (%s) not implemented",
        self.name, type(msg) )
        );
      if i==nmessages then
        return self.socket_handle:send( filter..msg );
      else
        ret = self.socket_handle:send( filter..msg, zmq.SNDMORE );
      end
    end
  end
  return channel_obj;
end

-- Make a new subscriber
simple_ipc.new_subscriber = function( channel, filter )
	local channel_obj = {}
	local channel_type = type(channel)
	if channel_type=="string" then
		channel_obj.name = simple_ipc.local_prefix..channel
	elseif channel_type=="number" then
		channel_obj.name = simple_ipc.intercom_prefix..channel
	end
	assert(channel_obj.name)
	print('Subscribing on',channel_obj.name)

  channel_obj.context_handle = zmq.init( simple_ipc.n_zmq_threads )
  assert( channel_obj.context_handle )

  -- Set the socket type
  channel_obj.socket_handle = channel_obj.context_handle:socket( zmq.SUB );
  assert( channel_obj.socket_handle );

  -- Store the filter
	channel_obj.filter = filter or '';
  -- Connect to a message pipeline
  local rc = channel_obj.socket_handle:connect( channel_obj.name )
  channel_obj.socket_handle:setopt( zmq.SUBSCRIBE, channel_obj.filter, 0 )

	-- Set up receiving object
	function channel_obj.receive( self )
	  local ret = self.socket_handle:recv();
		local has_more = self.socket_handle:getopt( zmq.RCVMORE )
    return ret, has_more==1;
  end

  return channel_obj;
end

-- Return a ZMQ Poller object based on the set of channels
simple_ipc.wait_on_channels = function( channels )
  local poll_obj = poller.new( #channels )
	for i=1,#channels do
    poll_obj:add( channels[i].socket_handle, zmq.POLLIN, channels[i].callback );--no callback yet
	end
	return poll_obj;
end

return simple_ipc