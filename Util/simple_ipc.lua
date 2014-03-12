---------------------------------
-- Simple Interface to Lua's 
-- Version 2 with lzmq from moteus
-- ZeroMQ wrapper for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------
--[[
local zmq        = require 'zmq' -- Based on ZMQ
local poller     = require 'zmq/poller'
--]]
local zmq        = require'lzmq'
local poller     = zmq.poller

local simple_ipc = {} -- Our module

-- Available for simple_ipc
local N_THREAD_POOL = 1
local CTX = zmq.init( N_THREAD_POOL )

local type2prefix = {
	string = function(s)
		-- Prefix string with # for inproc
		-- Postfix string with ! for inverted
		local is_inproc = s:byte(1,1)==35
		local name
		if is_inproc then
			name = 'inproc://'..s
		else
			name = 'ipc:///tmp/'..s
		end
		local is_inverted = s:byte(-1,-1)==33
		return name, is_inverted
	end,
	number = function(n, target)
		--TCP_PREFIX:gsub('*',addr or 'localhost')..channel
		local name, is_inverted
		if type(target)~='string' then
			name = 'tcp://*:'..n
		else
			name = 'tcp://'..target..':'..n
		end
		return name, is_inverted
	end,
}

-- Set up the sending object
-- Supported: string, userdata, array of strings
local ch_send = function( self, messages, sz )
	local tmsg, s, ret = type(messages), self.socket, nil
	if tmsg == "string" then
		ret = s:send( messages )
	elseif tmsg=="table" then
		local nmessages = #messages
		for i, msg in ipairs(messages) do
			local more = (i==nmessages and zmq.SNDMORE) or nil
			ret = s:send( msg, more )
		end
		return ret
	elseif tmsg=="userdata" then
		-- TODO
	end
	return ret
end

-- Set up receiving object
local ch_receive = function( self, noblock )
	local s, ret = self.socket, nil
	if noblock then
		ret = s:recv(zmq.NOBLOCK)
	else
		ret = s:recv()
	end
	-- Check if there is more to be received
	--local has_more = s:getopt( zmq.RCVMORE )
	local has_more = s:get_rcvmore()
	return ret, has_more==1
end

-- Make a new publisher
simple_ipc.new_publisher = function( channel, target )
	-- Form the prefix
  local ch_name, connect = type2prefix[type(channel)](channel,target)
	assert(ch_name,'PUBLISH | Bad prefix!')
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.PUB )
  assert( ch_socket, 'PUBLISH | Bad socket!' )
  -- Attach to the channel
  if connect then
		print('CONNECT')
    ch_socket:connect( ch_name )
  else
		print('BIND')
    ch_socket:bind( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		name = ch_name,
	}
end

-- Make a new subscriber
simple_ipc.new_subscriber = function( channel, target )
	-- Form the prefix
  local ch_name, bind = type2prefix[type(channel)](channel,target)
	assert(ch_name,'SUBSCRIBE | Bad prefix!')
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.SUB )
  assert( ch_socket, 'SUBSCRIBE | Bad socket!' )
	-- Set the subscribe flag with no filters
	
	-- FIXME: API difference between lzmq and zmq
	--ch_socket:setopt( zmq.SUBSCRIBE, '', 0 )
	assert(ch_socket.set_subscribe,'Please use lzmq!')
	ch_socket:set_subscribe''
	
  -- Attach to the channel
  if bind then
		print('BIND')
    ch_socket:bind( ch_name )
  else
		print('CONNECT')
    ch_socket:connect( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		receive = ch_receive,
		name = ch_name,
	}
end

-- Make a new requester
simple_ipc.new_requester = function( channel, target )
	-- Form the prefix
  local ch_name, bind = type2prefix[type(channel)](channel,target)
	assert(ch_name,'REQUEST | Bad prefix!')
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.REQ )
  assert( ch_socket, 'REQUEST | Bad socket!' )
  -- Attach to the channel
  if bind then
    ch_socket:bind( ch_name )
  else
    ch_socket:connect( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
	}
end

-- Make a new replier
simple_ipc.new_replier = function( channel, target )
	-- Form the prefix
  local ch_name, connect = type2prefix[type(channel)](channel,target)
	assert(ch_name,'REPLIER | Bad prefix!')
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.REP )
  assert( ch_socket, 'REPLIER | Bad socket!' )
  -- Attach to the channel
  if connect then
    ch_socket:connect( ch_name )
  else
    ch_socket:bind( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
	}
end

-- Return a ZMQ Poller object based on the set of channels
-- Callbacks set in the code
simple_ipc.wait_on_channels = function( channels )
  local poll_obj = poller.new( #channels )
  -- Add lookup table for the callbacks
  local lut = {}
  for i,ch in ipairs(channels) do
    poll_obj:add( ch.socket_handle, zmq.POLLIN, ch.callback )
    assert(not lut[ch.socket_handle],'Duplicate poller channel!')
    lut[ch.socket_handle] = ch
  end
  return poll_obj, lut
end

return simple_ipc
