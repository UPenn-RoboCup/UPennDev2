---------------------------------
-- Simple Interface to Lua's 
-- Version 2 with lzmq from moteus
-- ZeroMQ wrapper for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------
local zmq, poller, llthreads, CTX
-- LEGACY means we use zmq 3.x with lua-zmq from Neopallium
-- Else, use lzmq from moetus, which supports zmq 4.x
local LEGACY = false
local USE_FFI = false
if type(jit) == 'table' then USE_FFI = true end
if LEGACY then
	-- lua-zmq
	zmq    = require'zmq'
	poller = require'zmq/poller'
elseif USE_FFI then
	-- lzmq
	zmq    = require'lzmq.ffi'
	poller = require'lzmq.ffi.poller'
	llthreads = require'llthreads'
else
	-- lzmq
	zmq    = require'lzmq'
	poller = require'lzmq.poller'
	llthreads = require'llthreads'
end

local simple_ipc = {} -- Our module

-- Available for simple_ipc
local N_THREAD_POOL = 1

-- Lookup table helper
-- Easily specify the channel
local type2prefix = {
	string = function(s)
		-- Prefix string with # for inproc
		local is_inproc = s:byte(1,1)==35
		local name
		if is_inproc then
			name = 'inproc://'..s
		else
			name = 'ipc:///tmp/'..s
		end
		-- Postfix string with ! for inverted
		local is_inverted = s:byte(-1,-1)==33
		return name, is_inverted
	end,
	number = function(n, target)
		-- TODO: Specify inverted
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
	local has_more
	-- Remove old API of lua-zmq
	if LEGACY then
		has_more = s:getopt( zmq.RCVMORE )
	else
		has_more = s:get_rcvmore()
	end
	return ret, has_more==1
end

-- Make a new publisher
simple_ipc.new_publisher = function( channel, target )
	-- Form the prefix
  local ch_name, connect = type2prefix[type(channel)](channel,target)
	assert(ch_name,'PUBLISH | Bad prefix!')
	-- Grab or creat the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.PUB )
  assert( ch_socket, 'PUBLISH | Bad socket!' )
  -- Attach to the channel
	local is_bind = false
  if connect then
    ch_socket:connect( ch_name )
  else
    ch_socket:bind( ch_name )
		is_bind = true
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		name = ch_name,
		is_bind = is_bind,
	}
end

-- Make a new subscriber
simple_ipc.new_subscriber = function( channel, target )
	-- Form the prefix
  local ch_name, bind = type2prefix[type(channel)](channel,target)
	assert(ch_name,'SUBSCRIBE | Bad prefix!')
	-- Grab or creat the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.SUB )
  assert( ch_socket, 'SUBSCRIBE | Bad socket!' )
	-- Set the subscribe flag with no filters
	if LEGACY then
		ch_socket:setopt( zmq.SUBSCRIBE, '', 0 )
	else
		ch_socket:set_subscribe''
	end
  -- Attach to the channel
	local is_bind = false
  if bind then
    ch_socket:bind( ch_name )
		is_bind = true
  else
    ch_socket:connect( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
	}
end

-- Make a new requester
simple_ipc.new_requester = function( channel, target )
	-- Form the prefix
  local ch_name, bind = type2prefix[type(channel)](channel,target)
	assert(ch_name,'REQUEST | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.REQ )
  assert( ch_socket, 'REQUEST | Bad socket!' )
  -- Attach to the channel
	local is_bind = false
  if bind then
    ch_socket:bind( ch_name )
		is_bind = true
  else
    ch_socket:connect( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
	}
end

-- Make a new replier
simple_ipc.new_replier = function( channel, target )
	-- Form the prefix
  local ch_name, connect = type2prefix[type(channel)](channel,target)
	assert(ch_name,'REPLIER | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.REP )
  assert( ch_socket, 'REPLIER | Bad socket!' )
  -- Attach to the channel
	local is_bind = false
  if connect then
    ch_socket:connect( ch_name )
  else
    ch_socket:bind( ch_name )
		is_bind = true
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
	}
end

-- Make a new pair
simple_ipc.new_pair = function( channel, is_parent )
	-- Form the prefix
  local ch_name, connect = type2prefix[type(channel)](channel)
	assert(ch_name,'PAIR | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
  -- Set the socket type
  local ch_socket = CTX:socket( zmq.PAIR )
  assert( ch_socket, 'PAIR | Bad socket!' )
  -- Attach to the channel
	local is_bind = false
  if is_parent then
    ch_socket:bind( ch_name )
		is_bind = true
  else
    ch_socket:connect( ch_name )
  end
	-- Return the table
  return {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
	}
end

-- Return a ZMQ Poller object based on the set of channels
-- Callbacks set in the code
simple_ipc.wait_on_channels = function( channels )
	assert(type(channels)=='table', 'Bad wait channels!')
	local n_ch = #channels
  local poll_obj = poller.new( n_ch )
  -- Add local lookup table for the callbacks
  local lut = {}
  for i,ch in ipairs(channels) do
		local s = ch.socket
		assert(s,'No socket for poller!')
		assert(not lut[s],'Duplicate poller channel!')
		assert(ch.callback,'No callback for poller!')
    poll_obj:add( ch.socket, zmq.POLLIN, ch.callback )
		lut[s] = ch
  end
	poll_obj.lut = lut
	poll_obj.n = n_ch
	poll_obj.clean = function(self,s)
		-- NOTE: may only be able to remove the last added socket...
		-- That means arbitrary removal is bad...
		self:remove(s)
		self.lut[s] = nil
		self.n = self.n - 1
		return self.n
	end
  return poll_obj
end

simple_ipc.import_context = function( existing_ctx )
	CTX = zmq.init_ctx(existing_ctx)
end

-- Make a thread with a channel
simple_ipc.new_thread = function(scriptname, channel, metadata)
	--local t_ch = type(channel)
	assert(type(channel)=='string','Must given a comm channel string')
	-- Make the communication channel name
	--local ch_name = 'inproc://'..channel
	--local ch_name, connect = type2prefix[t_ch](channel)
	-- llthread only accepts a string
	-- TODO: Add a patch to take a file name
	-- TODO: Add the ability to take in a function
	-- Really, this is just for readability and reusability :)
	-- I don't want to do: [[ code... ]] all over the place
	local f = io.open(scriptname,'r')
	assert(f,'No script found!')
	local script_str = f:read'*all'
	f:close()
	
	-- Grab or creat the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
	
	-- Load the script into the child Lua state
	-- pass in the ctx, since it is thread safe
	-- Pass the filename per usual, so that non-thread
	-- execution of the script is the same
	-- in the scriptname file, recover values:
	-- scriptname, CTX, ch_name = ...
	metadata = metadata or {}
	metadata.ch_name = '#'..channel
	local thread = llthreads.new(script_str, CTX:lightuserdata(), metadata )
	
	-- Must add the communication...
	-- NOTE: It is the job of the script to
	-- ascertain if it was called as a thread
	-- (Should just check if it was given a context...
	-- Must call import_context in the thread to achieve communication
	local requester = simple_ipc.new_pair(metadata.ch_name,true)
	
	return thread, requester
end

return simple_ipc
