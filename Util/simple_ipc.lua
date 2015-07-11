---------------------------------
-- Simple Interface to Lua's
-- ZeroMQ wrapper for Team THOR
-- Version 2 with lzmq from moteus
-- Version 2 adds pthreads with llthreads2 from moteus
-- (c) Stephen McGill, 2014
---------------------------------
local zmq, poller, llthreads, udp, CTX
if type(jit)=='table' then
	zmq    = require'lzmq.ffi'
	poller = require'lzmq.ffi.poller'
	llthreads = require'llthreads'
else
	zmq    = require'lzmq'
	poller = require'lzmq.poller'
	llthreads = require'llthreads'
end
udp = require'udp'

local simple_ipc = {}

-- Available for simple_ipc
local N_THREAD_POOL = 2

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
		local name, is_inverted = nil, false
		if type(target)~='string' then
			name = 'tcp://*:'..n
		else
			name = 'tcp://'..target..':'..n
		end
		return name, is_inverted
	end,
}

-- Use UDP directly
if type(udp)=='table' and not udp.ffi then
	local function udp_send(self, data, uuid)
		-- Single send for now...
		return self.sender:send(data)
		--return self.sender:send_all(data)
		-- Double send since packet loss is a possibility
		--[[
		local ret, uuid = self.sender:send_all(data)
		return self.sender:send_all(data, uuid)
		--]]
	end

	local function udp_send_triple(self, data)
		-- Double send since packet loss is a possibility
		local ret, uuid = self.sender:send_all(data)
		self.sender:send_all(data, uuid)
		return self.sender:send_all(data, uuid)
	end

	local function udp_receive(self)
		return self.receiver:receive()
	end
	local function udp_descriptor(self)
		return (self.sender or self.receiver):descriptor()
	end
	local function recv_all(self)
		local skt_buffer = {}
		while self:size() > 0 do tinsert(skt_buffer, self:receive()) end
		return skt_buffer
	end
	local function udp_size(self)
		return self.receiver:size()
	end
	function simple_ipc.new_sender(ip, port)
		local ok, sender = pcall(udp.new_sender, ip, port)
		if not ok then return end
		local obj = {
			sender = sender,
			ip = ip,
			port = port,
			fd = sender:descriptor(),
			send = udp_send,
			send_triple = udp_send_triple,
			close = close,
		}
		return obj
	end
	function simple_ipc.new_receiver(port)
		local receiver = udp.new_receiver(port)
		local obj = {
			receiver = receiver,
			port = port,
			fd = receiver:descriptor(),
			receive = udp_receive,
			recv_all = udp_recv_all,
			size = udp_size,
		}
		return obj
	end
elseif type(udp)=='table' then
	simple_ipc.new_sender = udp.new_sender
	simple_ipc.new_receiver = udp.new_receiver
end

-- Set up the sending object
local function ch_send(self, messages)
	local tmsg, s, ret = type(messages), self.socket, nil
	if tmsg == "string" then
		ret = s:send(messages)
	elseif tmsg=="table" then
		ret = s:send_all(messages)
	end
	return ret
end

-- Set up receiving object
local function ch_receive(self, nonblock)
	return self.socket:recv_all(nonblock and zmq.DONTWAIT)
	--[[
	local ret = self.socket:recv_all(nonblock and zmq.DONTWAIT)
	if type(ret)=='table' and #ret==1 then
	return ret[1]
	else
	return ret
	end
	--]]
end

-- Set up receiving object
local function ch_send_and_receive(self, messages)
	ch_send(self, messages)
	return ch_receive(self)
end

-- Make a new publisher
function simple_ipc.new_publisher(channel, target)
	-- Form the prefix
	local ch_name, inverted = type2prefix[type(channel)](channel, target)
	assert(ch_name,'PUBLISH | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init(N_THREAD_POOL)
	-- Set the socket type
	local ch_socket = CTX:socket(zmq.PUB)
	assert(ch_socket, 'PUBLISH | Bad socket!')
	-- Attach to the channel
	local is_bind = false
	if inverted then
		ch_socket:connect(ch_name)
	else
		ch_socket:bind(ch_name)
		is_bind = true
	end
	-- Return the table
	local obj = {
		socket = ch_socket,
		send = ch_send,
		name = ch_name,
		is_bind = is_bind,
		kind = 'pub',
	}
	if type(ch_socket)=='table' then
		ch_socket.obj = obj
	end
	return obj
end

-- Make a new subscriber
function simple_ipc.new_subscriber(channel, target)
	-- Form the prefix
	local ch_name, inverted = type2prefix[type(channel)](channel, target)
	assert(ch_name,'SUBSCRIBE | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init(N_THREAD_POOL)
	-- Set the socket type
	local ch_socket = CTX:socket(zmq.SUB)
	assert(ch_socket, 'SUBSCRIBE | Bad socket!')
	-- Set the subscribe flag with no filters
	ch_socket:set_subscribe''
	-- Attach to the channel
	local is_bind = false
	if inverted then
		ch_socket:bind(ch_name)
		is_bind = true
	else
		ch_socket:connect(ch_name)
	end
	-- Return the table
	local obj = {
		socket = ch_socket,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
		kind = 'sub',
	}
	if type(ch_socket)=='table' then
		ch_socket.obj = obj
	end
	return obj
end

-- Make a new requester
function simple_ipc.new_requester(channel, target)
	-- Form the prefix
	local ch_name, inverted = type2prefix[type(channel)](channel, target)
	assert(ch_name, 'REQUEST | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init(N_THREAD_POOL)
	-- Set the socket type
	local ch_socket = CTX:socket(zmq.REQ)
	assert(ch_socket, 'REQUEST | Bad socket!')
	-- Attach to the channel
	local is_bind = false
	if inverted then
		ch_socket:bind(ch_name)
		is_bind = true
	else
		ch_socket:connect(ch_name)
	end
	-- Return the table
	local obj = {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		send_recv = ch_send_and_receive,
		name = ch_name,
		is_bind = is_bind,
		kind = 'req',
	}
	if type(ch_socket)=='table' then
		ch_socket.obj = obj
	end
	return obj
end

-- Make a new replier
function simple_ipc.new_replier(channel, target)
	-- Form the prefix
	local ch_name, inverted = type2prefix[type(channel)](channel, target)
	assert(ch_name,'REPLIER | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init(N_THREAD_POOL)
	-- Set the socket type
	local ch_socket = CTX:socket(zmq.REP)
	assert(ch_socket, 'REPLIER | Bad socket!')
	-- Attach to the channel
	local is_bind = false
	if inverted then
		ch_socket:connect(ch_name)
	else
		ch_socket:bind(ch_name)
		is_bind = true
	end
	-- Return the table
	local obj = {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
		kind = 'rep',
	}
	if type(ch_socket)=='table' then
		ch_socket.obj = obj
	end
	return obj
end

-- Make a new pair
function simple_ipc.new_pair(channel, is_parent)
	-- Form the prefix
	local ch_name, connect = type2prefix[type(channel)](channel)
	assert(ch_name,'PAIR | Bad prefix!')
	-- Grab or create the context
	CTX = CTX or zmq.init( N_THREAD_POOL )
	-- Set the socket type
	local ch_socket = CTX:socket(zmq.PAIR)
	assert(ch_socket, 'PAIR | Bad socket!')
	-- Attach to the channel
	local is_bind = false
	if is_parent then
		ch_socket:bind( ch_name )
		is_bind = true
	else
		ch_socket:connect( ch_name )
	end
	-- Return the table
	local obj = {
		socket = ch_socket,
		send = ch_send,
		receive = ch_receive,
		name = ch_name,
		is_bind = is_bind,
		kind = 'pair',
	}
	if type(ch_socket)=='table' then
		ch_socket.obj = obj
	end
	return obj
end

-- Make a thread with a channel
function simple_ipc.new_thread(scriptname, channel, metadata)
	-- Type checking
	assert(type(channel)=='string','Must given a comm channel string')
	assert(type(scriptname)=='string','Must givefilename for stript')

	local f = assert(io.open(scriptname,'r'),'No script found!')
	local script_str = f:read'*all'
	f:close()

	-- Grab or create the context
	CTX = CTX or zmq.init( N_THREAD_POOL )

	-- Load the script into the child Lua state
	-- pass in the ctx, since it is thread safe
	metadata = metadata or {}
	metadata.ch_name = '#'..channel
	local thread = llthreads.new(script_str, CTX:lightuserdata(), metadata)

	-- Must add the communication...
	-- NOTE: It is the job of the script to
	-- ascertain if it was called as a thread
	-- (Should just check if it was given a context...
	-- Must call import_context in the thread to achieve communication
	local pair = simple_ipc.new_pair(metadata.ch_name, true)
	pair.thread = thread

	return pair, thread
end

local function clean(self, s)
	-- NOTE: may only be able to remove the last added socket...
	-- That means arbitrary removal is bad...
	self:remove(s)
	self.lut[s] = nil
	self.n = self.n - 1
	return self.n
end

-- Return a ZMQ Poller object based on the set of channels
-- Callbacks set in the code
function simple_ipc.wait_on_channels(channels)
	assert(type(channels)=='table', 'Bad wait channels!')
	-- Remove dummy functions
	local n_ch = 0
	for _, ch in ipairs(channels) do
		if not ch.is_dummy then n_ch = n_ch + 1 end
	end	
	local poll_obj = poller.new(n_ch)
	-- Add lookup table for keeping track of duplicates
	local lut, s = {}
	for i, ch in ipairs(channels) do
		if not ch.is_dummy then
			-- UDP case is FD as a number
			s = ch.socket or ch.fd
			assert(s, 'No socket for poller!')
			assert(not lut[s], 'Duplicate poller channel!')
			assert(ch.callback, 'No callback for poller!')
			poll_obj:add(s, zmq.POLLIN, ch.callback)
			lut[s] = i
		end
	end
	poll_obj.lut = lut
	poll_obj.n = n_ch
	poll_obj.clean = clean
	return poll_obj
end

function simple_ipc.import_context(existing_ctx)
	CTX = zmq.init_ctx(existing_ctx)
end

local null_func = function() end

function simple_ipc.new_dummy()
	return {send=null_func, receive=null_func, is_dummy=true}
end

return simple_ipc
