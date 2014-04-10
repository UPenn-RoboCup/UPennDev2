----------------------------------------
-- Touchscreen Motion processor
-- Listens to touch messages from ZeroMQ
-- (c) Stephen McGill, 2013
----------------------------------------
dofile'include.lua'
-- Libraries
local unix = require'unix'
local util = require'util'
local mp = require'msgpack'
local simple_ipc, poller = require'simple_ipc'
local tou_ch = simple_ipc.new_subscriber'touch'
local libTouch = require'libTouch'
-- Allow logging
local DO_LOG, libLog, logger = false
if DO_LOG then
	libLog = require'libLog'
	-- Make the logger
	logger = libLog.new'touch'
end

-- Callback for processing
tou_ch.callback = function(s)
	local t = unix.time()
	local data, has_more = tou_ch:receive()
	local evt = mp.unpack(data)
	if DO_LOG then
		evt.TIMESTAMP = t
		logger:record(evt)
	end
	local ts = evt.t/1e3
	local f = libTouch[evt.e]
	if type(f)~='function' then return end
	-- Process a non-touch event
	if not evt.touch then
		f(ts,evt)
	else
		-- Process the touches
		for _,c in ipairs(evt.touch) do
			f(ts,c)
		end
	end
end

local signal = require'signal'
local function shutdown()
	-- Stop the poller
	poller:stop()
	-- Save the logs
	if DO_LOG then logger:stop() end
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Start listening with the poller
poller = simple_ipc.wait_on_channels{tou_ch}
poller:start()
