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

-- TODO: make libTouch instead...
local contacts = {}

local function pstart(c,ts)
	-- TODO: Make more efficient somehow...?
	contacts[c.id] = {
		x = c.x,
		y = c.y,
		t = ts
	}
	util.ptable(contacts)
end

local function pend(c,ts)
	-- Remove from table
	local contact = contacts[c.id]
	util.ptable(contact)
	contacts[c.id] = nil
end

-- Processing hashtable
local process = {}
process.start = pstart
process['end'] = pend
-- Callback for processing
tou_ch.callback = function(s)
	local data, has_more = tou_ch:receive()
	local evt = mp.unpack(data)
	local ts = evt.t/1e3
	local f = process[evt.e]
	if type(f)~='function' then return end
	-- Process a non-touch event
	if not evt.touch then
		f(ts)
	else
		-- Process the touches
		for _,c in ipairs(evt.touch) do
			f(c,ts)
		end
	end
end

-- Start listening with the poller
poller = simple_ipc.wait_on_channels{tou_ch}
poller:start()