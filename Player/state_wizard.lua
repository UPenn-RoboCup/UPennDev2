---------------------------------
-- State Machine Manager for Team THOR
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'
local util = require'util'
local mp   = require'msgpack'
local signal = require'signal'
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
local vector     = require'vector'

-- Not using this right now...
local broadcast_en = false
local state_pub_ch
if broadcast_en then
  state_pub_ch = simple_ipc.new_publisher(Config.net.state)
end

require'gcm'
require'wcm'
require'jcm'
require'mcm'

local needs_broadcast = true
local state_machines = {}

local status = {}

-- TODO: Make coroutines for each FSM
-- TODO: Or other way of handling state machine failure
-- Maybe a reset() function in each fsm?
for _,sm in ipairs(Config.fsm.enabled) do
  local my_fsm = require(sm)
  my_fsm.sm:set_state_debug_handle(function(cur_state_name,event)
    -- For other processes
    gcm['set_fsm_'..sm](cur_state_name)
    -- Local copy
    local s = {cur_state_name,event}
    status[my_fsm._NAME] = s
    -- Broadcast requirement
    needs_broadcast = true
    -- Debugging printing
    --print(table.concat(s,' from '))
  end)
  state_machines[sm] = my_fsm
  print( util.color('FSM | Loaded','yellow'),sm)
end

-- Start the state machines
local t0 = Body.get_time()
local t_debug = t0

--------------------
-- Clean Shutdown function
function shutdown()
  print'Shutting down the state machines...'
  for _,my_fsm in pairs(state_machines) do
    my_fsm.exit()
    -- Print helpful message
    print('Exit',my_fsm._NAME)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Perform inialization
local entry = function()
	for _,my_fsm in pairs(state_machines) do
		my_fsm.entry()
		local cur_state = my_fsm.sm:get_current_state()
		local cur_state_name = cur_state._NAME
		local s = {cur_state_name,nil}
		status[my_fsm._NAME] = s
		if broadcast_en then
			local ret = state_pub_ch:send( mp.pack(status) )
		end
	end
end

local exit = function()
	for _,my_fsm in pairs(state_machines) do
		my_fsm.exit()
		local cur_state = my_fsm.sm:get_current_state()
		local cur_state_name = cur_state._NAME
		local s = {cur_state_name,nil}
		status[my_fsm._NAME] = s
		local ret = state_pub_ch:send( mp.pack(status) )
	end
end

local update = function(pulse)
	local t = Body.get_time()
	--print('State Update from Body pulse',pulse.t,t)
	-- Update each state machine
  for _,my_fsm in pairs(state_machines) do local event = my_fsm.update() end

  -- Broadcast state changes... why...? Need the stage as well?
  if broadcast_en and needs_broadcast then
    needs_broadcast = false
    -- Broadcast over UDP/TCP/IPC
    local ret = state_pub_ch:send( mp.pack(status) )
  end
end

-- Listen for pulses
local wait_channels = {}

local pulse_ch = simple_ipc.new_subscriber'pulse'
print'Receiving Body pulse'
pulse_ch.callback = function(sh)
	local ch = wait_channels.lut[sh]
	local ekg, has_more
	repeat
		-- Do not block
    data, has_more = ch:receive(true)
		--print('Pulse Data',type(data))
		if data then ekg = mp.unpack(data) end
	until not data
	-- Call the update
	update(ekg)
end

-- Make the poller
table.insert(wait_channels, pulse_ch)
local channel_timeout = 2 * Body.update_cycle * 1e3
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

entry()
while true do
  local npoll = channel_poll:poll(channel_timeout)
	-- Update anyway, just missing a cycle
	update()
end
exit()
