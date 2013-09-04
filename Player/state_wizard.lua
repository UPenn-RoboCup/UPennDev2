dofile'include.lua'
local Body = require'Body'
local util = require'util'
local mp   = require'msgpack'
local simple_ipc   = require'simple_ipc'
local state_pub_ch = simple_ipc.new_publisher(Config.net.state)

local state_machines = {}

local status = {}
local needs_broadcast = false
local function set_broadcast()
  needs_broadcast = true
end

-- TODO: Make coroutines for each FSM
-- TODO: Or other way of handling state machine failure
-- Maybe a reset() function in each fsm?
for _,sm in ipairs(Config.fsm.enabled) do
  local my_fsm = require(sm)
  my_fsm.sm:set_state_debug_handle(set_broadcast)
  state_machines[sm] = my_fsm
  print( util.color('FSM | Loaded','yellow'),sm)
end

-- Update rate (if not webots)
local fps = 120
local us_sleep = 1e6 / fps

-- Start the state machines
local t0 = Body.get_time()
local t_debug = t0

-- Perform inialization
Body.entry()
for _,my_fsm in pairs(state_machines) do
  my_fsm.entry()
  local cur_st = my_fsm.sm:get_current_state()
  status[my_fsm._NAME] = cur_st._NAME
end
while true do
  local t = Body.get_time()
  local t_diff = t-t_debug
  
  -- Update each state machine
  for _,my_fsm in pairs(state_machines) do
    local event  = my_fsm.update()
    local cur_st = my_fsm.sm:get_current_state()
    local s = {cur_st._NAME,event}
    status[my_fsm._NAME] = s
    if event then print(unpack(s)) end
  end

  if needs_broadcast then
    needs_broadcast = false
    -- Broadcast over UDP/TCP/IPC
    local ret = state_pub_ch:send( mp.pack(status) )
  end

  -- Update the body (mostly needed for webots)
	Body.update()
  
  -- Sleep a bit if not webots
  if not IS_WEBOTS then unix.usleep(us_sleep) end
  
end
Body.exit()
for _,my_fsm in pairs(state_machines) do
  my_fsm.exit()
  local cur_st = my_fsm.sm:get_current_state()
  status[my_fsm._NAME] = cur_st._NAME
  local ret = state_pub_ch:send( mp.pack(status) )
end