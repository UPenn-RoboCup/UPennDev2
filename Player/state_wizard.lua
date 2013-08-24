dofile'include.lua'
local Body = require'Body'
local util = require'util'
local mp = require'msgpack'
local simple_ipc = require'simple_ipc'
local state_pub_ch = simple_ipc.new_publisher(5544)

local state_machines = {}

local function broadcast_states(name)
  local status = {}
  for _,my_fsm in pairs(state_machines) do
    local cur_st = my_fsm.sm:get_current_state()
    print('FSM Status',my_fsm._NAME,cur_st._NAME)
    table.insert(status,{my_fsm._NAME,cur_st._NAME})
  end
  -- Broadcast over UDP/TCP/IPC
  local ret = state_pub_ch:send( mp.pack(status) )
end

-- TODO: Make coroutines for each FSM
-- TODO: Or other way of handling state machine failure
-- Maybe a reset() function in each fsm?
for _,sm in ipairs(unix.readdir(CWD)) do
  if sm:find'FSM' then
    package.path = CWD..'/'..sm..'/?.lua;'..package.path
    local my_fsm = require(sm)
    my_fsm.sm:set_state_debug_handle(broadcast_states)
    state_machines[sm] = my_fsm
    print( util.color('FSM | Loaded','yellow'),sm)
  end
end

-- Start the webots routine
local t0 = Body.get_time()
local t_debug = t0

-- Update rate
local fps = 100
local us_sleep = 1e6 / fps

-- Perform inialization
Body.entry()
for _,sm in pairs(state_machines) do sm.entry() end
while true do
  local t = Body.get_time()
  local t_diff = t-t_debug
  
  -- Update each state machine
  for _,sm in pairs(state_machines) do sm.update() end

  if t_diff>1 then
	  print( string.format('Running Time: %d seconds',t-t0) )
    t_debug = t
  end
  -- NOTE: Body is the main update, so it must not fail
	Body.update()
  
  -- Sleep a bit if not webots
  if not IS_WEBOTS then unix.usleep(us_sleep) end
  
end

Body.exit()