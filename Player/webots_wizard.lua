---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
require'gcm'
local Body = require(Config.dev.body)

-- Cleanly exit on Ctrl-C
local running = true
function shutdown () running = false end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Load the FSMs and attach event handler
local state_machines = {}
for _,sm in ipairs(Config.fsm.enabled) do
  local my_fsm = require(sm..'FSM')
  -- TODO: Check that this SHM segment exists...
  my_fsm.sm:set_state_debug_handle( function(cur_state_name, event)
    gcm['set_fsm_'..sm](cur_state_name)
  end)
  state_machines[sm] = my_fsm
  print('FSM | Loaded', sm)
end

-- Start the state machines
local t0 = Body.get_time()
local t_debug = t0

-- Entry
Body.entry()
for _,my_fsm in pairs(state_machines) do my_fsm:entry() end

-- Update loop
while running do
  local t = Body.get_time()
  -- Update the body
  Body.update()
  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
end

-- Exit
print'Exiting state wizard...'
for _,my_fsm in pairs(state_machines) do my_fsm:exit() end
Body.exit()
