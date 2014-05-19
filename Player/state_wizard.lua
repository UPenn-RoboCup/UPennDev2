---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
require'gcm'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time, bupdate, usleep = Body.get_time, Body.update, unix.usleep

-- Cleanly exit on Ctrl-C
local running, signal = true, nil
if not IS_WEBOTS then
  signal = require'signal'
  function shutdown () running = false end
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

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

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()

-- Entry
Body.entry()
for _,my_fsm in pairs(state_machines) do my_fsm:entry() end

-- Update loop
while running do
  t = get_time()
  -- Update the body
  bupdate()
  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
  -- If not webots, then wait the update cycle rate
  if not IS_WEBOTS then usleep(1e6 * (t_sleep - (get_time() - t))) end
  -- Run garbage collection each cycle so that we have consistent timing
  collectgarbage()
end

-- Exit
print'Exiting state wizard...'
for _,my_fsm in pairs(state_machines) do my_fsm:exit() end
Body.exit()
