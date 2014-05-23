---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
require'gcm'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep

-- Cleanly exit on Ctrl-C
local running, signal = true, nil
if not IS_WEBOTS then
  signal = require'signal'
  function shutdown ()
    running = false
    --os.exit()
  end
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

-- Load the FSMs and attach event handler
local state_machines = {}
local function load_fsm ()
  for _,sm in ipairs(Config.fsm.enabled) do
    local my_fsm = require(sm..'FSM')
    local set_gcm_fsm = gcm['set_fsm_'..sm]
    if set_gcm_fsm then
      my_fsm.sm:set_state_debug_handle(function(cur_state_name, event)
        set_gcm_fsm(cur_state_name)
      end)
      set_gcm_fsm('UNKNOWN')
    end
    state_machines[sm] = my_fsm
    print('FSM | Loaded', sm)
  end
end

if Config.fsm.enabled then load_fsm() end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()
local debug_interval, t_debug = 1.0, t0

-- Entry
Body.entry()
for _,my_fsm in pairs(state_machines) do my_fsm:entry() end
-- Update loop
while running do
  t = get_time()
  -- Update the body
  Body.update()
  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
  -- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    --print('Joints', Body.get_larm_position())
  end
  -- If not webots, then wait the update cycle rate
  if not IS_WEBOTS then
    --collectgarbage()
    local t_s = (t_sleep - (get_time() - t))
    usleep(1e6 * t_s)
  end
end

-- Exit
print'Exiting state wizard...'
for _,my_fsm in pairs(state_machines) do my_fsm:exit() end
Body.exit()
