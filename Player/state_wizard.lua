#!/usr/bin/env luajit
---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
--require'gcm'
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
    local set_gcm_fsm = gcm and gcm['set_fsm_'..sm]
    if set_gcm_fsm then
      my_fsm.sm:set_state_debug_handle(function(cur_state_name, event)
        set_gcm_fsm(cur_state_name)
      end)
      set_gcm_fsm('UNKNOWN')
    end
    state_machines[sm] = my_fsm
    print('State | Loaded', sm)
  end
end

if not Config.fsm.disabled then load_fsm() end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()
local debug_interval, t_debug = 2.0, t0

-- Entry
for _, my_fsm in pairs(state_machines) do
  my_fsm:entry()
end
if IS_WEBOTS then
  Body.entry()
  Body.update()
end
-- Update loop
local gc_pktNum = 0

local t_switch = get_time()
local last_position = nil

local rgb = {255,255,255}
gcm.set_game_role(Config.default_role or 2) --Testing
gcm.set_game_state(Config.default_state or 5) --Pre-init

while running do
  t = get_time()
  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
  -- If time for debug
  if t-t_debug>debug_interval then
--    os.execute('clear')
    t_debug = t
		print(string.format('State | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
    --print('Wire', vcm.get_wire_model())
--    print("GCM ROLE:",gcm.get_game_role())
--    print("GCM STATE:",gcm.get_game_state())


	end

  local  current_lidar_deg = Body.get_lidar_position()[1] * RAD_TO_DEG
--print(current_lidar_deg)

  if t-t_switch>1.0 then
    t_switch = t
    local  current_lidar_deg = Body.get_lidar_position()[1] * RAD_TO_DEG
--print(current_lidar_deg)
    local cur_position 
    if current_lidar_deg>45 then
      cur_position=1
      rgb={255,0,0}
    elseif current_lidar_deg<-45 then
      cur_position = -1
      rgb={0,0,255}
    else
      cur_position = 0
      rgb={255,0,255}

    end
--    print(cur_position, rgb[1],rgb[3])
    if cur_position~=last_position then
      last_position = cur_position
      if cur_position == 1 then
        gcm.set_game_role(1)
        gcm.set_game_state(0)
      elseif cur_position == -1 then
        gcm.set_game_role(0) --Zero: goalie
        gcm.set_game_state(0)
      else
        gcm.set_game_role(2) --tester role
        gcm.set_game_state(6) --tester state
      end


  local intensity = 1.0
  Body.set_head_led_red(rgb[1]*intensity)
  Body.set_head_led_blue(rgb[3]*intensity)
  Body.set_head_led_green(rgb[3]*intensity)


    end

  end


  -- If not webots, then wait the update cycle rate
  if IS_WEBOTS then
    Body.update()
  else
    collectgarbage('step')
    local t_s = (t_sleep - (get_time() - t))
    if t_s>0 then usleep(1e6 * t_s) end
  end
end

-- Exit
print'Exiting state wizard...'
for _,my_fsm in pairs(state_machines) do
  my_fsm:exit()
end

os.exit()
