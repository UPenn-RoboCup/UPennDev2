#!/usr/bin/env luajit
---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'../fiddle.lua'
local Body = require('Body')
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep
-- Cleanly exit on Ctrl-C
local running = true
local function shutdown() running = false end
local signal
if not IS_WEBOTS then
  signal = require'signal'
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

local util = require'util'
local vector = require'vector'
require'mcm'
local lW, uOdometry0
if not IS_WEBOTS then
  lW=require'libWorld'
  lW.entry()
end

local state_ch = require'simple_ipc'.new_subscriber('state!')

-- Load the FSMs and attach event handler
local state_machines = {}
local function load_fsm ()
  for sm, en in pairs(Config.fsm.enabled) do
		if en then
			local my_fsm = require(sm..'FSM')
			assert(type(my_fsm)=='table', "Bad FSM: "..sm)
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
end

if not Config.fsm.disabled then load_fsm() end

-- Timing
local t_sleep
if not IS_WEBOTS then
	t_sleep = 1 / Config.fsm.update_rate
end
local t0, t = get_time()
local debug_interval, t_debug = 5.0, t0

-- Entry
for _, my_fsm in pairs(state_machines) do
  my_fsm:entry()
end
if IS_WEBOTS then
  Body.entry()
  Body.update()
end
-- Update loop
while running do
  t = get_time()

  local events = state_ch:receive(true)
  if events then
    for _, e in ipairs(events) do
      if e=='reset' and Body.WebotsBody then
        Body.WebotsBody.reset()
      end
    end
  end

  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
		-- If time for debug
	if t-t_debug>debug_interval then
		t_debug = t
		local kb = collectgarbage('count')
    if not IS_WEBOTS then --we don't need for webots
		  print(string.format('State | Uptime: %.2f sec, Mem: %d kB', t-t0, kb))
    end
  end

  if lW then
		local uOdometry = mcm.get_status_odometry()
    dOdometry = util.pose_relative(uOdometry, uOdometry0 or uOdometry)
		uOdometry0 = vector.copy(uOdometry)
    lW.update(dOdometry)
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

if lW then lW.exit() end

if IS_WEBOTS then
	wb_supervisor_simulation_revert()
end
