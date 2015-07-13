#!/usr/bin/env luajit
---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'../fiddle.lua'
-- Cache some functions
local get_time, usleep = unix.time, unix.usleep

-- Cleanly exit on Ctrl-C
local running = true
local function shutdown() running = false end

if not IS_WEBOTS then
	local signal = require'signal'
	signal.signal("SIGINT", shutdown)
	signal.signal("SIGTERM", shutdown)
	-- TODO: Check this out
	require'mcm'
end

local function co_fsm(sm, en)
	assert(type(sm)=='string', 'State | Not a proper name')
	assert(en, 'State | '..sm..' not enabled')
	local fsm = require(sm..'FSM')
	assert(type(fsm)=='table', "State | Bad FSM: "..sm)
	local set_gcm_fsm = gcm['set_fsm_'..sm]
	assert(set_gcm_fsm, 'State | No gcm entry for '..sm)
	--[[
	my_fsm.sm:set_state_debug_handle(function(cur_state_name, event)
		set_gcm_fsm(cur_state_name)
	end)
	--]]
	fsm.sm:set_state_debug_handle(set_gcm_fsm)
	set_gcm_fsm''
	-- Run the stuff
	fsm:entry()
	while coroutine.yield() do fsm:update() end
	fsm:exit()
end

local state_threads = {}
local state_times = {}
for sm, en in pairs(Config.fsm.enabled) do
	local co = coroutine.create(co_fsm)
	state_threads[sm] = co
	state_times[sm] = 0
	-- Initial setup
	local status, msg = coroutine.resume(co, sm, en)
	if not status then print(msg) end
end

if IS_WEBOTS then
  Body.entry()
  Body.update()
end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0 = get_time()
local t_debug = t0
local debug_interval = 5.0
local count = 0
repeat
	count = count + 1
  local t_start = get_time()

	--print()
  -- Update the state machines
  for name,th in pairs(state_threads) do
		if coroutine.status(th)~='dead' then
			local t00 = get_time()
			local status, msg = coroutine.resume(th, running)
			local t11 = get_time()
			state_times[name] = state_times[name] + (t11-t00)
			--print(name, t11-t00)
			if not status then print(string.format('%s: %s', util.color(name, 'red'), msg)) end
		end
	end

	--print('Total',get_time()-t_start)

	-- If time for debug
	local dt_debug = t_start - t_debug
	if dt_debug>debug_interval and not IS_WEBOTS then
		local batt = Body.get_lleg_voltage()
		local batt_ave = (batt[2]+batt[3]+batt[4])/3

		mcm.set_status_battery(batt_ave)


		local times_str = ""
		local total = 0
		for name,time in pairs(state_times) do
			--print(time, count)
  		  --only print out slow processes
		  if time>0.001 then
		    times_str=times_str..string.format('%s:%.2fms ', name, 1e3*time/count)
		  end
			total = total + time
			state_times[name] = 0
		end



		local kb = collectgarbage('count')
		print(string.format('\nState | Uptime: %.2f sec, Mem: %d kB, %.2f Hz %g ms cycle battery %.1fV\n%s',
				t_start-t0, kb, count/dt_debug, 1e3*total/count, batt_ave/10,times_str))
		count = 0
		t_debug = t_start
		--collectgarbage('step')
  end

	if IS_WEBOTS then
		Body.update()
	else
		local t_end = get_time()
		local t_s = (t_sleep - (t_end - t_start))
		if t_s>0 then usleep(1e6 * t_s) end
	end
until not running

print'Exiting state wizard...'



if IS_WEBOTS then
	wb_supervisor_simulation_revert()
end
