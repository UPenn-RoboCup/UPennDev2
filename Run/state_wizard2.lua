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
local signal = require'signal'
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

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
for sm, en in pairs(Config.fsm.enabled) do
	local co = coroutine.create(co_fsm)
	state_threads[sm] = co
	-- Initial setup
	local status, msg = coroutine.resume(co, sm, en)
	if not status then print(msg) end
end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0 = get_time()
local t_debug = t0
local debug_interval = 5.0
repeat
  local t = get_time()

  -- Update the state machines
  for name,th in pairs(state_threads) do
		if coroutine.status(th)=='suspended' then
			local status, msg = coroutine.resume(th, running)
			if not status then print(string.format('%s: %s', util.color(name, 'red'), msg)) end
		end
	end
		-- If time for debug
	if t-t_debug>debug_interval then
		t_debug = t
		local kb = collectgarbage('count')
		print(string.format('State | Uptime: %.2f sec, Mem: %d kB', t-t0, kb))
  end

	collectgarbage('step')
	local t_s = (t_sleep - (get_time() - t))
	if t_s>0 then usleep(1e6 * t_s) end
until not running
