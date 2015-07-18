#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

--local constant = 268
--local start_idx = 260
local start_idx = 201

require'wcm'
wcm.set_goal_disable({0})
wcm.set_robot_reset_pose({0})
require'gcm'
gcm.set_game_state({3})

local function pairmin(t)
	-- find the minimum element in the array table
	-- returns the min value and its index
	local imin = nil
	local tmin = math.huge
	for i, v in pairs(t)  do
		if v < tmin then
			tmin = v
			imin = i
		end
	end
	return tmin, imin
end

-- In lab Saturday
local logs = {
}

logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/day2_9am_B',
	'07.17.2015.21.04.11',
	'07.17.2015.21.04.35',
	'07.17.2015.21.04.59',
	'07.17.2015.21.05.23',
	'07.17.2015.21.05.47',
	'07.17.2015.21.06.11',
	'07.17.2015.21.06.34',
	'07.17.2015.21.06.58',
	'07.17.2015.21.07.22',
	'07.17.2015.21.07.46',
	'07.17.2015.21.08.10',
	'07.17.2015.21.08.34',
	'07.17.2015.21.08.57',
	'07.17.2015.21.09.21',
	'07.17.2015.21.09.45',
	'07.17.2015.21.10.09',
	'07.17.2015.21.10.33',
	'07.17.2015.21.10.57',
	'07.17.2015.21.11.21',
	'07.17.2015.21.11.44',
	'07.17.2015.21.12.08',
	'07.17.2015.21.12.31',
	'07.17.2015.21.12.55',
	'07.17.2015.21.13.19',
	'07.17.2015.21.13.43',
}
--[[
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/day2_9am',
	--'07.17.2015.20.56.44',
	--'07.17.2015.20.57.08',
	--'07.17.2015.20.57.32',
	--'07.17.2015.20.57.55',
	--'07.17.2015.20.58.19',
	--'07.17.2015.20.58.43',
	--'07.17.2015.20.59.06',
	--'07.17.2015.20.59.30',
	--'07.17.2015.20.59.53',
	--'07.17.2015.21.00.17',
	'07.17.2015.21.00.40',
	'07.17.2015.21.01.04',
	'07.17.2015.21.01.27',
	'07.17.2015.21.01.51',
	'07.17.2015.21.02.15',
	'07.17.2015.21.02.38',
	'07.17.2015.21.03.02',
	'07.17.2015.21.03.26',
}
--]]

--[[
logs.yuyv = {
ch = si.new_publisher'camera0',
dir = HOME..'Data'..'/onfield',
	--'07.17.2015.08.43.54',
	--'07.17.2015.08.44.15',
	--'07.17.2015.08.44.37',
	--'07.17.2015.08.44.58',
	--'07.17.2015.08.49.36',
	--'07.17.2015.08.49.58',
	--'07.17.2015.08.50.20',
	--'07.17.2015.08.50.42',

	-- This is a bad frame: ball in goalpost.
	-- Fix with localization...
	--local constant = 742

	'07.17.2015.08.51.04',
	'07.17.2015.08.51.26',
	'07.17.2015.08.51.47',
	'07.17.2015.08.52.09',
	'07.17.2015.08.52.31',
	'07.17.2015.08.52.53',
	'07.17.2015.08.53.15',
	'07.17.2015.08.53.36',
	'07.17.2015.08.54.42',
	'07.17.2015.08.55.04',
	'07.17.2015.08.55.26',
	'07.17.2015.08.57.51',
	'07.17.2015.08.58.12',
	'07.17.2015.08.58.34',
	'07.17.2015.08.58.56',
	'07.17.2015.08.59.18',
	'07.17.2015.08.59.40',
	'07.17.2015.09.00.02',
	'07.17.2015.09.00.24',
	'07.17.2015.09.00.45',
	'07.17.2015.09.01.07',
	'07.17.2015.09.01.29',
	'07.17.2015.09.01.51',
	'07.17.2015.09.02.13',
	'07.17.2015.09.02.34',
	'07.17.2015.09.02.56',
	'07.17.2015.09.03.18',
	'07.17.2015.09.03.40',
	'07.17.2015.09.04.02',

}
--]]
--[[
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/ucla4',
	'07.14.2015.18.30.36',
	'07.14.2015.18.30.46',
	'07.14.2015.18.30.56',
	'07.14.2015.18.31.06',
	'07.14.2015.18.31.16',
	'07.14.2015.18.31.25',
	'07.14.2015.18.31.34',
	'07.14.2015.18.31.42',
	'07.14.2015.18.31.50',
	'07.14.2015.18.31.58',
	'07.14.2015.18.32.06',
	'07.14.2015.18.32.14',
	'07.14.2015.18.32.22',
	'07.14.2015.18.32.30',
	'07.14.2015.18.32.38',
	'07.14.2015.18.32.46',
	'07.14.2015.18.32.54',
	'07.14.2015.18.33.03',
	'07.14.2015.18.33.13',
	'07.14.2015.18.33.23',
	'07.14.2015.18.33.33',
}
--]]
--[[
logs.yuyv = {
	'07.14.2015.10.53.08',
	'07.14.2015.10.53.29',
	'07.14.2015.10.53.51',
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/grasp2',
}
--]]

-- Initialize the coroutines
for name, log in pairs(logs) do
	log.co = coroutine.create(function()
		local counter = 0
		for j, date in ipairs(log) do
			print(name,'opening',date)
			local log = libLog.open(log.dir, date, name)
			local metadata = log:unroll_meta()
			local it = log:log_iter()
			for i, meta, payload in it do
				counter = counter + 1
				coroutine.yield(counter, meta, payload)
			end
		end
	end)
end

-- Start the multiplexer
local t_next = {}
local data_next = {}
for name, log in pairs(logs) do
	print('Initializing', name)
	local ok, counter, meta, payload = coroutine.resume(log.co)
	t_next[name] = meta.t
	-- Strip off the logging stuff
	meta.tlog = nil
	meta.rsz = nil
	-- Save the next data to send
	data_next[name] = {mp.pack(meta), payload}
end
-- Begin sending
local t_cursor = pairmin(t_next)
local t0 = t_cursor
local cnt = 0
print('Sending!')
while true do
	cnt = cnt + 1
	local t_n, i = pairmin(t_next)
	if not i then break end
	local dt = t_n - t_cursor
	t_cursor = t_n
	-- Send
	local data = data_next[i]
	-- Custom start time...
	if t_cursor >= 0 then
		print(cnt, i, counter, 'Time:', t_n - t0)
		--print(i, dt, names[i])
		local speedup = 1
		-- Temporary loop hack
		if start_idx and cnt < start_idx then

		elseif not constant then
			unix.usleep( dt * 1e6 / speedup )
			logs[i].ch:send(data)
		elseif constant==cnt then
			while true do
				unix.usleep( dt * 1e6 / speedup )
				logs[i].ch:send(data)
			end
		end
	end
	-- Repopulate
	local ok, counter, meta, payload = coroutine.resume(logs[i].co)
	if type(meta)=='table' then
		t_next[i] = meta.t
		meta.tlog = nil
		meta.rsz = nil
		data_next[i] = {mp.pack(meta), payload}
	else
		t_next[i] = math.huge
	end
end
print('Done!')
