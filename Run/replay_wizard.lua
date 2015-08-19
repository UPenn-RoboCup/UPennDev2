#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

local constant
local start_idx

--local constant = 4590 -- weird frame for lines
--local start_idx = 4500

--local constant = 5445 -- weird frame for lines

--local constant = 5440 -- weird frame for lines
--local start_idx = 5150

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

----[[
start_idx = 201
--constant = 10
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/lastTest/',
	'07.21.2015.22.21.08',
	'07.21.2015.22.21.30',
	'07.21.2015.22.21.52',
	'07.21.2015.22.22.14',
	'07.21.2015.22.22.36',
	'07.21.2015.22.22.58',
	'07.21.2015.22.23.19',
	'07.21.2015.22.23.41',
	'07.21.2015.22.24.03',
}
--]]

--[[
--start_idx = 1
--constant = 10
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/1015Final/',
	'07.21.2015.22.05.48',
	'07.21.2015.22.06.09',
	'07.21.2015.22.06.31',
	'07.21.2015.22.06.53',
	'07.21.2015.22.07.15',
	'07.21.2015.22.07.37',
	'07.21.2015.22.07.59',
	'07.21.2015.22.08.20',
	'07.21.2015.22.08.42',
	'07.21.2015.22.09.04',
	'07.21.2015.22.09.26',
	'07.21.2015.22.09.48',
	'07.21.2015.22.10.10',
	'07.21.2015.22.10.32',
	'07.21.2015.22.10.53',
	'07.21.2015.22.11.15',
	'07.21.2015.22.11.37',
	'07.21.2015.22.11.59',
	'07.21.2015.22.12.21',
	'07.21.2015.22.12.42',
	'07.21.2015.22.13.04',
}
--]]

--[[
--start_idx = 1
--constant = 10
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/weird',
	'07.21.2015.22.28.18',
	'07.21.2015.22.28.40',
	'07.21.2015.22.29.01',
	'07.21.2015.22.29.23',
	'07.21.2015.22.29.45',
	'07.21.2015.22.30.07',
	'07.21.2015.22.30.29',
	'07.21.2015.22.30.50',
	'07.21.2015.22.31.12',
}
--]]

--[[
--start_idx = 1
--constant = 10
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/prepDay3',
	'07.20.2015.21.20.54',
	'07.20.2015.21.21.16',
	'07.20.2015.21.21.38',
	'07.20.2015.21.22.00',
	'07.20.2015.21.22.21',
	'07.20.2015.21.22.43',
	'07.20.2015.21.23.05',
	'07.20.2015.21.23.27',
	'07.20.2015.21.23.49',
	'07.20.2015.21.24.11',
	'07.20.2015.21.24.32',
	'07.20.2015.21.24.54',
	'07.20.2015.21.25.16',
	'07.20.2015.21.25.38',
}
--]]

--[[
start_idx = 100
--constant = 841
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/PrepDay2',
	--'07.19.2015.20.34.01',
	--'07.19.2015.20.37.38',
	--'07.19.2015.20.38.00',
	'07.19.2015.20.38.22',
	'07.19.2015.20.38.44',
	'07.19.2015.20.39.06',
	'07.19.2015.20.39.28',
	'07.19.2015.20.39.49',
	'07.19.2015.20.40.11',
	'07.19.2015.20.40.33',
	'07.19.2015.20.40.55',
	'07.19.2015.20.41.17',
	'07.19.2015.20.41.38',
	'07.19.2015.20.42.00',
	'07.19.2015.20.42.22',
	'07.19.2015.20.42.44',
	'07.19.2015.20.43.06',
	'07.19.2015.20.43.28',
	'07.19.2015.20.43.49',
	'07.19.2015.20.44.12',
	'07.19.2015.20.44.33',
	'07.19.2015.20.44.55',
	'07.19.2015.20.45.17',
	'07.19.2015.20.45.38',
	'07.19.2015.20.46.00',
	'07.19.2015.20.46.22',
	'07.19.2015.20.46.43',
	'07.19.2015.20.47.05',
	'07.19.2015.20.52.08',
	'07.19.2015.20.52.29',
	'07.19.2015.20.52.51',
	'07.19.2015.20.53.13',
	'07.19.2015.20.53.35',
	'07.19.2015.20.53.57',
	'07.19.2015.20.54.19',
	'07.19.2015.20.54.40',
	'07.19.2015.20.55.02',
}
--]]

--[[
start_idx = 1
--constant = 170
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/Match2/Attack_1/',
	'07.19.2015.04.03.11',
	'07.19.2015.04.03.33',
	'07.19.2015.04.03.55',
	'07.19.2015.04.04.17',
	'07.19.2015.04.04.38',
	'07.19.2015.04.05.00',
	'07.19.2015.04.05.22',
}
--]]

--[[
start_idx = 400
--constant = 170
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/Match1/Attack_1/',
	'07.19.2015.01.00.33',
	'07.19.2015.01.00.55',
	'07.19.2015.01.01.17',
	'07.19.2015.01.01.39',
	'07.19.2015.01.02.01',
	'07.19.2015.01.00.33',
	'07.19.2015.01.00.55',
	'07.19.2015.01.01.17',
	'07.19.2015.01.01.39',
	'07.19.2015.01.02.01',
	'07.19.2015.01.00.33',
	'07.19.2015.01.00.55',
	'07.19.2015.01.01.17',
	'07.19.2015.01.01.39',
	'07.19.2015.01.02.01',
	'07.19.2015.01.00.33',
	'07.19.2015.01.00.55',
	'07.19.2015.01.01.17',
	'07.19.2015.01.01.39',
	'07.19.2015.01.02.01',
	'07.19.2015.01.00.33',
	'07.19.2015.01.00.55',
	'07.19.2015.01.01.17',
	'07.19.2015.01.01.39',
	'07.19.2015.01.02.01',
	'07.19.2015.01.00.33',
	'07.19.2015.01.00.55',
	'07.19.2015.01.01.17',
	'07.19.2015.01.01.39',
	'07.19.2015.01.02.01',
}
--]]

--[[
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/Match1/Attack_5/',
	'07.19.2015.01.22.01',
	'07.19.2015.01.22.23',
	'07.19.2015.01.22.45',
}
--]]
--[[
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/day2_1pm',
	'07.18.2015.01.31.20',
	'07.18.2015.01.31.31',
	'07.18.2015.01.31.41',
	'07.18.2015.01.31.50',
	'07.18.2015.01.31.58',
	'07.18.2015.01.32.06',
	'07.18.2015.01.32.14',
	'07.18.2015.01.32.22',
	'07.18.2015.01.32.30',
	'07.18.2015.01.32.38',
	'07.18.2015.01.32.46',
	'07.18.2015.01.32.54',
	'07.18.2015.01.33.02',
	'07.18.2015.01.33.10',
	'07.18.2015.01.33.18',
	'07.18.2015.01.33.26',
	'07.18.2015.01.33.36',
	'07.18.2015.01.33.46',
	'07.18.2015.01.33.56',
	'07.18.2015.01.34.06',
	'07.18.2015.01.34.16',
	'07.18.2015.01.34.26',
	'07.18.2015.01.34.36',
	'07.18.2015.01.34.46',
	'07.18.2015.01.34.56',
	'07.18.2015.01.35.06',
	'07.18.2015.01.35.16',
	'07.18.2015.01.35.26',
	'07.18.2015.01.35.36',
	'07.18.2015.01.35.46',
	'07.18.2015.01.35.56',
	'07.18.2015.01.36.06',
	'07.18.2015.01.36.16',
	'07.18.2015.01.36.26',
	'07.18.2015.01.36.36',
	'07.18.2015.01.36.46',
	'07.18.2015.01.36.56',
	'07.18.2015.01.37.06',
	'07.18.2015.01.37.16',
	'07.18.2015.01.37.26',
	'07.18.2015.01.37.36',
	'07.18.2015.01.37.46',
	'07.18.2015.01.37.56',
	'07.18.2015.01.38.04',
	'07.18.2015.01.38.12',
	'07.18.2015.01.38.20',
	'07.18.2015.01.38.28',
	'07.18.2015.01.38.36',
	'07.18.2015.01.38.44',
	'07.18.2015.01.38.52',
	'07.18.2015.01.39.00',
	'07.18.2015.01.39.09',
	'07.18.2015.01.39.19',
	'07.18.2015.01.39.29',
	'07.18.2015.01.39.39',
	'07.18.2015.01.39.49',
	'07.18.2015.01.39.59',
	'07.18.2015.01.40.09',
	'07.18.2015.01.40.19',
	'07.18.2015.01.40.29',
	'07.18.2015.01.40.39',
	'07.18.2015.01.40.49',
	'07.18.2015.01.40.59',
	'07.18.2015.01.41.09',
	'07.18.2015.01.41.19',
	'07.18.2015.01.41.29',
	'07.18.2015.01.41.39',
	'07.18.2015.01.41.49',
	'07.18.2015.01.41.59',
	'07.18.2015.01.42.09',
	'07.18.2015.01.42.19',
	'07.18.2015.01.42.29',
	'07.18.2015.01.42.39',
	'07.18.2015.01.42.49',
	'07.18.2015.01.42.59',
	'07.18.2015.01.43.08',
	'07.18.2015.01.43.18',
	'07.18.2015.01.43.26',
	'07.18.2015.01.43.34',
	'07.18.2015.01.43.42',
	'07.18.2015.01.43.50',
	'07.18.2015.01.43.58',
	'07.18.2015.01.44.06',
	'07.18.2015.01.44.14',
	'07.18.2015.01.44.22',
	'07.18.2015.01.44.30',
	'07.18.2015.01.44.40',
	'07.18.2015.01.44.50',
	'07.18.2015.01.45.00',
	'07.18.2015.01.45.10',
	'07.18.2015.01.45.20',
	'07.18.2015.01.45.30',
	'07.18.2015.01.45.40',
	'07.18.2015.01.45.50',
	'07.18.2015.01.46.54',
	'07.18.2015.01.47.40',
	'07.18.2015.01.47.50',
	'07.18.2015.01.48.00',
	'07.18.2015.01.48.10',
	'07.18.2015.01.48.20',
	'07.18.2015.01.48.29',
	'07.18.2015.01.48.37',
	'07.18.2015.01.51.34',
	'07.18.2015.01.51.50',
	'07.18.2015.01.52.00',
	'07.18.2015.01.52.10',
	'07.18.2015.01.52.19',
	'07.18.2015.01.52.27',
	'07.18.2015.01.52.36',
	'07.18.2015.01.52.45',
	'07.18.2015.01.52.53',
	'07.18.2015.01.53.01',
	'07.18.2015.01.53.09',
	'07.18.2015.01.53.17',
	'07.18.2015.01.53.25',
	'07.18.2015.01.53.33',
	'07.18.2015.01.53.41',
	'07.18.2015.01.53.49',
	'07.18.2015.01.53.57',
	'07.18.2015.01.54.05',
	'07.18.2015.01.54.15',
	'07.18.2015.01.54.25',
	'07.18.2015.01.54.35',
	'07.18.2015.01.54.45',
	'07.18.2015.01.54.55',
	'07.18.2015.01.55.05',
	'07.18.2015.01.55.15',
	'07.18.2015.01.55.25',
	'07.18.2015.01.55.35',
	'07.18.2015.01.55.45',
	'07.18.2015.01.55.55',
	'07.18.2015.01.56.05',
	'07.18.2015.01.56.15',
	'07.18.2015.01.56.25',
	'07.18.2015.01.56.35',
	'07.18.2015.01.56.45',
}
--]]
--[[
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
--]]
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
	--'07.17.2015.21.00.40',
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
		local speedup = 2
		-- Temporary loop hack
		if start_idx and cnt < start_idx then

		elseif not constant then
			unix.usleep( dt * 1e6 / speedup )
			-- Set the localization in memory
			-- wcm.set_pose
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
