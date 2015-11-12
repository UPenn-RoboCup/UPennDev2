#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'
local constant
local start_idx
local speedup = 1
--speedup = 2
--speedup = 0.5

-- In lab Saturday
local logs = {}
--[[
logs.feedback = {
	ch = si.new_publisher'feedback',
	dir = HOME..'Data'..'/trial2/',
	-- Car
	--'06.06.2015.13.25.18',
	--'06.06.2015.13.26.58',
	--'06.06.2015.13.28.39',
	--'06.06.2015.13.30.19',
	--'06.06.2015.13.31.59',
	--'06.06.2015.13.33.39',
	--'06.06.2015.13.35.19',
	--'06.06.2015.13.37.00',
	--'06.06.2015.13.38.40',
	--'06.06.2015.13.40.20',
	--'06.06.2015.13.42.00',
	--'06.06.2015.13.43.40',
	--'06.06.2015.13.45.20',
	'06.06.2015.13.47.01',
	'06.06.2015.13.48.41',
	'06.06.2015.13.50.21',
	'06.06.2015.13.54.52',
	'06.06.2015.13.59.29',
	'06.06.2015.14.01.09',
}
logs.ittybitty0 = {
	ch = si.new_publisher'ittybitty0',
	dir = HOME..'Data'..'/trial2/',
	'06.06.2015.13.27.21',
	'06.06.2015.13.54.41',
	'06.06.2015.13.59.11',
	'06.06.2015.14.05.13'
}
logs.mesh0 = {
	ch = si.new_publisher'mesh0',
	raw = true,
	dir = HOME..'Data'..'/trial2/',
	--'06.06.2015.13.25.18',
	--'06.06.2015.13.25.19',
	--'06.06.2015.13.26.59',
	--'06.06.2015.13.28.42',
	--'06.06.2015.13.30.27',
	--'06.06.2015.13.32.15',
	--'06.06.2015.13.34.01',
	--'06.06.2015.13.35.41',
	--'06.06.2015.13.37.31',
	--'06.06.2015.13.39.15',
	--'06.06.2015.13.41.02',
	--'06.06.2015.13.42.43',
	--'06.06.2015.13.44.27',
	--'06.06.2015.13.46.09',
	-- Post reset
	'06.06.2015.13.47.50',
	'06.06.2015.13.49.30',
	'06.06.2015.13.57.58',
}
logs.camera0 = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/trial2/',
	raw = true,
	-- Car
	--'06.06.2015.13.29.07',
	--'06.06.2015.13.35.49',
	-- Reset
	--'06.06.2015.13.46.27',
	-- After reset
	'06.06.2015.13.47.22',
	'06.06.2015.13.48.21',
	'06.06.2015.13.49.21',
	'06.06.2015.13.50.21',
	'06.06.2015.14.04.20',
}
--]]
logs.k2_rgb = {
	-- Both channels work :D
	--ch = si.new_publisher'camera0',
	ch = si.new_publisher'kinect2_color',
	dir = HOME..'Data'..'/iros/',
	raw = true,
	'12.31.2008.19.39.19',
	'12.31.2008.19.42.23',
	'12.31.2008.19.48.32',
}
logs.k2_depth = {
	ch = si.new_publisher'kinect2_depth',
	dir = HOME..'Data'..'/iros/',
	raw = true,
	'12.31.2008.19.39.19',
	'12.31.2008.19.42.23',
	'12.31.2008.19.48.32',
}



-- RoboCup
--[[
--local constant = 4590 -- weird frame for lines
--local start_idx = 4500
require'wcm'
wcm.set_goal_disable({0})
wcm.set_robot_reset_pose({0})
require'gcm'
gcm.set_game_state({3})
--]]

--[[
--start_idx = 201
--constant = 275
logs.yuyv = {
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/semifinal/',
	--dir = '/Volumes/SHARED/semifinal',
	'07.21.2015.07.29.33',
	'07.21.2015.07.29.55',
	'07.21.2015.07.30.17',
	'07.21.2015.07.30.38',
	'07.21.2015.07.31.00',
	'07.21.2015.07.31.22',
}
--]]
--[[
logs.joint = {
	ch = si.new_publisher'feedback',
	dir = HOME..'Data'..'/',
	'10.24.2014.14.16.48',
}
--]]

-- Initialize the coroutines
for name, log in pairs(logs) do
	log.co = coroutine.create(function()
		local counter = 0
		for j, date in ipairs(log) do
			io.write(string.format('Opening %s [%s] ... ', name,date))
			local log = libLog.open(log.dir, date, name)
			local metadata
			if name:find'mesh' then
				metadata = log:unroll_meta2()
			else
				metadata = log:unroll_meta()
			end
			io.write('Done!\n')
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
	local ok, counter, meta, payload = coroutine.resume(log.co)
	t_next[name] = meta.t or meta.tlog
	-- Strip off the logging stuff
	meta.tlog = nil
	meta.rsz = nil
	-- Save the next data to send
	data_next[name] = {mp.pack(meta), payload}
end

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

-- Begin sending
print('Beginning...')
local t_cursor = pairmin(t_next)
local t0 = t_cursor
local cnt = 0
while true do
	cnt = cnt + 1
	local t_n, i = pairmin(t_next)
	if not i then break end
	local dt = t_n - t_cursor
	if cnt > 10 then
		dt = math.min(dt, 10)
	else
		dt = math.min(dt, 1)
	end
	t_cursor = t_n
	-- Send
	local data = data_next[i]
	-- Custom start time...
	if t_cursor >= 0 then
		io.write(string.format('%d [%s] %6.2f s\n', cnt, i, t_n - t0))
		--print(i, dt, names[i])
		-- Temporary loop hack
		if start_idx and cnt < start_idx then

		elseif not constant then
			unix.usleep( dt * 1e6 / speedup )
			-- Set the localization in memory
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
		t_next[i] = meta.t or meta.tlog
		meta.tlog = nil
		meta.rsz = nil
		data_next[i] = {mp.pack(meta), payload}
	else
		t_next[i] = math.huge
	end
end
print('Done!')
