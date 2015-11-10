#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

local speedup = 1
--speedup = 2
--speedup = 0.5

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

logs.joint = {
	ch = si.new_publisher'feedback',
	dir = HOME..'Data'..'/',
	--dir = '/Volumes/SHARED/semifinal',
	'10.24.2014.14.16.48',
	--'06.04.2014.13.54.20'
}

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
	'07.21.2015.07.31.44',
	'07.21.2015.07.32.06',
	'07.21.2015.07.32.27',
	'07.21.2015.07.32.49',
	'07.21.2015.07.33.11',
	'07.21.2015.07.33.33',
	'07.21.2015.07.33.55',
	'07.21.2015.07.34.16',
	'07.21.2015.07.34.38',
	'07.21.2015.07.35.00',
	'07.21.2015.07.35.22',
	'07.21.2015.07.35.44',
	'07.21.2015.07.36.06',
	'07.21.2015.07.36.27',
	'07.21.2015.07.36.49',
	'07.21.2015.07.37.11',
	'07.21.2015.07.37.33',
	'07.21.2015.07.37.55',
	'07.21.2015.07.38.16',
	'07.21.2015.07.38.38',
	'07.21.2015.07.39.00',
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
		io.write(string.format('%d [%s] %6.2f s\n', cnt, i, t_n - t0))
		--print(i, dt, names[i])
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
