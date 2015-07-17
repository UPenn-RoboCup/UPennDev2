#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

local constant = 128
--local constant = 280
--local constant = 83


--local constant = 184

--local constant = 680
--local start_idx = 600
--local start_idx = 800
--local constant = 840
--local constant = 1280
--local constant = 1066
--local constant = 1176
--local start_idx = 1100
--local start_idx = 1750
--local constant = 1778
--local constant = 1820
--local constant = 1062
--local start_idx = 915

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
	dir = HOME..'Data'..'/day1a',
	'07.16.2015.23.54.56',
	'07.16.2015.23.55.18',
	'07.16.2015.23.55.40',
	'07.16.2015.23.56.02',
	'07.16.2015.23.56.25',
	'07.16.2015.23.56.47',
	'07.16.2015.23.57.09',
}
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
