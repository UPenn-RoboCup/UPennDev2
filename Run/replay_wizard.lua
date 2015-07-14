#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

--local constant = 458

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
	'07.13.2015.17.03.14',
	'07.13.2015.17.03.26',
	'07.13.2015.17.03.38',
	'07.13.2015.17.03.50',
	'07.13.2015.17.04.02',
	'07.13.2015.17.04.14',
	ch = si.new_publisher'camera0',
	dir = HOME..'Data'..'/ucla1',
}

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
		repeat
			unix.usleep( dt * 1e6 / speedup )
			logs[i].ch:send(data)
		until constant~=cnt
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
