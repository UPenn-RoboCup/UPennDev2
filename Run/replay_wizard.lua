#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

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
local logs = {}
logs.yuyv = {
	'07.11.2015.09.51.10',
	'07.11.2015.09.51.20',
	'07.11.2015.09.51.30',
	'07.11.2015.09.51.40',
	'07.11.2015.09.51.49',
	'07.11.2015.09.51.57',
	ch = si.new_publisher('camera0')
}

-- Initialize the coroutines
for name, log in pairs(logs) do
	log.co = coroutine.create(function()
		for j, date in ipairs(log) do
			local log = libLog.open(HOME..'Data', date, name)
			local metadata = log:unroll_meta()
			local it = log:log_iter()
			for i, meta, payload in it do
				coroutine.yield(meta, payload)
			end
		end
	end)
end

-- Start the multiplexer
local t_next = {}
local data_next = {}
for name, log in pairs(logs) do
	print('Initializing', name)
	local ok, meta, payload = coroutine.resume(log.co)
	--require'util'.ptable(meta)
	t_next[name] = meta.t
	-- Strip off the logging stuff
	meta.tlog = nil
	meta.rsz = nil
	-- Save the next data to send
	data_next[name] = {mp.pack(meta), payload}
end
-- Begin sending
local t_cursor = pairmin(t_next)
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
		print(i, 'Time:', t_n)
		--print(i, dt, names[i])
		local speedup = 1
		unix.usleep( dt * 1e6 / speedup )
		logs[i].ch:send(data)
	end
	-- Repopulate
	local ok, meta, payload = coroutine.resume(logs[i].co)
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
