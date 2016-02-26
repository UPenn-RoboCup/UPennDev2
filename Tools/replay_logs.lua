#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local util = require'util'
local si = require'simple_ipc'

local LOG_DATE_CAMERA0 = {

-- Car
	'06.06.2015.13.29.07',
	'06.06.2015.13.35.49',

	-- Reset
'06.06.2015.13.46.27',

-- After reset
'06.06.2015.13.47.22',
'06.06.2015.13.48.21',
'06.06.2015.13.49.21',
'06.06.2015.13.50.21',
'06.06.2015.14.04.20',


}

local co_camera0 = coroutine.create(function()
	for j, date in ipairs(LOG_DATE_CAMERA0) do
		local replay_itty0 = libLog.open(HOME..'Tools/Matlab/logs2', date, 'camera0')
		local metadata = replay_itty0:unroll_meta()
		--print('Unlogging', #metadata, 'points from', LOG_DATE)
		local itty0_iter = replay_itty0:log_iter()

		for i, meta, payload in itty0_iter do
			coroutine.yield(meta, payload)
			--local id = (j-1)*100 + i
			--local fname = string.format('ittybitty0_%03d.jpeg', id)
			--ptable(meta)
			--print(name, #payload)
			--local f = io.open('ittybitty0/'..fname, 'w')
			--f:write(payload)
			--f:close()
		end
	end
end)

-- ffmpeg -start_number 0 -i ittybitty0_%03d.jpeg -vcodec mpeg4 ittybitty0.mp4
local LOG_DATE_ITTY0 = {
	'06.06.2015.13.27.21',
	'06.06.2015.13.54.41',
	'06.06.2015.13.59.11',
	'06.06.2015.14.05.13'
}


local co_itty0 = coroutine.create(function()
	for j, date in ipairs(LOG_DATE_ITTY0) do
		local replay_itty0 = libLog.open(HOME..'Tools/Matlab/logs2', date, 'ittybitty0')
		local metadata = replay_itty0:unroll_meta()
		--print('Unlogging', #metadata, 'points from', LOG_DATE)
		local itty0_iter = replay_itty0:log_iter()

		for i, meta, payload in itty0_iter do
			coroutine.yield(meta, payload)
			--local id = (j-1)*100 + i
			--local fname = string.format('ittybitty0_%03d.jpeg', id)
			--ptable(meta)
			--print(name, #payload)
			--local f = io.open('ittybitty0/'..fname, 'w')
			--f:write(payload)
			--f:close()
		end
	end
end)

local LOG_DATE_ITTY1 = {
	'06.06.2015.13.27.18'
}
--[[
for j, date in ipairs(LOG_DATE_ITTY1) do
	local replay_itty1 = libLog.open(HOME..'Tools/Matlab/logs2', date, 'ittybitty1')
	local metadata = replay_itty1:unroll_meta()
	print('Unlogging', #metadata, 'points from', LOG_DATE)
	local itty1_iter = replay_itty1:log_iter()

	for i, meta, payload in itty1_iter do
		local id = (j-1)*100 + i
		local fname = string.format('ittybitty1_%03d.jpeg', id)
		--ptable(meta)
		--print(name, #payload)
		local f = io.open('ittybitty1/'..fname, 'w')
		f:write(payload)
		f:close()
	end
end
--]]


local LOG_DATE_FB = {
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

local co_fb = coroutine.create(function()
	for j, date in ipairs(LOG_DATE_FB) do
		--print(j, date)
		local replay = libLog.open(HOME..'Tools/Matlab/logs2', date, 'feedback')
		local metadata = replay:unroll_meta()
		--print('Unlogging', #metadata, 'points from', LOG_DATE)
		local iter = replay:log_iter()
		local t0 = metadata[1].tlog
		for i, meta in iter do
			coroutine.yield(meta)
			--local id = (j-1)*100 + i
			--local fname = string.format('ittybitty1_%03d.jpeg', id)
			--print()
			--print(id)
			--ptable(meta)
			--unix.usleep((meta.tlog - t0)*1e6 / 10)
			--t0 = meta.tlog
			--fb_ch:send(mp.pack(meta))
			--local f = io.open('ittybitty1/'..fname, 'w')
			--f:write(payload)
			--f:close()
		end
	end
end)

local LOG_DATE_MESH0 = {
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

'06.06.2015.13.47.50',
'06.06.2015.13.49.30',
'06.06.2015.13.57.58',

}


local co_mesh0 = coroutine.create(function()
	for j, date in ipairs(LOG_DATE_MESH0) do
		--print(j, date)
		local replay = libLog.open(HOME..'Tools/Matlab/logs2', date, 'mesh0')
		--print('mesh2 replay', replay)
		--ptable(replay)
		local metadata = replay:unroll_meta2()
		--print('Unlogging', #metadata, 'points from', LOG_DATE)
		local iter = replay:log_iter()
		--local t0 = metadata[1].tlog
		for i, meta, payload in iter do
			coroutine.yield(meta, payload)
			--local id = (j-1)*100 + i
			--local fname = string.format('ittybitty1_%03d.jpeg', id)
			--print()
			--print(id, #payload)
			--ptable(meta)

			--if id>30 and id<75 then
			--unix.usleep((meta.tlog - (t0 or meta.tlog))*1e6)

			--if id>150 and id<225 then
				--unix.usleep((meta.tlog - (t0 or meta.tlog))*1e6)
				--local dt = meta.tlog - (t0 or meta.tlog)
			--	t0 = meta.tlog
		--		if dt>=1 then
	--				print(id, dt)
--					table.insert(avg, dt)
				--end
				--unix.usleep(1e6)
				--mesh_ch:send({mp.pack(meta), payload})
			--end

			--local f = io.open('ittybitty1/'..fname, 'w')
			--f:write(payload)
			--f:close()
		end
	end
end)

local itty0_ch = si.new_publisher('ittybitty0')
local fb_ch = si.new_publisher('feedback')
local camera0_ch = si.new_publisher('camera0')
local mesh0_ch = si.new_publisher('mesh0')

local ch = {
	--itty0_ch,
	fb_ch,
	camera0_ch,
	mesh0_ch
}
local names = {
	--'itty0',
	'fb',
	'camera0',
	'mesh0'
}
local coro = {
	--co_itty0,
	co_fb,
	co_camera0,
	co_mesh0
}

local t_next = {}
local data_next = {}
for i, co in ipairs(coro) do
	print(names[i])
	local ok, meta, payload = coroutine.resume(co)
	ptable(meta)
	t_next[i] = meta.tlog
	data_next[i] = {mp.pack(meta), payload}
	print()
end
local t_cursor = math.min(unpack(t_next))


local cnt = 0
local done
while not done do
	cnt = cnt + 1
	local t_n, i = util.min(t_next)
	local dt = (t_n - t_cursor)
	t_cursor = t_n
	if cnt > 10 then
		dt = math.min(dt, 10)
	else
		dt = math.min(dt, 1)
	end
	dt = dt / 2
	-- Send
	local data = data_next[i]

-- 1433624372: Near the valve
-- At the door: 1433623646.0488
-- Door swings open: 1433623732.4492
-- Begin walking through the door: 1433623770.537
-- Inside conditions: 1433623822.3194
-- Move arm to the valve: 1433624078.416
-- Aligned to the valve: 1433624212.348
-- Done with SJ's low level control to turn it: 1433624312.668
-- Back from the valve: 1433624372.5473
--print( t/1e3, (t-t0)/1e3, b, t/1e3 - 1433538627.495)

-- 1433623922 --1433623994.7933

  -- TODO: Loop the crossing threshold (Figure 23) @ 1433623813.5
  local loopy = false
  
	if i==0 then break end
                 
	if t_cursor >= 1433624070 then  
    repeat
  		print(i, 'Time:', t_n)
  		print(i, dt, names[i])
  		unix.usleep( dt * 1e6 )
  		ch[i]:send(data)
    until not loopy
	end
	-- Repopulate
	local ok, meta, payload = coroutine.resume(coro[i])
	if not ok then done = true end
	if meta then
		t_next[i] = meta.tlog
		data_next[i] = {mp.pack(meta), payload}
	else
		t_next[i] = math.huge
	end
end
