#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local ptable = require'util'.ptable
local si = require'simple_ipc'

-- ffmpeg -start_number 0 -i ittybitty0_%03d.jpeg -vcodec mpeg4 ittybitty0.mp4
--[[
local LOG_DATE = {
	'06.06.2015.13.27.21',
	'06.06.2015.13.54.41',
	'06.06.2015.13.59.11',
	'06.06.2015.14.05.13'
}
for j, date in ipairs(LOG_DATE) do
	local replay_itty0 = libLog.open(HOME..'Tools/Matlab/logs2', date, 'ittybitty0')
	local metadata = replay_itty0:unroll_meta()
	print('Unlogging', #metadata, 'points from', LOG_DATE)
	local itty0_iter = replay_itty0:log_iter()

	for i, meta, payload in itty0_iter do
		local id = (j-1)*100 + i
		local fname = string.format('ittybitty0_%03d.jpeg', id)
		--ptable(meta)
		--print(name, #payload)
		local f = io.open('ittybitty0/'..fname, 'w')
		f:write(payload)
		f:close()
	end
end

local LOG_DATE = {
	'06.06.2015.13.27.18'
}
for j, date in ipairs(LOG_DATE) do
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

--[[
local LOG_DATE = {
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
local fb_ch = si.new_publisher('feedback')
for j, date in ipairs(LOG_DATE) do
	print(j, date)
	local replay = libLog.open(HOME..'Tools/Matlab/logs2', date, 'feedback')
	local metadata = replay:unroll_meta()
	print('Unlogging', #metadata, 'points from', LOG_DATE)
	local iter = replay:log_iter()
	local t0 = metadata[1].tlog
	for i, meta in iter do
		local id = (j-1)*100 + i
		--local fname = string.format('ittybitty1_%03d.jpeg', id)
		print()
		print(id)
		ptable(meta)

		unix.usleep((meta.tlog - t0)*1e6 / 10)
		t0 = meta.tlog
		fb_ch:send(mp.pack(meta))
		--local f = io.open('ittybitty1/'..fname, 'w')
		--f:write(payload)
		--f:close()
	end
end
--]]

local LOG_DATE = {
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
local mesh_ch = si.new_publisher('mesh0')
local avg = {}
for j, date in ipairs(LOG_DATE) do
	print(j, date)
	local replay = libLog.open(HOME..'Tools/Matlab/logs2', date, 'mesh0')
	local metadata = replay:unroll_meta2()
	--print('Unlogging', #metadata, 'points from', LOG_DATE)
	local iter = replay:log_iter()
	--local t0 = metadata[1].tlog
	for i, meta, payload in iter do
		local id = (j-1)*100 + i
		--local fname = string.format('ittybitty1_%03d.jpeg', id)
		--print()
		--print(id, #payload)
		--ptable(meta)

		--if id>30 and id<75 then
		--unix.usleep((meta.tlog - (t0 or meta.tlog))*1e6)

		if id>150 and id<225 then
			--unix.usleep((meta.tlog - (t0 or meta.tlog))*1e6)
			local dt = meta.tlog - (t0 or meta.tlog)
			t0 = meta.tlog
			if dt>=1 then
				print(id, dt)
				table.insert(avg, dt)
			end
			--unix.usleep(1e6)
			--mesh_ch:send({mp.pack(meta), payload})


		end

		--local f = io.open('ittybitty1/'..fname, 'w')
		--f:write(payload)
		--f:close()
	end
end
local sum = 0
for ii,vv in ipairs(avg) do sum = sum + vv end
print(sum / #avg)
