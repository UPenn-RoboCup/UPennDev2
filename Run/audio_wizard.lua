#!/usr/local/bin/luajit
-- arecord -f S16_LE -c2 -d 20 -D hw:3,0 -t raw | ./vumeter > test.tmp
-- arecord -f S16_LE -c2 -d 7 -D hw:1,0 -t raw | lame -r -s 16 -b 8 -h - /tmp/robot.mp3
dofile'../include.lua'
require'hcm'

local cmdstr = {
--[[
	'arecord -f S16_LE -r16000 -c2 -D hw:2,0 -t raw | ./vumeter',
	'arecord -f S16_LE -r16000 -c1 -D hw:3,0 -t raw | ./vumeter',
	--]]
	----[[
	'arecord -f S16_LE -r16000 -c1 -D hw:2,0 -t raw | ./vumeter',
	'arecord -f S16_LE -r16000 -c2 -D hw:3,0 -t raw | ./vumeter',
	--]]
}

print('arg[1]', arg[1])
local idx = tonumber(arg[1]) or 1
local cmd = cmdstr[idx]

-- TODO: Reliably opening these, regardless of the dev number
local f = assert(io.popen(cmd))
f:setvbuf'no'

local volume_ptr = hcm.audioPtr.volume
local rawvolume_ptr = hcm.audioPtr.rawvolume

local level
repeat
	level = f:read('*line')
	print('vumeter', level)
	local raw, percent = level:match("(%d+) (%d+)")
	raw = tonumber(raw)
	percent = tonumber(percent)
	if percent then
		print('Volume', percent, idx-1)
		volume_ptr[idx-1] = percent
	end
	if raw then
		print('Raw', raw, idx-1)
		--rawvolume_ptr[idx-1] = raw
	end
until not level
print('done reading!')

f:close()
