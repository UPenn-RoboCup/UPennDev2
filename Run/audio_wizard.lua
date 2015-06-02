#!/usr/local/bin/luajit
-- arecord -f S16_LE -c2 -d 20 -D hw:3,0 -t raw | ./vumeter > test.tmp
-- arecord -f S16_LE -c2 -d 7 -D hw:1,0 -t raw | lame -r -s 16 -b 8 -h - /tmp/robot.mp3
dofile'../include.lua'
require'hcm'

local cmdstr = {
	'arecord -f S16_LE -c1 -D hw:2,0 -t raw | ./vumeter',
	'arecord -f S16_LE -c2 -D hw:3,0 -t raw | ./vumeter',
	-- flip
	'arecord -f S16_LE -c2 -D hw:2,0 -t raw | ./vumeter',
	'arecord -f S16_LE -c1 -D hw:3,0 -t raw | ./vumeter'
}

hcm.set_audio_volume(percent)
hcm.set_audio_rawvolume(raw)

-- TODO: Reliably opening these, regardless of the dev number
local f = assert(io.popen(cmdstr[arg[1]]or cmdstr[1]))
f:setvbuf'no'

local level
repeat
	level = f:read('*line')
	local raw, percent = level:match("(%d+) (%d+)")
	raw = tonumber(raw)
	percent = tonumber(percent)
	print('from vumeter', level)
	print('Volume', raw, percent)
	hcm.set_audio_volume(percent)
	hcm.set_audio_rawvolume(raw)
until not level
print('done reading!')

f:close()
