dofile'../include.lua'
require'hcm'
--local f = io.popen'arecord -f S16_LE -c2 -D hw:3,0 -t raw | ./vumeter'
local f = io.popen'arecord -f S16_LE -c1 -D hw:2,0 -t raw | ./vumeter'
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
