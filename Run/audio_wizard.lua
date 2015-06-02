-- arecord -f S16_LE -c2 -d 20 -D hw:3,0 -t raw | ./vumeter > test.tmp
-- arecord -f S16_LE -c2 -d 7 -D hw:1,0 -t raw | lame -r -s 16 -b 8 -h - /tmp/robot.mp3
dofile'../include.lua'
require'hcm'

hcm.set_audio_volume(percent)
hcm.set_audio_rawvolume(raw)

-- TODO: Reliably opening these, regardless of the dev number
local f2 = io.popen'arecord -f S16_LE -c1 -D hw:2,0 -t raw | ./vumeter'
f2:setvbuf'no'
local f3 = io.popen'arecord -f S16_LE -c2 -D hw:3,0 -t raw | ./vumeter'
f3:setvbuf'no'
--unix.select({f2, f3})å

local level
repeat
	level = ≈:read('*line')
	local raw, percent = level:match("(%d+) (%d+)")
	raw = tonumber(raw)
	percent = tonumber(percent)
	print('from vumeter', level)
	print('Volume', raw, percent)
	hcm.set_audio_volume(percent)
	hcm.set_audio_rawvolume(raw)
until not level
print('done reading!')

f2:close()
f3:close()
