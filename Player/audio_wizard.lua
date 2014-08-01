dofile'../include.lua'
-- (c) 2013, 2014 Stephen McGill
require'hcm'
local mp = require'msgpack.MessagePack'
local simple_ipc = require'simple_ipc'
local audio_tcp_ch = simple_ipc.new_publisher(Config.net.audio,false,'*')

-- Listen forever
while true do

	-- Check the shared memory segment for a change
	if hcm.get_audio_request()==1 then
		-- Suppress the request
		hcm.set_audio_request(0)
		-- Get the time
		local t = unix.time()
		-- Start the audio record process
		os.execute'arecord -f S16_LE -c2 -d 7 -D hw:1,0 -t raw | lame -r -s 16 -b 8 -h - /tmp/robot.mp3'
		-- Open the file to snd over the net
		local f = io.open'/tmp/robot.mp3'
		local audio = f:read'*all'
		f:close()
		local ret = audio_tcp_ch:send{mp.pack{t=t},audio}
		print('Sent',#audio,'bytes',ret)
	end
	
	-- Sleep 10 ms
	unix.usleep(1e4)

end
