dofile'include.lua'
local simple_ipc = require'simple_ipc'
local Config = require'Config'
local mp = require'msgpack'
require'hcm'
local audio_tcp_ch = simple_ipc.new_publisher(Config.net.audio,false,'*')

while true do

if(hcm.get_audio_request()==1) then
  --os.execute'arecord -f S16_LE -c2 -d 10 -D hw:1,0 -t raw | lame -r -s 16 -b 8 -h - /tmp/robot.mp3'
  os.execute'arecord -f S16_LE -c2 -d 7 -D hw:1,0 -t raw | lame -r -s 16 -b 8 -h - /tmp/robot.mp3'
  local f = io.open('/tmp/robot.mp3')
  local audio = f:read('*all')
  local ret =audio_tcp_ch:send({mp.pack{t=0},audio})
  print('Send',#audio,'bytes')
  -- Suppress
  hcm.set_audio_request(0)
end

end
