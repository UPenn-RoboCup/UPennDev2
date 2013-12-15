dofile'include.lua'
local simple_ipc = require'simple_ipc'
local Config = require'Config'
local mp = require'msgpack'
require'hcm'
local audio_tcp_ch = simple_ipc.new_publisher(Config.net.audio,false,'*')

while true do

if(hcm.get_audio_request()==1) then
--  os.execute('sh get_audio.sh')
  os.execute('arecord -f S16_LE -c2 -d 10 -D hw:1,0 -t raw | lame -r - /tmp/robot.mp3')
--arecord -f S16_LE -c2 -d 10 -D hw:1,0 /tmp/robot.wav')
  local f = io.open('/tmp/robot.mp3')
  local audio = f:read('*all')
  local ret =audio_tcp_ch:send({mp.pack{t=0},audio})
print('ret',ret,#audio)
  -- Suppress
  hcm.set_audio_request(0)
end

end
