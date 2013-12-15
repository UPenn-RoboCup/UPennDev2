dofile'include.lua'
local simple_ipc = require'simple_ipc'
require'hcm'
local audio_tcp_ch = simple_ipc.new_publisher(Config.net.reliable_audio,false,'*')

while true do

if(hcm.get_audio_request()==1) then
  os.execute('sh get_audio.sh')
  local f = io.open('/tmp/robot.mp3')
  local audio = f:read('*all')
  local ret =audio_tcp_ch:send(audio)
print('ret',ret,#audio)
  -- Suppress
  hcm.set_audio_request(0)
end

end
