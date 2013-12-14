dofile'include.lua'
local simple_ipc = require'simple_ipc'

local rpc_zmq = simple_ipc.new_replier(Config.net.reliable_audio,'*')
rpc_zmq.callback = function()
print'hi'
  local request, has_more
    request, has_more = rpc_zmq:receive()
    local rpc         = mp.unpack(request)
    os.execute('sh get_audio.sh')
    local f = io.open('/tmp/robot.mp3')
    local audio = f:read('*all')
    local ret         = rpc_zmq:send(audio)
end
local wait_channels = {rpc_zmq}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

print('RPC | Receiving on',Config.net.reliable_audio)
channel_poll:start()
