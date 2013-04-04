local cwd = os.getenv('PWD')
dofile('include.lua')
local simple_ipc = require 'simple_ipc'
local camera_channel = simple_ipc.setup_subscriber('camera')
print('load')
while true do
    local res = camera_channel:receive()
    print(res)
end
