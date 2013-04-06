dofile('include.lua')
local simple_ipc = require 'simple_ipc'
local camera_channel = simple_ipc.new_subscriber('camera')
while true do
    local res = camera_channel:receive()
    print(#res)
end
