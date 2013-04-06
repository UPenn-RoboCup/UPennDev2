dofile('include.lua')
local simple_ipc = require 'simple_ipc'
local msgpack = require 'msgpack'
local camera_channel = simple_ipc.new_subscriber('imu')
local actuator_channel = simple_ipc.new_subscriber('actuator')
while true do
    local res = camera_channel:receive()
    tbl = msgpack.unpack(res)
    print(tbl[1], tbl[2], tbl[3], tbl[4], tbl[5], tbl[6])
    local res = actuator_channel:receive()
    act = msgpack.unpack(res)
    print(#act)
end
