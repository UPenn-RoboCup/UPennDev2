local libDynamixel = require('libDynamixel');
local twait = 0.010;

local use_real_device = false;
local test_probe = false;

--[[
for k,v in pairs(libDynamixel) do
	print(k,v)
end
--]]

-- Fake device for testing?
local dev_name = 'fake'
if use_real_device then
	dev_name = 'nil'
end

local Dynamixel = libDynamixel.open(dev_name)

--[[
for k,v in pairs(Dynamixel) do
	print(k,v)
end
--]]

local val = 1024
for id=1,6 do
	local ret = Dynamixel:set_torque_enable(id,val)
	io.write('ID ',id,' | Sent ',ret,' bytes\n')
end

if test_probe then
	libDynamixel.ping_probe(Dynamixel.fd,twait);
end