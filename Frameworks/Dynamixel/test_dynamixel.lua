local libDynamixel = require('libDynamixel');
local twait = 0.010;

local use_real_device = false
local test_probe = false
local test_torque = true

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

if test_torque then
	print('Testing torque enable with MX motors')
local val = 0
for id=7,9 do
	io.write(string.format('ID %d\n', id) )
	local ret = Dynamixel:set_torque_enable(id,val)
	io.write( string.format('Sent %d bytes.\n',ret) )
	print()
end
end

if test_probe then
	libDynamixel.ping_probe(Dynamixel.fd,twait);
end