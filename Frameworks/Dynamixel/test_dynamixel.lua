local Dynamixel = require('libDynamixel');
local twait = 0.010;

local use_real_device = true;
local test_probe = false;

if use_real_device then
	fd = Dynamixel.open()
end

local val = 1024
for id=1,6 do
	local ret = Dynamixel.set_torque_enable(fd,id,val)
	io.write('ID ',id,' | Sent ',ret,' bytes\n')
end

if test_probe then
	Dynamixel.ping_probe(twait);
end