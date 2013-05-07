local Dynamixel = require('libDynamixel');

local twait = 0.010;

--Dynamixel.open();
local val = 1024
for id=1,6 do
	Dynamixel.set_torque_enable(fd,id,val)
end
--Dynamixel.ping_probe(twait);