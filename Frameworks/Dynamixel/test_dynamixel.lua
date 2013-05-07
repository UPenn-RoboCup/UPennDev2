local Dynamixel = require('libDynamixel');

local twait = 0.010;

--Dynamixel.open();
Dynamixel.ping_probe(twait);
