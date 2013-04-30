local hokuyo = require('hokuyo')

local serial = "00805676"
--local serial = "00907258"
local device = "/dev/ttyACM1"
hokuyo.open( device, serial);

hokuyo.update();
local lidar = Hokuyo.retrieve();

hokuyo.shutdown()
for k,v in pairs(lidar) do
	print( k, type(v) )
end
