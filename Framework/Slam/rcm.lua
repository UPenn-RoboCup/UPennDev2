require('util');
require('vector');

---------------------------------------------------------------------------
-- Range Communication Module
---------------------------------------------------------------------------

rcm = {}
rcm.nReturns = 1081;

local shared_data = {}

shared_data.counter = vector.zeros(1);
shared_data.id = vector.zeros(1);
shared_data.timestamp = vector.zeros(1);

shared_data.startAngle = vector.zeros(1); -- radians
shared_data.stopAngle = vector.zeros(1);  -- radians
shared_data.angleStep = vector.zeros(1);  -- radians
shared_data.startTime = vector.zeros(1);  -- seconds
shared_data.stopTime = vector.zeros(1);   -- seconds

-- 4 bytes per range return (float is 4 bytes in 64bit)
shared_data.ranges = rcm.nReturns*4; 
--shared_data.intensities ;

shared_data.odom = vector.zeros(3);
shared_data.imu = vector.zeros(3);
shared_data.gyro = vector.zeros(3); -- Just to give the rate

-- Generated Map
--mapSize = 400;
--shared_data.map = {};
--shared_data.map.omap = mapSize*mapSize;

local shared_size = shared_data.ranges + 2^16;

util.init_shm_module(rcm, 'rcm', shared_data,shared_size)

return rcm