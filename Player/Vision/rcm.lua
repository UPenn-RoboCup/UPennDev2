module(..., package.seeall);

require('shm');
require('util');
require('vector');
require 'Config'

-- shared properties
shared = {};
shsize = {};

nReturns = 1080;
shared.lidar = {};
shared.lidar.ranges = nReturns*4; -- 4 bytes per return (float is 4 bytes in 64bit)
shared.lidar.timestamp = vector.zeros(1);

print('Init shm for ',_NAME)

util.init_shm_segment(getfenv(), 'rcm', shared, shsize);