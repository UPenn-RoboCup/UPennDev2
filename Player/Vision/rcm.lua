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
shared.lidar.ranges = nReturns*2; -- 2 bytes per return
shared.lidar.timestamp = vector.zeros(1);

print('Init shm for ',_NAME)

util.init_shm_segment(getfenv(), 'rcm', shared, shsize);