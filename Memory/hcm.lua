---------------------------------------------------------------------------
-- Human Communication Module
-- (c) 2013 Stephen McGill
-- with code from Mike Hopkins as a base
-- SJ: Re-wrote for easy generalization over multiple dcms
---------------------------------------------------------------------------
local vector = require'vector'
local util = require'util'

local shared_data = {}
local shared_data_sz = {}

-- LEAP motion sensor
shared_data.leap = {}
shared_data.leap.sphere = vector.zeros( 1 )
shared_data.leap.timestamp = vector.zeros( 1 )

-- Kinect skeleton sensor
shared_data.skeleton = {}
--shared_data_sz.skeleton = 
shared_data.skeleton.positions = vector.zeros( nServos )

-- Call the initializer
util.init_shm_segment(..., shared_data, shared_data_sz)