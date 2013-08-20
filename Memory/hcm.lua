---------------------------------------------------------------
-- Human Communication Module
-- (c) 2013 Stephen McGill
---------------------------------------------------------------
local vector = require'vector'
local memory = require'memory'
local nJoints = 35

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
memory.init_shm_segment(..., shared_data, shared_data_sz)
