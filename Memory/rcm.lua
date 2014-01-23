--------------------------------
-- ROS Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local memory = require'memory'
local vector = require'vector'
local shared_data = {}
local shared_data_sz = {}

-- NOTE: Sensor and actuator names should not overlap!
-- This is because Body makes certain get/set assumptions

shared_data.datamatrix = {}
shared_data.datamatrix.translation = vector.zeros( 3 )
shared_data.datamatrix.rotation = vector.zeros(9)
shared_data.datamatrix.t = vector.zeros(1)

-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
