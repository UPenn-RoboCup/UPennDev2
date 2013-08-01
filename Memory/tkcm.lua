--------------------------------
-- Telekinesis Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local memory = require'memory'
local shared_data = {}
local shared_data_sz = {}

-- NOTE: Sensor and actuator names should not overlap!
-- This is because Body makes certain get/set assumptions

shared_data.table = {}
-- Position is in world coordinates? relative?
shared_data.table.position = vector.zeros( 3 )
-- Orientation is a quaternion
-- TODO: Write a lua quaternion library
shared_data.table.orientation = vector.zeros( 4 )

shared_data.drill = {}
-- Position is in world coordinates? relative?
shared_data.drill.position = vector.zeros( 3 )
-- Orientation is a quaternion
-- TODO: Write a lua quaternion library
shared_data.drill.orientation = vector.zeros( 4 )

-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)