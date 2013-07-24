---------------------------------------------------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
-- SJ: Re-wrote for easy generalization over multiple dcms
---------------------------------------------------------------------------
local vector = require'vector'
local util = require'util'

local nServos = 25

local shared_data = {}
local shared_data_sz = {}

shared_data.present = {}
shared_data.present.position = vector.zeros( nServos )
shared_data.present.velocity = vector.zeros( nServos )
shared_data.present.hand = vector.zeros( 3 )

shared_data.commanded = {}
shared_data.commanded.position = vector.zeros( nServos )
shared_data.commanded.velocity = vector.zeros( nServos )
shared_data.commanded.hand = vector.zeros( 3 )

-- Call the initializer
util.init_shm_segment(..., shared_data, shared_data_sz)