--------------------------------
-- Human Communication Module --
-- (c) 2013 Stephen McGill    --
--------------------------------
local init_shm_segment = require'memory'.init_shm_segment
local zeros = require'vector'.zeros
local ones = require'vector'.ones

local shared_data = {}
local shared_data_sz = {}

shared_data.teleop = {
  head = zeros(2)
}

shared_data.guidance={}
shared_data.guidance.color = 'CYAN'
shared_data.guidance.t = zeros(1)


-- Call the initializer
init_shm_segment(..., shared_data, shared_data_sz)
