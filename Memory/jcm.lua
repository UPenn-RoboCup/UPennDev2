--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local vector = require'vector'
local util = require'util'
local nJoints = 25
local shared_data = {}
local shared_data_sz = {}

shared_data.sensor = {}
-- Position of joints is in radians from the zero position
shared_data.sensor.position = vector.zeros( nJoints )
-- Velocity of joints is in radians per second
shared_data.sensor.velocity = vector.zeros( nJoints )

shared_data.actuator = {}
-- Position of joints is in radians from the zero position
shared_data.actuator.command = vector.zeros( nJoints )
-- Velocity of joints is in radians per second
shared_data.actuator.velocity = vector.zeros( nJoints )

-- Call the initializer
util.init_shm_segment(..., shared_data, shared_data_sz)