module(..., package.seeall);

local shm = require('shm');
local util = require('util');
local vector = require('vector');
local Config = require('Config');

-- shared properties
shared = {};
shsize = {};

shared.opponent = {}
shared.opponent.pose = vector.zeros(3);
shared.opponent.ready = vector.zeros(1);
shared.opponent.caught = vector.zeros(1)

shared.agent = {}
shared.agent.caught = vector.zeros(1);
shared.agent.ready = vector.zeros(1);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

