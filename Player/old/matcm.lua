local Config = require("Config")
local vector = require("vector")
local util = require("util")
local shm = require("shm")

local shared = {}
local shsize = {}

shared.control = {}
shared.control.lut_updated = vector.zeros(1);
shared.control.key = vector.zeros(1);

util.init_shm_segment(..., shared, shsize);

