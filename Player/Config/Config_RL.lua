module(..., package.seeall);
require('vector')
require 'unix'
-- Stretcher Parameters
-- Sample Step size
rl = {}
rl.d_tStep = 0.01
rl.d_bodyHeight = 0.001;
rl.d_stepHeight = 0.002;
--[[
shared.params.footY = vector.zeros(1);
shared.params.supportX = vector.zeros(1);
shared.params.supportY = vector.zeros(1);
--]]
rl.d_vx = 0.002; -- Add in vy maybe for a diagonal walk...

-- Gradient step sizes
rl.g_tStep = 0.01
rl.g_bodyHeight = 0.001;
rl.g_stepHeight = 0.002;
--[[
shared.params.footY = vector.zeros(1);
shared.params.supportX = vector.zeros(1);
shared.params.supportY = vector.zeros(1);
--]]
rl.g_vx = 0.002; -- Add in vy maybe for a diagonal walk...

