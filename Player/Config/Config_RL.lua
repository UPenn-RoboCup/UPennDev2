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

optimal = {}
--trialnum,reward,bodyHeight,vx,stepHeight,tStep
-- 78,1.242180,0.304093,0.082094,0.092785,1.641774
optimal.bodyHeight = 0.304093;
optimal.stepHeight = 0.092785;
optimal.vx = 0.082094;
optimal.tStep = 1.641774;

--[[
--trialnum,reward,bodyHeight,vx,stepHeight,tStep
-- 59,1.066126,0.304070,0.068114,0.084572,1.375899
optimal.bodyHeight = 0.304093;
optimal.stepHeight = 0.084572;
optimal.vx = 0.068114;
optimal.tStep = 1.375899
--]]
