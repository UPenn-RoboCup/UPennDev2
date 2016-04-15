#!/usr/local/bin/luajit

dofile'fiddle.lua'

local ok, mattorch = pcall(require, 'mattorch')
assert(ok, "Loading mattorch failed")

local P = require'libArmPlan'
local K = require'K_ffi'

local util = require'util'
--local vector = require'vector'

-- Read in the data from a MATLAB file
local plan = mattorch.load("~/plan0.mat")

print('Plan:');util.ptable(plan)

local lPlanner, rPlanner
local radiansPerSecond, torso0
do
  --
	local degreesPerSecond = vector.ones(7) * 15
	radiansPerSecond = degreesPerSecond * DEG_TO_RAD
	torso0 = {-Config.walk.torsoX, 0, 0}
  --
	local minLArm = vector.slice(
		Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm])
	local maxLArm = vector.slice(
		Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm])
	local minRArm = vector.slice(
		Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm])
	local maxRArm = vector.slice(
		Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm])
	-- Set up the planners for each arm
	print('Setting up planners')
	lPlanner = P.new_planner('Left')
		:set_chain(K.forward_larm, K.inverse_larm, K.jacobian_larm)
		:set_limits(minLArm, maxLArm, radiansPerSecond)
		:set_update_rate(10)
		:set_shoulder_granularity(2*DEG_TO_RAD)
	rPlanner = P.new_planner('Right')
		:set_chain(K.forward_rarm, K.inverse_rarm, K.jacobian_rarm)
		:set_limits(minRArm, maxRArm, radiansPerSecond)
		:set_update_rate(10)
		:set_shoulder_granularity(2*DEG_TO_RAD)
end

--[[
local qOptimized = {}
for i=1,#plan.qwPath do
  -- MATLAB and torch are transposed...
	qOptimized[i] = vector.new(plan:t()[i])
end
--]]

--lPlanner:jacobians(plan)