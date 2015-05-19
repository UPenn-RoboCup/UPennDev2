#!/usr/local/bin/luajit
dofile'../../include.lua'
os.execute('clear')

local P = require'libArmPlan'
local K = require'K_ffi'
local T = require'Transform'
local vector = require'vector'

local lPlanner, rPlanner
do
	local minLArm = vector.slice(
		Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm])
	local maxLArm = vector.slice(
		Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm])
	local minRArm = vector.slice(
		Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm])
	local maxRArm = vector.slice(
		Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm])
	--
	lPlanner = P.new_planner(minLArm, maxLArm, radiansPerSecond):set_chain(
		K.forward_larm, K.inverse_larm, K.jacobian)
	rPlanner = P.new_planner(minRArm, maxRArm, radiansPerSecond):set_chain(
		K.forward_rarm, K.inverse_rarm, K.jacobian)
end

-- Velocity and angular velocity [m/sec | rad/sec]
--local qRArm = vector.new{10,0,0, -5, 90,5,-90} * DEG_TO_RAD
--local qRArm = vector.new{90,0,0, -90, 0,0,0} * DEG_TO_RAD
--local vwRTarget = vector.new{-0.10,0,0, 0,0,0}
local qRArm = vector.new{90,0,0, -90, 0,0,0} * DEG_TO_RAD
local vwRTarget = vector.new{0,-0.1,0, 0,0,0}
local dqRArm = rPlanner:get_delta_qarm(vwRTarget, qRArm)

-- Velocity and angular velocity [m/sec | rad/sec]
--local qLArm = vector.new{10,0,0, -5, 90,5,-90} * DEG_TO_RAD
--local qLArm = vector.new{90,0,0, -90, 0,0,0} * DEG_TO_RAD
--local vwTarget = vector.new{-0.10,0,0, 0,0,0}
local qLArm = vector.new{90,0,0, -90, 0,0,0} * DEG_TO_RAD
local vwLTarget = vector.new{0,0.1,0, 0,0,0}
local dqLArm = lPlanner:get_delta_qarm(vwLTarget, qLArm)

print('Right')
print(qRArm * RAD_TO_DEG)
print(vwRTarget)
print(dqRArm * RAD_TO_DEG)

print('Left')
print(qLArm * RAD_TO_DEG)
print(vwLTarget)
print(dqLArm * RAD_TO_DEG)

--[[
local qLArm0 = vector.new{90,0,0, -90, 0,0,0} * DEG_TO_RAD
local trLArmGoal = T.transform6D{0.05, 0.35, -0.25, 0.00, 0.00, 0.00}
lPlanner:jacobian_stack(trLArmGoal, qLArm0)
--]]

local qRArm0 = vector.new{90,0,0, -90, 0,0,0} * DEG_TO_RAD
local trRArmGoal = T.transform6D{0.05, -0.35, -0.25, 0.00, 0.00, 0.00}
local qStack, qRArmGoal, qRDist = rPlanner:jacobian_stack(trRArmGoal, qRArm0)
print('qRArmGoal', vector.new(qRArmGoal)*RAD_TO_DEG)
print('Guess', unpack{137.752, -22.7613, -5, -71.6028, -68.0006, -66.6442, 92.8623})
for i, q in ipairs(qStack) do
	print('Distance', q[1], q[2]*RAD_TO_DEG)
end
