#!/usr/local/bin/luajit
dofile'../../include.lua'
os.execute('clear')

local P = require'libArmPlan'
local K = require'K_ffi'
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
