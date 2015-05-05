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
local qRArm = vector.new{0,0,0, 0, 0,0,0} * DEG_TO_RAD
local vwTarget = vector.new{-0.10,0,0, 0,0,0}
local dqArm = rPlanner:get_delta_qarm(vwTarget, qRArm)

print(qRArm)
print(vwTarget)
print(dqArm)
