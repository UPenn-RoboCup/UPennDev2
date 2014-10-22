local movearm={}
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
local P = require'libPlan'
require'hcm'
local K = require'K_ffi'

local lShoulderYaw = -45*DEG_TO_RAD;
local rShoulderYaw = 45*DEG_TO_RAD;

local lPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	vector.slice(Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]))
local rPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]), 
	vector.slice(Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]))
rPlanner:set_chain(K.forward_r_arm, K.inverse_r_arm)

local dqLimit = DEG_TO_RAD / 3

	-- Body.get_inverse_lwrist:
	--[[
  return Kinematics.inverse_l_wrist(trL, qL, lShoulderYaw or qL[3],
      bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
	--]]

function movearm.goto_wrists(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
	  local qLArm = Body.get_larm_position()
	  local qLWrist = Body.get_inverse_lwrist(qLArm, unpack(lwrist, 1, 2))
	  local qLGoal = Body.get_inverse_arm_given_wrist(qLWrist, lwrist[3])
		lPathIter = lPlanner:joint_iter(qLGoal, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_position()
	  local qRWrist = Body.get_inverse_rwrist(qRArm, unpack(rwrist, 1, 2))
	  local qRGoal = Body.get_inverse_arm_given_wrist(qRWrist, rwrist[3])
		rPathIter = rPlanner:joint_iter(qRGoal, dqLimit)
	end
	return lPathIter, rPathIter
end

-- Take in 6D Transform
function movearm.goto_tr6(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		local qWaist = Body.get_waist_command_position()
		local q_lGoal = Body.Kinematics.inverse_l_arm_7(lwrist,qRArm,0,0,qWaist)
		lPathIter = lPlanner:joint_iter(q_lGoal, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		local qWaist = Body.get_waist_command_position()
		local q_rGoal = Body.Kinematics.inverse_r_arm_7(rwrist,qRArm,0,0,qWaist)
		rPathIter = rPlanner:joint_iter(q_rGoal, dqLimit)
	end
	return lPathIter, rPathIter
end

function movearm.goto_tr(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		lPathIter = lPlanner:line_iter(lwrist, qLArm, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		rPathIter = rPlanner:line_iter(rwrist, qRArm, dqLimit)
	end
	return lPathIter, rPathIter
end

-- Take in Angles
function movearm.goto_q(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
		lPathIter = lPlanner:joint_iter(lwrist, dqLimit)
	end
	if rwrist then
		rPathIter = rPlanner:joint_iter(rwrist, dqLimit)
	end
	return lPathIter, rPathIter
end

function movearm.setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  

  local dqVelLeft = mcm.get_arm_dqVelLeft()
  local dqVelRight = mcm.get_arm_dqVelRight()

  local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqVelLeft, dt )  
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqVelRight, dt )

  -- TODO: Dynamixel is STUPID so we should manually check for the direction
  for i=1,7 do
    local qL_increment = util.mod_angle(qL_approach[i]-qLArm[i])
    local qR_increment = util.mod_angle(qR_approach[i]-qRArm[i])
    qL_approach[i] = qLArm[i] + qL_increment
    qR_approach[i] = qRArm[i] + qR_increment
  end

  Body.set_larm_command_position( qL_approach )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then return 1 end
end

return movearm
