--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 15.0
local arm_planner = require 'libArmPlan'.new_planner()

-- Goal position is arm Init, with hands in front, ready to manipulate

local qLArmTarget, qRArmTarget, stage

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t
 

  local vel_limit_arm = vector.new(
    {0.02,0.02,0.02, 15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})
  local ang_limit_arm = vector.new(
    {20,20,20,20,90,90,90})*DEG_TO_RAD

  mcm.set_arm_dpVelLeft(vel_limit_arm*10)
  mcm.set_arm_dpVelRight(vel_limit_arm*10)
  mcm.set_arm_dqVelLeft(ang_limit_arm*2)
  mcm.set_arm_dqVelRight(ang_limit_arm*2)
  

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])


  local trLArm1 = util.shallow_copy(trLArm0)
  local trRArm1 = util.shallow_copy(trRArm0)

  trLArm1[1] = trLArm1[1] + 0.15
  trRArm1[1] = trRArm1[1] + 0.15

  stage="init"

  local arm_seq = {
    {'move',trLArm1,trRArm1},
    {'move',trLArm0,trRArm0},
    {'move',trLArm1,trRArm1},
    {'move',trLArm0,trRArm0},
  }
  arm_planner:plan_arm_sequence(arm_seq) 
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if stage=="init" then --Turn wrist angles without moving arms
    if arm_planner:play_arm_sequence(t) then 
      return "done"
    end
  end



end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
