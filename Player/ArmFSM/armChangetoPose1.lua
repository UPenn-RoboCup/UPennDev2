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

-- Goal position is arm Init, with hands in front, ready to manipulate

local qL_desired = {}
local qR_desired = {}


-- Angular velocity limit
local dqArmMax = Config.arm.slow_elbow_limit

local move_stage=1


--IK based home position
local pLWristTarget = Config.arm.pLWristTarget1
local pRWristTarget = Config.arm.pRWristTarget1
local lShoulderYawTarget = Config.arm.lShoulderYawTarget1
local rShoulderYawTarget = Config.arm.rShoulderYawTarget1

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  stage = 1
  Body.set_lgrip_percent(.7)
  Body.set_rgrip_percent(.7)
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )
  -- Get the time of update  

--Wrist IK based movement
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  if stage==1 then   --Straighten wrist    
    ret = movearm.setArmJoints(
      {qLArm[1],qLArm[2],qLArm[3],qLArm[4],qLArm[5],0,qLArm[7]},
      {qRArm[1],qRArm[2],qRArm[3],qRArm[4],qRArm[5],0,qRArm[7]},
      dt,Config.arm.joint_init_limit)
    if ret==1 then stage = stage + 1 end
  elseif stage==2 then
    ret = movearm.setArmJoints(
      {qLArm[1],qLArm[2],qLArm[3],qLArm[4],
          81*Body.DEG_TO_RAD,15*Body.DEG_TO_RAD,9*Body.DEG_TO_RAD},
      {qRArm[1],qRArm[2],qRArm[3],qRArm[4],
          -81*Body.DEG_TO_RAD,-15*Body.DEG_TO_RAD,9*Body.DEG_TO_RAD},
      dt,Config.arm.joint_init_limit)
    if ret==1 then stage = stage + 1 end
  elseif stage==3 then --Move arms to the sides            
    ret = movearm.setWristPosition(
      pLWristTarget, pRWristTarget,dt, lShoulderYawTarget,rShoulderYawTarget)
    if ret==1 then return"done";
    end
  end

end

function state.exit()

  local qLArm = Body.get_larm_command_position()
  trWrist = Body.get_forward_lwrist(qLArm)
  print("LWrist pos:",unpack(trWrist))
  Body.set_lgrip_percent(0.9)
  Body.set_rgrip_percent(0.9)

  print(state._NAME..' Exit' )
end

-- Add Epi-sate
state.epi = {}
-- Is this going going in forward to Ready, or reverse to Init?
state.epi.reverse = false

return state
