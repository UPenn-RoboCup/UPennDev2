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
local t_entry, t_update

-- Angular velocity limit
local dqArmMax = Config.arm.slow_elbow_limit
local total_stage,stage
local pLWrist, pRWrist, pLWristTarget, pRWristTarget, lShoulderTarget, rShoulderTarget

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Where are the arms right now?
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  if qLArm[4]<-15*Body.DEG_TO_RAD and qRArm[4]<-15*Body.DEG_TO_RAD then
    total_stage = 1
    print("REV")
    qL_desired = {Config.arm.qLArmInit[3]}
    qR_desired = {Config.arm.qRArmInit[3]}
  else
    --Forward motion  
    total_stage = 2
    qL_desired = {Config.arm.qLArmInit[2], Config.arm.qLArmInit[3]}
    qR_desired = {Config.arm.qRArmInit[2], Config.arm.qRArmInit[3]}
  end
  stage = 1  



--NEW IK based control
  pLWrist = Body.get_forward_lwrist(qLArm)
  pRWrist = Body.get_forward_lwrist(qLArm)  

--------------------------------------------
  pLWristTarget = {-.0,.35,-.05,0,0,0}
  pRWristTarget = {-.0,-.35,-.05,0,0,0}
  lShoulderYawTarget = -24*Body.DEG_TO_RAD
  rShoulderYawTarget = 24*Body.DEG_TO_RAD
--------------------------------------------

--Take 2
  pLWristTarget = {.05,.32,-.05,0,0,0}
  pRWristTarget = {.05,-.32,-.05,0,0,0}
  lShoulderYawTarget = -12*Body.DEG_TO_RAD
  rShoulderYawTarget = 12*Body.DEG_TO_RAD

  

end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )
  -- Get the time of update  

--[[
  if movearm.setArmJoints(qL_desired[stage],qR_desired[stage],
      dt, dqArmMax)==1 then
    stage = stage+1;  
  end  
  if stage>total_stage then
    return "done";  
  end
  --]]

--Wrist IK based movement
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  if stage==1 then   --Straighten wrist    
    ret = movearm.setArmJoints(
      {qLArm[1],qLArm[2],qLArm[3],qLArm[4],0,0,0},
      {qRArm[1],qRArm[2],qRArm[3],qRArm[4],0,0,0},
      dt)
    if ret==1 then stage = stage + 1 end
  elseif stage==2 then --Move arms to the sides        
    ret = movearm.setWristPosition(
      pLWristTarget, pRWristTarget,dt, lShoulderYawTarget,rShoulderYawTarget)
    if ret==1 then stage = stage + 1 end
  elseif stage==3 then  

    return"done";
--[[
    qL = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0,0,0,-45*Body.DEG_TO_RAD})
    qR = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0,0,0,45*Body.DEG_TO_RAD})
    if movearm.setArmJoints(qL, qR, dt, dqArmMax)==1 then
      return"done";
    end
--]]    
  end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

-- Add Epi-sate
state.epi = {}
-- Is this going going in forward to Ready, or reverse to Init?
state.epi.reverse = false

return state
