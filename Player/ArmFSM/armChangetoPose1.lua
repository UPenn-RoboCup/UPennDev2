--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

local t_entry, t_update, t_finish
local timeout = 15.0

-- Goal position is arm Init, with hands in front, ready to manipulate

local qL_desired = {}
local qR_desired = {}


-- Angular velocity limit
local dqArmMax = Config.arm.slow_elbow_limit

local move_stage=1

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Where are the arms right now?
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  Body.set_larm_command_position(qLArm)
  Body.set_rarm_command_position(qRArm)

  move_stage = 1

  --Is it forward motion or downward motion?
  if qLArm[1]>45*Body.DEG_TO_RAD and qRArm[1]>45*Body.DEG_TO_RAD then
    --Both arm low. should be a forward motion    
    total_stage = 1
    qL_desired = {Config.arm.qLArmInit[1]}
    qR_desired = {Config.arm.qRArmInit[1]}    
  else
    --Arm stretched forward... should be a backward motion
    total_stage = 2
    qL_desired = {Config.arm.qLArmInit[2], Config.arm.qLArmInit[1]}
    qR_desired = {Config.arm.qRArmInit[2], Config.arm.qRArmInit[1]}
  end

end

function move_arms(qLTarget, qRTarget,dt)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local qL_approach, doneL
  qL_approach, doneL = util.approachTol( qLArm, qLTarget, dqArmMax, dt )
  Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR
  qR_approach, doneR = util.approachTol( qRArm, qRTarget, dqArmMax, dt )
  Body.set_rarm_command_position( qR_approach ) 
  
  return doneL and doneR
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if move_arms(qL_desired[move_stage],qR_desired[move_stage],dt) then 
    move_stage = move_stage+1;  
  end  
  if move_stage>total_stage then
    return "done";  
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
