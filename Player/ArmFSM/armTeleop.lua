local state = {}
state._NAME = 'armTeleop'
local Config = require'Config'
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local movearm = require'movearm'
require'hcm'

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry  = Body.get_time()
  t_update = t_entry
  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  trLArm = Body.get_forward_larm(qLArm);
  trRArm = Body.get_forward_rarm(qRArm);

  -- Set hcm to be here
  hcm.set_joints_plarm( trLArm )
  hcm.set_joints_prarm( trRArm )
  hcm.set_joints_qlarm( qLArm )
  hcm.set_joints_qrarm( qRArm )
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  -- Get the teleop mode
  local mode = hcm.get_joints_teleop()
  if mode==1 then
    local qLArmTarget = hcm.get_joints_qlarm()
    local qRArmTarget = hcm.get_joints_qrarm()
    movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)
    local qLArm = Body.get_larm_command_position()
    local qRArm = Body.get_rarm_command_position()
    trLArm = Body.get_forward_larm(qLArm);
    trRArm = Body.get_forward_rarm(qRArm);

    
    -- Set our hcm in case of a mode switch
    hcm.set_joints_plarm( Body.get_forward_larm(qLArm) )
    hcm.set_joints_prarm( Body.get_forward_rarm(qRArm) )
  else --IK based movement
    local trLArmTarget = hcm.get_joints_plarm()
    local trRArmTarget = hcm.get_joints_prarm()
    local lShoulderYaw = hcm.get_joints_qlshoulderyaw()
    local rShoulderYaw = hcm.get_joints_qrshoulderyaw()
    ret = movearm.setArmToPosition(
      trLArmTarget,
      trRArmTarget,
      dt,
      lShoulderYaw,rShoulderYaw
      )
    if ret==-1 then
      print("resetting")
    --TODO  

    end
    local qLArm = Body.get_larm_command_position()
    local qRArm = Body.get_rarm_command_position()
    hcm.get_joints_qlarm(qLArm)
    hcm.get_joints_qrarm(qRArm)

print("trLArm:",
  trLArmTarget[1],
  trLArmTarget[2],
  trLArmTarget[3],
  trLArmTarget[4]*Body.RAD_TO_DEG,
  trLArmTarget[5]*Body.RAD_TO_DEG,
  trLArmTarget[6]*Body.RAD_TO_DEG)

  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state