local state = {}
state._NAME = 'armTeleop'
local Config = require'Config'
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
require'hcm'

-- Arm joints Angular velocity limits
local dqArmMax = vector.new({30,30,30,45,60,60})*Body.DEG_TO_RAD

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry  = Body.get_time()
  t_update = t_entry
  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  -- Set hcm to be here
  hcm.set_joints_plarm(K.r_arm_torso(qLArm))
  hcm.set_joints_prarm(K.r_arm_torso(qRArm))
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

  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  -- Get the desired IK position
  local trLArm = hcm.get_joints_plarm()
  local trRArm = hcm.get_joints_prarm()
  print(trLArm,trRArm)
  local qL_desired = Body.get_inverse_larm(qLArm,trLArm)
  local qR_desired = Body.get_inverse_rarm(qRArm,trRArm)
  -- Go there
  if qL_desired then
    qL_desired = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
    if qL_desired~=true then Body.set_larm_command_position( qL_desired ) end
  end
  if qR_desired then
    qR_desired = util.approachTol( qRArm, qR_desired, dqArmMax, dt )
    if qR_desired~=true then Body.set_rarm_command_position( qR_desired ) end
  end

--Body.get_inverse_larm = function( qLArm, trL, pos_tol, ang_tol )
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state