local state = {}
state._NAME = 'armTeleop'
local Config = require'Config'
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
require'hcm'

-- Arm joints Angular velocity limits
local dqArmMax = Config.arm.slow_limit

local t_debug = Body.get_time()
local function update_joint(dt)

  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  -- Get the desired joints
  local qL_desired = hcm.get_joints_qlarm()
  local qR_desired = hcm.get_joints_qrarm()

  if t_update-t_debug>1 then
    t_debug = t_update
    print('Teleop | Desired joints')
    print(qL_desired)
    print(qR_desired)
  end

  -- Go to the allowable position
  local qL_approach, doneL
  qL_approach, doneL = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
  qL_approach = Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR
  qR_approach, doneR = util.approachTol( qRArm, qR_desired, dqArmMax, dt )
  qR_approach = Body.set_rarm_command_position( qR_approach )

  -- Set our hcm in case of a mode switch
  hcm.set_joints_plarm( K.l_arm_torso(qL_approach) )
  hcm.set_joints_prarm( K.r_arm_torso(qR_approach) )
end

local function update_ik(dt)

  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  -- Get the desired IK position
  local trLArm = hcm.get_joints_plarm()
  local trRArm = hcm.get_joints_prarm()
  if t_update-t_debug>1 then
    t_debug = t_update
    print('Teleop | Desired IK')
    print(trLArm)
    print(trRArm)
  end

  -- Check if this is possible
  local qL_desired = Body.get_inverse_larm(qLArm,trLArm)
  local qR_desired = Body.get_inverse_rarm(qRArm,trRArm)

  -- If not possible, set to where we are
  if not qL_desired then
    qL_desired = qLArm
    pL = K.l_arm_torso(qL_desired)
    hcm.set_joints_plarm(pL)
  end
  if not qR_desired then
    qR_desired = qRArm
    pR = K.r_arm_torso(qR_desired)
    hcm.set_joints_prarm(pR)
  end

  -- Go to the allowable position
  local qL_approach, doneL
  qL_approach, doneL = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
  qL_approach = Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR
  qR_approach, doneR = util.approachTol( qRArm, qR_desired, dqArmMax, dt )
  qR_approach = Body.set_rarm_command_position( qR_approach )

  -- Set our hcm in case of a mode switch
  hcm.set_joints_qlarm( qL_approach )
  hcm.set_joints_qrarm( qR_approach )

end

local update_mode = {
  [1] = update_joint,
  [2] = update_ik
}

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
  hcm.set_joints_plarm( K.l_arm_torso(qLArm) )
  hcm.set_joints_prarm( K.r_arm_torso(qRArm) )
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
  return update_mode[mode](dt)

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state