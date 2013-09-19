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
local dpArmMax = Config.arm.linear_slow_limit

local t_debug = Body.get_time()

--SJ: we should keep initial transform of the end effector
--Otherwise it will drift when it hits singularity and stuck
local trLArm0,trRArm0 
local trLArm, trRArm

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

  trLArm = Body.get_forward_larm(qLArm);
  trRArm = Body.get_forward_rarm(qRArm);

  -- Set our hcm in case of a mode switch
  hcm.set_joints_plarm( Body.get_forward_larm(qL_approach) )
  hcm.set_joints_prarm( Body.get_forward_rarm(qR_approach) )
end

local function update_ik(dt)

  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  
  -- Get the desired IK position
  local trLArm_desired = hcm.get_joints_plarm()
  local trRArm_desired = hcm.get_joints_prarm()
  if t_update-t_debug>1 then
    t_debug = t_update
    print('Teleop | Desired IK')
    print(trLArm_desired)
    print(trRArm_desired)
  end

  --SJ: Added interpolation in cartesian space as well (unless movement will jerky)

  local trLArmApproach, doneL = util.approachTol(trLArm, trLArm_desired, dpArmMax, dt )
  local trRArmApproach, doneR = util.approachTol(trRArm, trRArm_desired, dpArmMax, dt )

  -- Check if this is possible
  local qL_desired = Body.get_inverse_larm(qLArm,trLArmApproach)
  local qR_desired = Body.get_inverse_rarm(qRArm,trRArmApproach)

  -- If not possible, set to where we are
  if not qL_desired then
    trLArmApproach = trLArm
    qL_desired = qLArm
    hcm.set_joints_plarm(trLArm)
  end
  if not qR_desired then
    trRArmApproach = trRArm
    qR_desired = qRArm    
    hcm.set_joints_prarm(trRArm)
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

  trLArm = trLArmApproach;
  trRArm = trRArmApproach;

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
  return update_mode[mode](dt)

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state