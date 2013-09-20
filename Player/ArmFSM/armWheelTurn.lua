local state = {}
state._NAME = ...
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'hcm'

-- Arm joints Angular velocity limits
--local dqArmMax = Config.arm.slow_limit
local dqArmMax = Config.arm.fast_limit
-- Turning speed
local dturnAngleMax = 3*math.pi/180 -- 3 deg per sec
local turnAngleMax = 60*math.pi/180  -- 60 deg max

local turnAngle = 0
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

local turnAngleCurrent
local handle_pos,handle_pitch,handle_yaw
local handle_radius1,handle_radius0
local function calculate_arm_position(turnAngle)
   local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)
       * T.rotY(handle_pitch)

   local trGripL = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,handle_radius1,0)
       * T.rotZ(-math.pi/4)
   local trGripR = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,-handle_radius1,0)
       * T.rotZ(math.pi/4)
       
   local trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
       * T.rotY(body_rpy[2])
       
   local trLArm = T.position6D(T.inv(trBody)*trGripL)
   local trRArm = T.position6D(T.inv(trBody)*trGripR)
   return trLArm, trRArm
end

local function calculate_turn_angle(qL,qR)
  local pL = vector.slice(pLArm,1,3)
  local pR = vector.slice(pRArm,1,3)
  local dp = pL-pR
  local center = (pL+pR)/2
  print('handle_pitch',handle_pitch)
  print('handle_yaw',handle_yaw)
  print('Current turn angle:',dp)
  return 0
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry  = Body.get_time()
  t_update = t_entry 
  
-- Let's store wheel data here
  local wheel   = hcm.get_wheel_model()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  local handle_radius = wheel[6]
  -- Inner and outer radius
  handle_radius0 = handle_radius - 0.02
  handle_radius1 = handle_radius + 0.02

  -- Always enter this state
  -- with a turn angle of zero
  -- TODO: Use shm to fix this...
  turnAngleCurrent = 0
  --[[
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  calculate_turn_angle(qLArm,qRArm)
  --]]
     
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local  t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  -- Get the current joint positions (via commands)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  -- Get the current turning angle
  --local turnAngleCurrent = calculate_turn_angle(qLArm,qRArm)
  -- Get the target turning angle
  local turnAngleTarget = hcm.get_wheel_turnangle()

  ----------------------------------------------------
  --SJ: update wheel position online (for testing)
  local wheel   = hcm.get_wheel_model()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  handle_radius1 = handle_radius + 0.02
  ----------------------------------------------------








  -- Find out how much more we need to turn
  local turnAngleDiff = turnAngleTarget-turnAngleCurrent
  -- Do not allow too much movement
  turnAngleDiff = util.procFunc(turnAngleDiff,0,dturnAngleMax*dt)
  -- Find the new commanded turning angle
  local turnAngleCommand = turnAngleCurrent + turnAngleDiff
  -- Cannot steer to too extreme values
  turnAngleCommand = util.procFunc(turnAngleCommand,0,turnAngleMax)
  
  -- Find our current arm position
  --local pLArm = K.l_arm_torso(qLArm)
  --local pRArm = K.r_arm_torso(qRArm)
  -- Find our desired arm position
  local trLArm, trRArm = calculate_arm_position(turnAngleCommand)
  -- Get desired angles from current angles and target transform

  local shoulderAngle   = hcm.get_joints_shoulderangle()

  local qL_desired = Body.get_inverse_larm(qLArm,trLArm,shoulderAngle)
  local qR_desired = Body.get_inverse_rarm(qLArm,trRArm,-shoulderAngle)


  if not qL_desired then
    print("Left arm stuck")
  elseif not qR_desired then
    print("Right arm stuck")
    --We hit the IK limit. do nothing

  else
    --[[
    print("LWrist:",qLArm[5]*180/math.pi,
      qLArm[6]*180/math.pi,
      qLArm[7]*180/math.pi)

    print("RWrist:",qRArm[5]*180/math.pi,
      qRArm[6]*180/math.pi,
      qRArm[7]*180/math.pi)
--]]
    

  -- Go to the allowable position
  --
    local qL_approach, doneL
    qL_approach, doneL = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
    Body.set_larm_command_position( qL_approach )
  
    local qR_approach, doneR
    qR_approach, doneR = util.approachTol( qRArm, qR_desired, dqArmMax, dt )
    Body.set_rarm_command_position( qR_approach )
--

--FOR TESTING ONLY (interpolation makes giltch)
--Body.set_larm_command_position( qL_desired)
--Body.set_rarm_command_position( qR_desired)
    -- NOTE: This is kinda poor...
    -- Should calculate from arm angles
    turnAngleCurrent = turnAngleCommand
  end

  
--[[
  if qL_desired==true and qR_desired==true then
    return'done'
  end
--]]

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
