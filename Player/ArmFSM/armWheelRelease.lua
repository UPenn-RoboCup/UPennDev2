local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local wheelrotate = require'wheelrotate'

-- Angular velocity limit

local t_init = 5.0
local t_grip = 5.0
local handle_pos, handle_yaw, handle_pitch, handle_radius=0,0,0,0;
local turnAngle = 0
local lShoulderYaw = 0;
local rShoulderYaw = 0;
local stage = 1;




local qLArmTarget = Config.arm.qLArmInit[3]
local qRArmTarget = Config.arm.qRArmInit[3]
local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local dturnAngleMax = 3*math.pi/180 -- 3 deg per sec
local trLArmTarget = {}
local trRArmTarget = {}

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Let's store wheel data here
  local wheel   = hcm.get_wheel_model()
  turnAngle = hcm.get_wheel_turnangle()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  print("Handle model:",wheel)

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)


-- Inner and outer radius
  handle_radius0 = handle_radius 
  handle_radius1 = handle_radius + 0.08

  stage = 1;

  
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  lShoulderYaw = qLArm[3];
  rShoulderYaw = qRArm[3];
   
  --hack for now
  hcm.set_joints_shoulderangle(lShoulderYaw)

  trLArmTarget = Body.get_forward_larm(qLArmTarget)
  trRArmTarget = Body.get_forward_rarm(qRArmTarget)

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  if stage==1 then --return to arm side-by-side position
    turnAngle,doneA = util.approachTol(turnAngle,0,dturnAngleMax, dt )
      ret = wheelrotate.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius0, turnAngle,dt,
      lShoulderYaw, rShoulderYaw)
    if ret==1 and doneA then stage=stage+1; 
    elseif ret==-1 then 
      --We cannot reach here... or we're in serious trouble
    end
  elseif stage==2 then --Now spread arms apart
    ret = wheelrotate.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt,
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; end
  else --Now lower the arm to position #2 using IK
    local qLArm = Body.get_larm_command_position()
    local qRArm = Body.get_rarm_command_position()
    local trLArm = Body.get_forward_larm(qLArm)
    local trRArm = Body.get_forward_rarm(qRArm)
    local trLArmApproach, doneL = util.approachTol(trLArm, trLArmTarget, dpArmMax, dt )
    local trRArmApproach, doneR = util.approachTol(trRArm, trRArmTarget, dpArmMax, dt )
    -- Get desired angles from current angles and target transform
    
    local qL_desired = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw)
    local qR_desired = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw)

    if qL_desired then 
      local qL_approach, doneL2 = util.approachTolRad( qLArm, qL_desired, dqArmMax, dt )
      Body.set_larm_command_position( qL_approach )
    end
    if qR_desired then
      local qR_approach, doneR2 = util.approachTolRad( qRArm, qR_desired, dqArmMax, dt )
      Body.set_rarm_command_position( qR_approach )
    end

    if doneL and doneR then
      return'done'
    end  
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state