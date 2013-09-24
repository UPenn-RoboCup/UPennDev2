local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local handle_pos, handle_yaw, handle_pitch, handle_radius, turnAngle=0,0,0,0,0
local lShoulderYaw, rShoulderYaw = 0,0
local stage = 1;

local qLArmTarget = Config.arm.qLArmInit[3]
local qRArmTarget = Config.arm.qRArmInit[3]
local trLArmTarget = Body.get_forward_larm(qLArmTarget)
local trRArmTarget = Body.get_forward_rarm(qRArmTarget)

local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local dturnAngleMax = 3*math.pi/180 -- 3 deg per sec

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
    --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)
  lShoulderYaw = hcm.get_joints_qlshoulderyaw()
  rShoulderYaw = hcm.get_joints_qrshoulderyaw()
  stage = 1;
end

function state.update()

--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  if stage==1 then --Set the arm to grip-ready pose
    local trLArmTarget = {0.20,0.32, -0.15,
    -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
    local qLArmTarget = Body.get_inverse_larm(qLArm,trLArmTarget,lShoulderYaw)
    if not qLArmTarget then
      print("Left not possible!!!!")
      return
    end
    ret = movearm.setArmJoints(qLArmTarget,qRArm,dt)
    if ret==1 then stage=stage+1; end
  elseif stage==2 then --Move the arm forward using IK now 
    local trLArmTarget2 = {0.35,0.32, -0.15,
      -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
    local trRArm = Body.get_forward_rarm(qRArm)
    ret = movearm.setArmToPosition(trLArmTarget2, trRArm, dt, 
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; end
  elseif stage==3 then --Move the arm up to grip the handle
    local trLArmTarget3 = {0.35,0.32, -0.05,
      -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
    local trRArm = Body.get_forward_rarm(qRArm)
    ret = movearm.setArmToPosition(trLArmTarget3, trRArm, dt, 
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; end
  elseif stage==4 then
    Body.set_lgrip_percent(1) --Close gripper
    local trLArmTarget4 = {0.35,0.32, -0.10, --pull down
      -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
    local trRArm = Body.get_forward_rarm(qRArm)
    ret = movearm.setArmToPosition(trLArmTarget4, trRArm, dt, 
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; end
  elseif stage==5 then
    local trLArmTarget4 = {0.20,0.40, -0.10, --pull down
      -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
    local trRArm = Body.get_forward_rarm(qRArm)
    ret = movearm.setArmToPosition(trLArmTarget4, trRArm, dt, 
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; end

    --]]
    --[[
  elseif stage==5 then
    Body.set_lgrip_percent(1) --Close gripper
    local trLArmTarget3 = {0.30,0.32, 0.00, --pull down
      -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
    local trRArm = Body.get_forward_rarm(qRArm)
    ret = movearm.setArmToPosition(trLArmTarget2, trRArm, dt, 
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; end
--]]    
  else
--    return "done"
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state