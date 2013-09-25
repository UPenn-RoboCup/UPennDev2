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

local dDoorAngleMax = 3*math.pi/180
local door_yaw = 0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  lShoulderYaw = hcm.get_joints_qlshoulderyaw()
  rShoulderYaw = hcm.get_joints_qrshoulderyaw()
  door_yaw = 0  
end

function state.update()

--  print(state._NAME..' Update' )
  -- Get the time of update

  

  local trLArmTarget = vector.new(
    {0.18,0.31, -0.15,
    -90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})


--Left hinged door
  hinge_pos = vector.new({0.40,0.60,-0.15});
  door_r = -0.30;
  grip_offset_x = -0.05;

  --Temporary hack for testing!
  door_yaw_target = hcm.get_wheel_turnangle()

  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trRArm = Body.get_forward_rarm(qRArm)
  
 --Open door
  local door_yaw1, doneD = util.approachTol(door_yaw,door_yaw_target, 
                             dDoorAngleMax,dt)
  print(door_yaw*180/math.pi)
  hinge_pos5 = hinge_pos + vector.new({0,0,-0.03})
  local trLArmTarget5 = movearm.getDoorHandlePosition(
    hinge_pos5, door_r, door_yaw1, grip_offset_x)  
  ret = movearm.setArmToPositionAdapt(trLArmTarget5, trRArm, dt)          
  if ret~=-1 then
    door_yaw = door_yaw1
  else
    hcm.set_wheel_turnangle(door_yaw) --reset value
  end  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state