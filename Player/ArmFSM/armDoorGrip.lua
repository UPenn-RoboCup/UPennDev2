local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local dDoorAngleMax = 3*math.pi/180
local stage = 1;
local hinge_pos={}
local door_r,grip_offset_x,door_yaw,door_yaw1=0,0,0,0
local door_hand = 0

local handle_clearance = vector.new({0,0,-0.05})
local handle_pulldown = vector.new({0,0,-0.03})

--Initial arm pose 
local trLArmTarget = vector.new({0.18,0.31, -0.15,-90*Body.DEG_TO_RAD,-15*Body.DEG_TO_RAD,0})
local qLArmTarget0 = vector.new({138,13.7,-6.6,-84.4,-97.1,-52.4,-9.7})*Body.DEG_TO_RAD
local qLArmTarget = Body.get_inverse_larm(qLArmTarget0,trLArmTarget,-6.6*Body.DEG_TO_RAD)

--not working with rarm now... should be fixed
local trRArmTarget = vector.new({0.18,-0.31, -0.15,90*Body.DEG_TO_RAD,-15*Body.DEG_TO_RAD,0})
local qRArmTarget0 = vector.new({138,-13.7,6.6,-84.4,97.1,52.4,9.7})*Body.DEG_TO_RAD

--local trRArm = Body.get_forward_rarm(qRArmTarget0)
--print("TrRARm:",unpack(trRArm))

local qRArmTarget = Body.get_inverse_rarm(qRArmTarget0,trRArmTarget,6.6*Body.DEG_TO_RAD)
--print("qRArmTarget:",unpack(qRArmTarget))

--qRArmTarget = qRArmTarget0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)
  --Read door model from shm
  hinge_pos = hcm.get_door_hinge_pos()  
  door_r = hcm.get_door_r()
  grip_offset_x = hcm.get_door_grip_offset_x()  
  door_yaw = hcm.get_door_yaw()
  door_hand = hcm.get_door_hand()
  
  door_yaw_target = hcm.get_door_yaw_target()  
  door_yaw = 0

  stage = 1;  
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  
  door_yaw_target = hcm.get_door_yaw_target()

--TODO: values not set correctly for whatever reason
  door_hand = 1;  
  door_yaw_target = -20*math.pi/180
--

  --Right hand testing with webots
  door_hand = 0;  
  hinge_pos = vector.new({0.45,-0.95,0})
  door_r = 0.60
  grip_offset_x = -0.05
  door_yaw_target = 30*math.pi/180
  door_tilt = -10*math.pi/180
  --


  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm)
  local trRArm = Body.get_forward_rarm(qRArm)

  if stage==1 then --Set the arm to grip-ready pose
    if door_hand==1 then --Left hand
      ret = movearm.setArmJoints(qLArmTarget,qRArm,dt,
        {10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,
        30*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD}
        )
    else
      ret = movearm.setArmJoints(qLArm,qRArmTarget,dt,
        {10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,
        30*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD}

        )
    end
    if ret==1 then stage=stage+1; end
  else    

    local trArmTarget={}    
    if stage==2 then --Move the arm forward using IK now     
      trArmTarget= movearm.getDoorHandlePosition(
        hinge_pos+handle_clearance, door_r, door_yaw, grip_offset_x, door_hand)
    elseif stage==3 then --Move the arm up to grip the handle    
      trArmTarget = movearm.getDoorHandlePosition(
        hinge_pos, door_r, door_yaw, grip_offset_x,door_hand)
    elseif stage==4 then --Close gripper and pull down the lever
      trArmTarget = movearm.getDoorHandlePosition(
        hinge_pos + handle_pulldown, door_r, door_yaw, grip_offset_x, door_hand)

      if door_hand==1 then
        Body.set_lgrip_percent(1) --Close gripper  
      else
        Body.set_rgrip_percent(1) --Close gripper  
      end

    elseif stage==5 then --open the door            
      door_yaw1,doneD = util.approachTol(door_yaw,door_yaw_target, 
      dDoorAngleMax,dt)    
      trArmTarget = movearm.getDoorHandlePosition(
        hinge_pos + handle_pulldown, door_r, door_yaw1, grip_offset_x, door_hand)
    end




    if door_hand==1 then --Left hand
      ret = movearm.setArmToPositionAdapt(trArmTarget, trRArm, dt)
    else
      ret = movearm.setArmToPositionAdapt(trLArm, trArmTarget, dt)
    end
    
    if stage==5 then
      if ret==-1 then       
        hcm.set_door_yaw_target(door_yaw)            
      else
        door_yaw = door_yaw1;
      end
    elseif ret==1 then  stage=stage+1
    end
    
  end  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state