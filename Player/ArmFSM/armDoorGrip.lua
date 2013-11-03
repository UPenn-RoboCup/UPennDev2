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

local qLArmTarget, qRArmTarget

local velJointInit =  {10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,
                      30*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD}


local shoulderYaw = -6.6*Body.DEG_TO_RAD




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

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  --Initial arm pose 
  
  local trLArmTarget0 = vector.new({0.18,0.31, -0.15,-90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})
  local trRArmTarget0 = vector.new({0.18,-0.31, -0.15,90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})
  

  --local trRArm = Body.get_forward_rarm(qRArmTarget0)
  --print("TrRARm:",unpack(trRArm))




  qLArmTarget0 = Body.get_inverse_larm(qLArm,trLArmTarget0,-shoulderYaw)
  qRArmTarget0 = Body.get_inverse_rarm(qRArm,trRArmTarget0,shoulderYaw)

  qLArmTarget0[6] = qLArm[6] --Don't turn wrist roll at first
  qRArmTarget0[6] = qRArm[6] --Don't turn wrist roll at first

  qLArmTarget1 = Body.get_inverse_larm(qLArm,trLArmTarget0,-shoulderYaw)
  qRArmTarget1 = Body.get_inverse_rarm(qRArm,trRArmTarget0,shoulderYaw)

  door_yaw_target = hcm.get_door_yaw_target()


  --Right hand testing with webots
  door_hand = 0;  --0 for right, 1 for left
  hinge_pos = vector.new({0.45,-0.95,-0.10})
  hinge_pos = vector.new({0.45,-0.95,-0.05})

  hinge_pos = vector.new({0.55,-0.95,-0.05})

  door_r = 0.60
  grip_offset_x = -0.05
  door_yaw_target = 30*math.pi/180
  --  

  local trArmTarget0, qArm
  if door_hand==0 then --Right hand
    trArmTarget0 = vector.new({0.18,-0.31, -0.15,90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})
    qArm = qRArmTarget1
    trArmTarget05 = vector.new({0.35,-0.31, hinge_pos[3],90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})
  else --Left hand
    trArmTarget0 = vector.new({0.18,0.31, -0.15,-90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})

    trArmTarget05 = vector.new({0.35,0.31, hinge_pos[3],-90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0})
    qArm = qLArmTarget1
  end

  --Reach to the position BELOW the handle
  local trArmTarget1 = movearm.getDoorHandlePosition(
      hinge_pos+handle_clearance, door_r, door_yaw, grip_offset_x, door_hand)

  --Raise the hand up
  local trArmTarget2 = movearm.getDoorHandlePosition(
      hinge_pos, door_r, door_yaw, grip_offset_x,door_hand)

  --Pull the handle down
  local trArmTarget3 = movearm.getDoorHandlePosition(
      hinge_pos + handle_pulldown, door_r, door_yaw, grip_offset_x, door_hand)

  

  print("Planning ArmPlan1")
  ArmPlan1,qArm1 = arm_planner:plan_arm(qArm, trArmTarget0, door_hand)  

  print("Planning ArmPlan15")
  ArmPlan15,qArm15 = arm_planner:plan_arm(qArm1, trArmTarget05, door_hand)  

  print("Planning ArmPlan2")
  ArmPlan2,qArm2 = arm_planner:plan_arm(qArm15, trArmTarget1, door_hand)  

  print("Planning ArmPlan3")
  ArmPlan3,qArm3 = arm_planner:plan_arm(qArm2, trArmTarget2, door_hand)  

  print("Planning ArmPlan4")
  ArmPlan4,qArm4 = arm_planner:plan_arm(qArm3, trArmTarget3, door_hand)  







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

  local trLArm = Body.get_forward_larm(qLArm);
  local trRArm = Body.get_forward_rarm(qRArm)  



  if stage==1 then --Set the arm to grip-ready pose    
    --Turn yaw angles first
    if door_hand==1 then ret = movearm.setArmJoints(qLArmTarget0,qRArm,dt) --Left hand      
    else ret = movearm.setArmJoints(qLArm,qRArmTarget0,dt) end
    if ret==1 then stage=stage+1; end
  elseif stage==2 then --Now turn wrist roll angle    
    if door_hand==1 then ret = movearm.setArmJoints(qLArmTarget1,qRArm,dt) --Left hand      
    else ret = movearm.setArmJoints(qLArm,qRArmTarget1,dt) end    
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
      if door_hand==1 then  Body.set_lgrip_percent(1) 
      else Body.set_rgrip_percent(1) end
    elseif stage==5 then --open the door            
      door_yaw1,doneD = util.approachTol(door_yaw,door_yaw_target, 
      dDoorAngleMax,dt)    
      trArmTarget = movearm.getDoorHandlePosition(
        hinge_pos + handle_pulldown, door_r, door_yaw1, grip_offset_x, door_hand)
    elseif stage==6 then
      return

    end

    if door_hand==1 then --Left hand
      --ret = movearm.setArmToPositionAdapt(trArmTarget, trRArm, dt)
      ret = movearm.setArmToPositionAdapt(trArmTarget, trRArm, dt, -shoulderYaw, shoulderYaw)
    else
      --ret = movearm.setArmToPositionAdapt(trLArm, trArmTarget, dt)
      ret = movearm.setArmToPositionAdapt(trLArm, trArmTarget, dt, -shoulderYaw, shoulderYaw)
    end        

    if stage==5 then
      if ret==-1 then       
        hcm.set_door_yaw_target(door_yaw)   
        print("Final angle:",door_yaw*180/math.pi)         
        stage = stage+ 1
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