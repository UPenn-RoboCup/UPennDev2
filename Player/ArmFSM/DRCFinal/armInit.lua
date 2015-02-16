--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 15.0

-- Goal position is arm Init, with hands in front, ready to manipulate

local qLArmTarget, qRArmTarget



--SJ: now SLOWLY move joint one by one



function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  stage = 1

  --Slowly close all fingers
	--[[
  Body.move_lgrip1(Config.arm.torque.movement)
  Body.move_lgrip2(Config.arm.torque.movement)
  Body.move_rgrip1(Config.arm.torque.movement)
  Body.move_rgrip2(Config.arm.torque.movement)
	--]]

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()




  local qLWrist = Body.get_inverse_lwrist(qLArm,Config.arm.pLWristTarget0,
    Config.arm.lShoulderYaw0)
  local qRWrist = Body.get_inverse_rwrist(qRArm,Config.arm.pRWristTarget0,
    Config.arm.rShoulderYaw0)

  local qLWrist = Body.get_inverse_lwrist(qLArm,Config.arm.pLWristTarget0,0)
  local qRWrist = Body.get_inverse_rwrist(qRArm,Config.arm.pRWristTarget0,0)



  qLArmTarget = Body.get_inverse_arm_given_wrist(qLWrist, Config.arm.lrpy0)
  qRArmTarget = Body.get_inverse_arm_given_wrist(qRWrist, Config.arm.rrpy0)


  print(string.format("QLArmTarget: %.2f %.2f %.2f %.2f",
    unpack( vector.new(qLArmTarget)*RAD_TO_DEG)
  ))

--[[
  qLArmTarget = vector.new({90,0,0,-170,90,10,0})*DEG_TO_RAD
  qRArmTarget = vector.new({90,0,0,-170,-90,-10,0})*DEG_TO_RAD
--]]





  mcm.set_stance_enable_torso_track(0)

  mcm.set_arm_dqVelLeft(Config.arm.vel_angular_limit)
  mcm.set_arm_dqVelRight(Config.arm.vel_angular_limit)
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local ret = movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)
  if ret==1 then return "done" end
end

function state.exit()
  local libArmPlan = require 'libArmPlan'
  local arm_planner = libArmPlan.new_planner()
  --print("qLArm:",unpack(qLArmTarget))
  arm_planner:reset_torso_comp(qLArmTarget,qRArmTarget)
--[[
  Body.move_lgrip1(0)
  Body.move_lgrip2(0)
  Body.move_rgrip1(0)
  Body.move_rgrip2(0)
--]]

--[[
  if not IS_WEBOTS then
    for i=1,10 do      
      Body.set_larm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
      unix.usleep(1e6*0.01);  
      Body.set_larm_comma0nd_acceleration({200,200,200,200,200,200,200})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_acceleration({200,200,200,200,200,200,200})
      unix.usleep(1e6*0.01);
    end
  end
  
  Body.set_lgrip_percent(0.9)
  Body.set_rgrip_percent(0.9)
--]]

  print(state._NAME..' Exit' )
end

return state

