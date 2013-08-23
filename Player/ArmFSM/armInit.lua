local state = {}
state._NAME = 'armInit'

local Config     = require'Config'
local Body       = require'Body'
local util       = require'util'
local t_entry, t_update, t_finish

require('lcm')
require('unix')

local qLArmInit = Config.arm.qLArmInit 
local qRArmInit = Config.arm.qRArmInit 

-- Local for not polluting global scope
local left_arm_mode, right_arm_mode

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  left_arm_mode = 1 
  right_arm_mode = 1 
  
  --super slow: degrees per second
  local dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180 
  Body.set_arm_movement_velocity(dArmVelAngle) 

end

function state.update()
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

  if left_arm_mode==1 then
    Body.enable_larm_linear_movement(false)  
    Body.set_larm_target_position(qLArmInit[1]) 
    if Body.larm_joint_movement_done() then left_arm_mode = 2 end
  elseif left_arm_mode==2 then
    Body.set_larm_target_position(qLArmInit[2]) 
    if Body.larm_joint_movement_done() then left_arm_mode = 3 end
  elseif left_arm_mode==3 then
    Body.set_larm_target_position(qLArmInit[3]) 
    if Body.larm_joint_movement_done() then 
      left_arm_mode = 4 
    end
  end

  if right_arm_mode==1 then
    Body.enable_rarm_linear_movement(false)  
    Body.set_rarm_target_position(qRArmInit[1]) 
    if Body.rarm_joint_movement_done() then right_arm_mode = 2 end
  elseif right_arm_mode==2 then
    Body.set_rarm_target_position(qRArmInit[2]) 
    if Body.rarm_joint_movement_done() then right_arm_mode = 3 end
  elseif right_arm_mode==3 then
    Body.set_rarm_target_position(qRArmInit[3]) 
    if Body.rarm_joint_movement_done() then 
      right_arm_mode = 4 
    end
  end


  if left_arm_mode==4 and right_arm_mode==4 then
    return "done" 
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end