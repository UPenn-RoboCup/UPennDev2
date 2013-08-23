local state = {}
state._NAME = 'armReady'
local Config     = require'Config'
local Body       = require'Body'
local t_entry, t_update, t_finish

local qLArmInit=Config.arm.qLArmInit;
local qRArmInit=Config.arm.qRArmInit;

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  Body.enable_larm_linear_movement(false); 
  Body.enable_rarm_linear_movement(false); 
  
  --super slow
  dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180; --30 degree per second
  Body.set_arm_movement_velocity(dArmVelAngle);

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

    Body.enable_larm_linear_movement(false); 
    Body.set_larm_target_position(qLArmInit[#qLArmInit]);
    Body.set_rarm_target_position(qRArmInit[#qRArmInit]);
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state