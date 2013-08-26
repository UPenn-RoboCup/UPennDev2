local Body = require'Body'
local timeout = 3.0
local t_entry, t_update

local state = {}
state._NAME = 'motionFall'

function state.entry()
  print(state._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Relax all the joints while falling
  Body.set_body_hardness(0)

  --[[
  --Ukemi motion (safe fall)
  local imuAngleY = Body.get_sensor_imuAngle(2);
  if (imuAngleY > 0) then --Front falling 
print("UKEMI FRONT")
    Body.set_larm_hardness({0.6,0,0.6});
    Body.set_rarm_hardness({0.6,0,0.6});
    Body.set_larm_command(qLArmFront);
    Body.set_rarm_command(qRArmFront);
  else
  end
--]]

end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  if t - t_entry > timeout then return'timeout' end

  -- Do not exit from this state, 
  -- since we will determine if it is safe to move somewhere else

  -- TODO: Set all joint reading

end

function state.exit()
  print(state._NAME..' Exit' )
  
  -- Ensure we do not jerk in the worst case
  local qSensor = Body.get_sensor_position()
  Body.set_actuator_command_position(qSensor)
end

return state