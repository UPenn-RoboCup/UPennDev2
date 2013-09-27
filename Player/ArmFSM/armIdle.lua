local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update, t_finish
local timeout = 10.0

local qLArm, qRArm

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t


  -- Torque OFF the motors
  Body.set_larm_torque_enable(0)
  Body.set_rarm_torque_enable(0)  

  -- Initialize our joint positions estimate
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()

  -- Request new readings
  Body.request_larm_position()
  Body.request_rarm_position()

end

function state.update()
--  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  local updatedL, updatedR
  qL, updatedL = Body.get_larm_position()
  qR, updatedR = Body.get_rarm_position()
  
  if updatedL then
    qLArm = qL
    Body.request_larm_position()
  end
  if updatedR then
    qRArm = qR
    Body.request_rarm_position()
  end

end

function state.exit()
  print(state._NAME..' Exit' )
  Body.set_larm_torque_enable(1)
  Body.set_rarm_torque_enable(1)
  -- Wait 10 milliseconds for motors to turn on
  unix.usleep(1e4)  
  --Set the commanded position
  Body.set_larm_command_position(qLArm)
  Body.set_rarm_command_position(qRArm)

end

return state