local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish
local timeout = 10.0
require'mcm'

local qLArm0, qRArm0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  qLArm0 = Body.get_larm_command_position()
  qRArm0 = Body.get_rarm_command_position()

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local steering = hcm.get_teleop_steering()
  local qLArm = util.shallow_copy(qLArm0)
  qLArm[7] = qLArm0[7]+steering

  Body.set_larm_command_position(qLArm)
end

function state.exit()
  print(state._NAME..' Exit' )
  for i=1,10 do
    Body.set_larm_torque_enable(1)
    Body.set_rarm_torque_enable(1)
    unix.usleep(1e5)
  end
end

return state
