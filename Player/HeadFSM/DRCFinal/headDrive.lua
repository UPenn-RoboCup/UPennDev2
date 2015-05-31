local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update
local state = {}
state._NAME = ...

--faster
local headSpeed = {45 * DEG_TO_RAD, 30 * DEG_TO_RAD}


function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

  Body.set_head_torque_enable(1)

end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

--	local centerAngles = {0, 0*DEG_TO_RAD-Body.get_rpy()[2]}
  local centerAngles = {0, 0}
  local lookAngles = hcm.get_teleop_drive_head()
  local centerAngles = {math.pi + lookAngles[1] * 90*DEG_TO_RAD, lookAngles[2]*45*DEG_TO_RAD}

  local headNow = Body.get_head_command_position()
  local apprAng, doneHead = util.approachTol(headNow, centerAngles, headSpeed, dt, 1*DEG_TO_RAD)
  Body.set_head_command_position(apprAng)
end

function state.exit()
  --hack for stair climbing
  print(state._NAME..' Exit' )
end

return state
