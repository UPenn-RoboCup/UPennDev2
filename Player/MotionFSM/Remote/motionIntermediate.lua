local state = {}
state._NAME = ...
local vector = require'vector'
local util = require'util'

local timeout = 10.0
local t_entry, t_update
local rc

local lleg0 = vector.new{
  0, 0.05, -0.18, 
  0.4,
  -0.22, -0.05
}
local rleg0 = vector.new{
  0, -0.05, -0.18, 
  0.4,
  -0.22, 0.05
}
local larm0 = vector.new{
  0.39, .4, 0, 
  -1.18,
  0,0,0
}
local rarm0 = vector.new{
  0.39, -.4, 0, 
  -1.18,
  0,0,0
}

function state.entry()
  print(state._NAME..' Entry')

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
--  print(state._NAME..' Update')
  -- Get the time of update
  local t = Body.get_time()

  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  if t - t_entry > timeout then return'timeout' end

  local lleg, donelleg = util.approachTol(
    Body.get_lleg_command_position(),
    lleg0, 5*DEG_TO_RAD*vector.ones(#lleg0), t_diff )
    
  local rleg, donerleg = util.approachTol(
    Body.get_rleg_command_position(),
    rleg0, 5*DEG_TO_RAD*vector.ones(#rleg0), t_diff )
    
  local rarm, donerarm = util.approachTol(
    Body.get_rarm_command_position(),
    rarm0, 5*DEG_TO_RAD*vector.ones(#rleg0), t_diff )
    
  local larm, donelarm = util.approachTol(
    Body.get_larm_command_position(),
    larm0, 5*DEG_TO_RAD*vector.ones(#lleg0), t_diff )
    
  Body.set_lleg_command_position(lleg)
  Body.set_rleg_command_position(rleg)
  Body.set_larm_command_position(larm)
  Body.set_rarm_command_position(rarm)
    
  if donelleg and donerleg then return'done' end

end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state
