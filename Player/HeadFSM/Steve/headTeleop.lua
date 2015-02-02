local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update

-- Neck limits
local headSpeed = 5 * DEG_TO_RAD * vector.ones(2)
local headThresh = 1 * DEG_TO_RAD * vector.ones(2)

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
  hcm.set_teleop_head(Body.get_head_position())
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t 

  -- Grab the target
  local headAngles = hcm.get_teleop_head()
	local headNow = Body.get_head_command_position()
  local apprAng, doneHead = util.approachTol(headNow, headAngles, headSpeed, dt, headThresh)
	
  -- Update the motors
	Body.set_head_command_position(doneHead and headAngles or apprAng)
  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
