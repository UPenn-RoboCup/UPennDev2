local state = {}
state._NAME = ...

--motionInit: initialize legs to correct position


require'mcm'
require'hcm'
local Body       = require'Body'
local util       = require'util'
local vector     = require'vector'
local t_entry, t_update, t_finish

local stage = 1

function state.entry()
  print(state._NAME..' Entry' )
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  -- Get the time of update
  local t  = Body.get_time()

  local legBias = vector.new(mcm.get_leg_bias())
  qLegsTarget = legBias

  local qLLegTarget = vector.slice(qLegsTarget,1,6)
  local qRLegTarget = vector.slice(qLegsTarget,7,12)

  Body.set_lleg_command_position(qLLegTarget)
  Body.set_rleg_command_position(qRLegTarget)
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
