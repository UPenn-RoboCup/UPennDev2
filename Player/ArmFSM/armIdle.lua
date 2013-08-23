local state = {}
state._NAME = 'armIdle'

local Config     = require'Config'
local Body       = require'Body'

local timeout = 10.0

local t_entry, t_update, t_finish

local qLArmInit = Config.arm.qLArmInit
local qRArmInit = Config.arm.qRArmInit

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Move the arms to the Idle position
  -- TODO: Is this the best implementation?

-- TODO: This state machine should implement these methods,
-- and not the Body
--[[
  Body.enable_larm_linear_movement(false) 
  Body.set_larm_target_position(qLArmInit[1])
  Body.set_rarm_target_position(qRArmInit[1])
  Body.set_lhand_position(Config.arm.FingerOpen)
  Body.set_rhand_position(Config.arm.FingerOpen)
  --]]
end

function state.update()
--  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

end

function state.exit()
  print(state._NAME..' Exit' )

end

return state