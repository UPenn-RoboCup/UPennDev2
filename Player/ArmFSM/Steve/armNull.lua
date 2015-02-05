local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update, t_finish
local timeout = 10.0

local stictionL, stictionR
local STICTION_THRESHOLD = 10
local movingL, movingR

-- The null degree of freedom
-- TODO: Use a jacbian and fun stuff
-- For now, just use the free parameter of the IK, which is just the third joint
local dof = 3

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t
  -- First, determine the stiction current at present
  local curL = Body.get_larm_current()
  local curR = Body.get_rarm_current()
  stictionL, stictionR = curL[dof], curR[dof]
  -- Assume not moving to begin with
  movingL, movingR = false, false
end

function state.update()
  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

  -- Grab the current
  local curR = Body.get_rarm_current()[dof]

  -- Check if away from the stiction significantly
  if not movingR and math.abs(curR-stictionR) > STICTION_THRESHOLD then
    print('Moving the arm!')
    movingR = true
  end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
