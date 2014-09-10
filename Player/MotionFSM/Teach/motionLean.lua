--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local moveleg = require'moveleg'
require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step
-- Trak the torso
local uTorso, uLeft, uRight


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  uTorso = mcm.get_status_uTorso()  
  uLeft = mcm.get_status_uLeft()
  uRight = mcm.get_status_uRight()
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
  --print('L FT', l_ft)
  --print('R FT', r_ft)
  
  -- Check the CoM first
  if 4*r_ft[3] > l_ft[3] then
    uTorso = uTorso + vector.new{0,0.0001,0}
    -- Save
    --mcm.set_status_uLeft(uLeft)
    --mcm.set_status_uRight(uRight)
    mcm.set_status_uTorso(uTorso)
  
    local zLeft, zRight = 0, 0
    moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight)
  end
  
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
