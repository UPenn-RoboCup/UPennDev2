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
local timeout = 2
-- Track the torso
local uTorso, uLeft, uRight
local zLeft, zRight = 0, 0
local side

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  uTorso = mcm.get_status_uTorso()  
  uLeft = mcm.get_status_uLeft()
  uRight = mcm.get_status_uRight()
  side = mcm.get_teach_sway()
  side = side=='none' and 'left' or side
  print('Sway to the', side)
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
  if t - t_entry > timeout then return'timeout' end
  
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
  
  -- Check the CoM first
  if side=='left' then
    if l_ft[3] < 3*r_ft[3] then
      uTorso = uTorso + vector.new{0,0.0005,0}
      mcm.set_status_uTorso(uTorso)
    else
      side = 'right'
      mcm.set_teach_sway(side)
      --return'switch'
    end
  elseif side=='right' then
    --print('L FT', l_ft)
    --print('R FT', r_ft)
    if r_ft[3] < 3*l_ft[3] then
      uTorso = uTorso - vector.new{0,0.0005,0}
      mcm.set_status_uTorso(uTorso)
    else
      side = 'left'
      mcm.set_teach_sway(side)
      --return'switch'
    end
  end
  moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight)
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
