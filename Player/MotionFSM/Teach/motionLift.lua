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
local timeout = 20
-- Track the torso
local uTorso, uLeft, uRight
local zLeft, zRight
local side

-- Lift properties
local zTarget = 0.16
local dz = 0.001

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
  print('Support on the', side)
  zLeft, zRight = unpack(mcm.get_status_zLeg())
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t - t_entry > timeout then return'timeout' end
  
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
  
  -- TODO: Make sure we lean enough before lifting our legs
  if side=='left' and l_ft[3] < 2*r_ft[3] then return'lean' end
  if side=='right' and r_ft[3] < 2*l_ft[3] then return'lean' end
  
  -- Increment the leg height
  if side=='left' then
    zRight = zRight + dz
    if zRight > zTarget then return'done' end
  else
    zLeft = zLeft + dz
    if zLeft > zTarget then return'done' end
  end
  mcm.set_status_zLeg{zLeft, zRight}
  moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight, dt)
  
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
