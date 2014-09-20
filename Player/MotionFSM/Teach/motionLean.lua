--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local moveleg = require'moveleg'
local util = require'util'
require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step
-- Track the torso
local uTorso, uLeft, uRight
local zLeft, zRight = 0, 0
local side
local relTorso, relX, relY, relR
local uTorso = vector.new({Config.walk.supportX, 0, 0})
local dTorsoScale = 0.0005

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
  print('Lean to the', side)
  --relTorso = util.pose_global(uTorso, util.se2_interpolate(.5, uLeft, uRight))
  ----[[
  relTorso = util.pose_relative(uLeft, uRight)
  relR = math.sqrt(relTorso.x^2+relTorso.y^2)
  relX = dTorsoScale * relTorso.x / relR
  relY = dTorsoScale * relTorso.y / relR
  print(relTorso, relX, relY)
  dTorso = vector.pose{relX, relY, 0}
  --]]
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
  if side=='left' then
    -- left is the frame
    if l_ft[3] < 3*r_ft[3] then
      uTorso = uTorso + dTorso
      mcm.set_status_uTorso(uTorso)
    else
      return'done'
    end
  elseif side=='right' then
    --print('L FT', l_ft)
    --print('R FT', r_ft)
    if r_ft[3] < 3*l_ft[3] then
      uTorso = uTorso - dTorso
      mcm.set_status_uTorso(uTorso)
    else
      return'done'
    end
  end
  
  moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight)
  
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
