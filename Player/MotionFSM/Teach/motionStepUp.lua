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
local zLeft, zRight
local side
local supportDir, supportFoot, supportPoint
local uTorso, dTorso
local dTorsoScale = 0.001
local supportX, supportY = Config.walk.supportX, Config.walk.supportY
local zTarget = 0.16
local bH0, bH_final

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  uTorso = mcm.get_status_uTorso()  
  uLeft, uRight = mcm.get_status_uLeft(), mcm.get_status_uRight()
  zLeft, zRight = unpack(mcm.get_status_zLeg())
  side = mcm.get_teach_sway()
  side = side=='none' and 'left' or side
  print('Support on the', side)
  
  supportDir = side=='left' and 1 or -1
  supportFoot = side=='left' and uLeft or uRight
  supportPoint = util.pose_global({supportX, supportDir*supportY, 0}, supportFoot)
  print('SUPPORT POINT', supportPoint, uTorso)

end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  
  -- Where is our offset?
  local relTorso = util.pose_relative(uTorso, supportPoint)
  local drTorso = math.sqrt(relTorso.x^2 + relTorso.y^2)
  local torsoDone = drTorso<1e-3 and math.abs(relTorso.a)<DEG_TO_RAD
  
  -- How spread are our feet?
  local dX = dTorsoScale * relTorso.x / drTorso
  local dY = dTorsoScale * relTorso.y / drTorso
  -- TODO: Add the angle
  -- Move toward the support point
  uTorso = uTorso - vector.pose{dX, dY, 0}
  
  local zDone, dz
  if side=='left' then
    zDone = zRight > zTarget
    dz = (zDone or not torsoDone) and 0 or 0.0002
    zRight = zRight + dz
  else
    zDone = zLeft > zTarget
    dz = (zDone or not torsoDone) and 0 or 0.0002
    zLeft = zLeft + dz
  end
  
  if torsoDone and zDone then
    local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
    print('L FT', l_ft)
    print('R FT', r_ft)
    return'done'
  end
  
  -- Update
  mcm.set_status_zLeg{zLeft, zRight}
  mcm.set_status_uTorso(uTorso)
  moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight, dt)
end

function state.exit()
  print(state._NAME..' Exit')
  
end

return state
