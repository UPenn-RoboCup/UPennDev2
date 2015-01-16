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
local timeout = 30
-- Track the torso
local uTorso, uLeft, uRight
local zLeft, zRight
local side

-- Lift properties
local xTarget
local dpose = vector.pose{0.002, 0, 0}
local dzTarget = 0.01
local zTarget
local supportDir, supportFoot, supportPoint
local supportX, supportY = Config.walk.supportX, Config.walk.supportY
local dTorsoScale = 0.0005
local uTorso, dTorso

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  uTorso = mcm.get_status_uTorso()  
  uLeft = mcm.get_status_uLeft()
  uRight = mcm.get_status_uRight()
  zLeft, zRight = unpack(mcm.get_status_zLeg())
  side = zLeft < zRight and 'left' or 'right'
  mcm.set_teach_sway(side)
  print('Support on the', side)
  
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
  local side_check = l_ft[3]>r_ft[3] and 'left' or 'right'
  if side_check~=side then print('BAD SUPPORT FOR HOLD!') end
  
  -- Join the x
  xTarget = (side=='left' and uLeft[1] or uRight[1])
  -- lift the non support a bit
  zTarget = (side=='left' and zRight or zLeft) + dzTarget
  print('TARGETS', xTarget, zTarget)
  print('LEFT', uLeft[1], zLeft)
  print('RIGHT', uRight[1], zRight)
  
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
  if t - t_entry > timeout then return'timeout' end
  
  -- TODO: If the foot hits something, then must retract and lower the foot!
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
  
  -- Torso shift
  local torsoDone = false
  local relTorso = util.pose_relative(uTorso, supportPoint)
  local drTorso = math.sqrt(relTorso.x^2 + relTorso.y^2)
  --print('relTorso', supportPoint, relTorso, uTorso)
  if drTorso<1e-3 and math.abs(relTorso.a)<DEG_TO_RAD then
    torsoDone = true
  else
    mcm.set_status_uTorso(uTorso)
    -- How spread are our feet?
    local dX = dTorsoScale * relTorso.x / drTorso
    local dY = dTorsoScale * relTorso.y / drTorso
    -- TODO: Add the angle
    -- Move toward the support point
    uTorso = uTorso - vector.pose{dX, dY, 0}
  end
  
  -- Increment the leg height
  local zDone, dz
  if torsoDone then
    if side=='left' then
      zDone = zRight > zTarget
      dz = (zDone or not torsoDone) and 0 or 0.0002
      zRight = zRight + dz
    else
      zDone = zLeft > zTarget
      dz = (zDone or not torsoDone) and 0 or 0.0002
      zLeft = zLeft + dz
    end
  end
  
  local xDone
  if zDone then
    if side=='left' then
      uRight = uRight + dpose
      xDone = uRight[1] > xTarget
      if not xDone then mcm.set_status_uRight(uRight) end
    else
      uLeft = uLeft + dpose
      xDone = uLeft[1] > xTarget
      if not xDone then mcm.set_status_uLeft(uLeft) end
    end
  end
  
  if torsoDone and xDone and zDone then return'done' end
  
  moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight, dt)
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
