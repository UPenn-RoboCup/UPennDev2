--New staticwalk


local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local stepHeight  = Config.walk.stepHeight
--local tStep = Config.walk.tStep
local tStep = 40

-- Save the velocity between update cycles
local velCurrent = vector.new{0, 0, 0}

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}
  
local iStep

-- What foot trajectory are we using?
local foot_traj_func  
if Config.walk.foot_traj==1 then foot_traj_func = moveleg.foot_trajectory_base
else foot_traj_func = moveleg.foot_trajectory_square end

---------------------------
-- State machine methods --
---------------------------
function walk.entry()
  print(walk._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Reset our velocity
  velCurrent = vector.new{0,0,0}
  mcm.set_walk_vel(velCurrent)

  --Read stored feet and torso poses 
  local uTorso0 = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  uTorso_now, uTorso_next = uTorso0, uTorso0
  uLeft_now,  uLeft_next  = uLeft,  uLeft
  uRight_now, uRight_next = uRight, uRight

  --Now we advance a step at next update  
  t_last_step = Body.get_time() - tStep 

  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t   -- Save this at the last update time

  local ph = (t-t_last_step)/tStep
  if ph>1 then  
    local stoprequest = mcm.get_walk_stoprequest() --Should we stop now?
    if stoprequest>0 then return"done" end 
    ph = ph % 1
    iStep = iStep + 1  -- Increment the step index    
    supportLeg = iStep % 2 -- supportLeg: 0 for left support, 1 for right support
    velCurrent = moveleg.update_velocity(velCurrent)     -- Update the velocity via a filter

    uLeft_now,uRight_now,uTorso_now, uLeft_next,uRight_next,uTorso_next, uSupport=
      moveleg.advance_step(uLeft_next,uRight_next,uTorso_next,iStep,velCurrent,false)
    t_last_step = Body.get_time()   
  end

  uTorso = walk.get_static_com(ph)
  uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

  local phSingle = moveleg.get_ph_single(ph,0.4,0.6)

  local uLeft, uRight, zLeft, zRight = uLeft_now, uRight_now, 0,0
  if supportLeg == 0 then  -- Left support    
    uRight,zRight = foot_traj_func(phSingle,uRight_now,uRight_next,stepHeight)    
  else    -- Right support    
    uLeft,zLeft = foot_traj_func(phSingle,uLeft_now,uLeft_next,stepHeight)    
  end

  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )
  gyro_rpy={0,0,0}
  delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,phSingle,gyro_rpy, angleShift)
  moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)
end -- walk.update

function walk.exit()
  mcm.set_status_uLeft(uLeft_next)
  mcm.set_status_uRight(uRight_next)
  mcm.set_status_uTorso(uTorso_next)
  print(walk._NAME..' Exit')  
end

function walk.get_static_com(ph)
  local uTorso
  phSingle1 = 0.3
  phSingle2 = 0.7
  if ph<phSingle1 then
    ph2 = ph/phSingle1;
    uTorso = util.se2_interpolate( ph2  ,uTorso_now, uSupport)
  elseif ph<phSingle2 then
    uTorso = uSupport
  else
    ph2 = (1-ph)/(1-phSingle2);
    uTorso = util.se2_interpolate( ph2 ,uTorso_next, uSupport)
  end    
  return uTorso
end


return walk
