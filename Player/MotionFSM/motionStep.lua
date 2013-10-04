--Step-on testing code
--Based on humblewalk (for now)

local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libReactiveZMP = require'libReactiveZMP'
local zmp_solver
require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local tStep = Config.walk.tStep

local tStep = 10


local stepHeight  = Config.walk.stepHeight

-- Save the velocity between update cycles
local velCurrent = vector.new{0, 0, 0}

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local iStep

is_logging = false -- logging
local LOG_F_SENSOR

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

  -- Initiate the ZMP solver
  zmp_solver = libReactiveZMP.new_solver({
    ['tStep'] = Config.walk.tStep,
    ['tZMP']  = Config.walk.tZMP,
    ['start_phase']  = Config.walk.phSingle[1],
    ['finish_phase'] = Config.walk.phSingle[2],
  })
   
  --Now we advance a step at next update  
  t_last_step = Body.get_time() - tStep 

  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag

  -- log file
  if is_logging then
    LOG_F_SENSOR = io.open('feet.log','w')
    local log_entry = string.format('t ph supportLeg values\n')
    LOG_F_SENSOR:write(log_entry)
  end
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t   -- Save this at the last update time

  -- Grab the phase of the current step
  local ph = (t-t_last_step)/tStep
  if ph>1 then
    --Should we stop now?
    local stoprequest = mcm.get_walk_stoprequest()
    if stoprequest>0 then return"done"   end
    ph = ph % 1
    iStep = iStep + 1  -- Increment the step index  
    

--[[
    uLeft_now,uRight_now,uTorso_now, uLeft_next,uRight_next,uTorso_next, uSupport=
      moveleg.advance_step(uLeft_next,uRight_next,uTorso_next,iStep,velCurrent,true)
--]]

    --Instead of calculate next foothold based on velocity, pop them from queue
    mcm.set_status_uLeft(uLeft_next)
    mcm.set_status_uRight(uRight_next)
    mcm.set_status_uTorso(uTorso_next)

    uLeft_now,uRight_now,uTorso_now, uLeft_next,uRight_next,uTorso_next, uSupport, supportLeg=   
      moveleg.advance_step_from_queue(uLeft_next,uRight_next,uTorso_next,iStep)
    if not uLeft_now then return "done"  end

    -- Compute the ZMP coefficients for the next step
    zmp_solver:compute( uSupport, uTorso_now, uTorso_next )
    t_last_step = Body.get_time() -- Update t_last_step

    print( util.color('Walk velocity','blue'), string.format("%g, %g, %g",unpack(velCurrent)) )
  end

  local uTorso = zmp_solver:get_com(ph)
  uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

  local phSingle = moveleg.get_ph_single(ph,Config.walk.phSingle[1],Config.walk.phSingle[2])
  if iStep<=2 then phSingle = 0 end --This kills compensation and holds the foot on the ground  
  local uLeft, uRight, zLeft, zRight = uLeft_now, uRight_now, 0,0
  if supportLeg == 0 then  -- Left support    
    uRight,zRight = foot_traj_func(phSingle,uRight_now,uRight_next,stepHeight)    
  else    -- Right support    
    uLeft,zLeft = foot_traj_func(phSingle,uLeft_now,uLeft_next,stepHeight)    
  end

  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )
  delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,phSingle,gyro_rpy, angleShift)
  moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)

  -- Grab the sensor values
  local lfoot = Body.get_sensor_lfoot()
  local rfoot = Body.get_sensor_rfoot()
  -- ask for the foot sensor values
  Body.request_lfoot()
  Body.request_rfoot()
  if is_logging then
    -- write the log
    local log_entry = string.format('%f %f %d %s\n',t,ph,supportLeg,table.concat(lfoot,' '))
    LOG_F_SENSOR:write(log_entry)
  end

end -- walk.update

function walk.exit()

  print(walk._NAME..' Exit')
  -- TODO: Store things in shared memory?
  -- stop logging
  if is_logging then
    LOG_F_SENSOR:close()
  end
end

return walk
