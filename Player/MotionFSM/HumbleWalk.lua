--New, cleaned-up humblewalk 
--Got rid of any non-used codes (including walkkick)
--Arm movement is turned off (only handled by arm FSM)

local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libReactiveZMP = require'libReactiveZMP'
local zmp_solver

local libStep = require'libStep'
local step_planner

require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local tStep = Config.walk.tStep
local stepHeight  = Config.walk.stepHeight

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
 
  mcm.set_walk_vel({0,0,0})--reset target speed

  -- Initiate the ZMP solver
  zmp_solver = libReactiveZMP.new_solver({
    ['tStep'] = Config.walk.tStep,
    ['tZMP']  = Config.walk.tZMP,
    ['start_phase']  = Config.walk.phSingle[1],
    ['finish_phase'] = Config.walk.phSingle[2],
  })

  step_planner = libStep.new_planner()
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()

  --Reset odometry varialbe
  Body.init_odometry(uTorso_now)

  --Now we advance a step at next update  
  t_last_step = Body.get_time() - tStep 

  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
  mcm.set_walk_ismoving(1) --We started moving
  -- log file
  if is_logging then
    LOG_F_SENSOR = io.open('feet.log','w')
    local log_entry = string.format('t ph supportLeg values\n')
    LOG_F_SENSOR:write(log_entry)
  end

  --Check the initial torso velocity
  local uTorsoVel = mcm.get_status_uTorsoVel()
  local velTorso = math.sqrt(uTorsoVel[1]*uTorsoVel[1]+
                          uTorsoVel[2]*uTorsoVel[2])

  if velTorso>0.01 then
    if uTorsoVel[2]>0 then --Torso moving to left
      iStep = 3; --This skips the initial step handling
    else
      iStep = 4; --This skips the initial step handling
    end
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
    supportLeg = iStep % 2 -- supportLeg: 0 for left support, 1 for right support

    local steprequest = mcm.get_walk_steprequest()
    local step_supportLeg = mcm.get_walk_step_supportleg()
    if steprequest>0 and supportLeg ==step_supportLeg then
      return "done_step"
    end

    local initial_step = false
    if iStep<=3 then initial_step = true end

    --step_planner:update_velocity(hcm.get_motion_velocity())
    velCurrent = step_planner:update_velocity(mcm.get_walk_vel())
    --Calculate next step and torso positions based on velocity      
    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport =
      step_planner:get_next_step_velocity(uLeft_next,uRight_next,uTorso_next,supportLeg,initial_step)


    -- Compute the ZMP coefficients for the next step
    zmp_solver:compute( uSupport, uTorso_now, uTorso_next )
    t_last_step = Body.get_time() -- Update t_last_step

    print( util.color('Walk velocity','blue'), string.format("%g, %g, %g",unpack(step_planner.velCurrent)) )
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

  --Robotis-style simple feedback (which seems to be better :[])
--  delta_legs, angleShift = moveleg.get_leg_compensation_simple(supportLeg,phSingle,gyro_rpy, angleShift)

--Old feedback with different compensation
  delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,ph,gyro_rpy, angleShift)

  moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)

--For external monitoring
  local uZMP = zmp_solver:get_zmp(ph)
  mcm.set_status_uTorso(uTorso)
  mcm.set_status_uZMP(uZMP)
  mcm.set_status_t(t)

  if is_logging then
-- Grab the sensor values
    local lfoot = Body.get_sensor_lfoot()
    local rfoot = Body.get_sensor_rfoot()
    -- ask for the foot sensor values
  
    Body.request_lfoot()
    Body.request_rfoot()
    -- write the log
    local log_entry = string.format('%f %f %d %s\n',t,ph,supportLeg,table.concat(lfoot,' '))
    LOG_F_SENSOR:write(log_entry)
  end

  --Update the odometry variable
  Body.update_odometry(uTorso)
  --print("odometry pose:",unpack(wcm.get_robot_pose_odom()))
end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit')

  --Get the final COM velocity
  local uTorsoVel = zmp_solver:get_com_vel(1)
  print("Torso vel:",uTorsoVel[1],uTorsoVel[2])

  step_planner:save_stance(uLeft_next,uRight_next,uTorso_next)
  mcm.set_status_uTorsoVel(uTorsoVel)

  -- stop logging
  if is_logging then
    LOG_F_SENSOR:close()
  end
end

return walk
