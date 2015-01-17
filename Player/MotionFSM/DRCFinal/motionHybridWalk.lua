
local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libReactiveZMP = require'libReactiveZMPAdaptive'

local libStep = require'libStep'
local zmp_solver
local step_planner

require'mcm'
require'wcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local tStep = Config.walk.tStep
local stepHeight  = Config.walk.stepHeight

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local iStep,ph_last


local zmp_param_set = false

-- What foot trajectory are we using?
local foot_traj_func  
if Config.walk.foot_traj==1 then foot_traj_func = moveleg.foot_trajectory_base
else foot_traj_func = moveleg.foot_trajectory_square end

local init_odometry = function(uTorso)
  wcm.set_robot_utorso0(uTorso)
  wcm.set_robot_utorso1(uTorso)
end

local update_odometry = function(uTorso_in)
  local uTorso1 = wcm.get_robot_utorso1()

  --update odometry pose
  local odometry_step = util.pose_relative(uTorso_in,uTorso1)

  local pose_odom0 = wcm.get_robot_odometry()
  local pose_odom = util.pose_global(odometry_step, pose_odom0)
  wcm.set_robot_odometry(pose_odom)
  
  wcm.set_robot_utorso1(uTorso_in)--updae odometry variable


end

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
  init_odometry(uTorso_now)

  --Now we advance a step at next update  
  t_last_step = Body.get_time() - tStep 

  iStep = 1   -- Initialize the step index  
  ph_last = 0 
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
  mcm.set_walk_steprequest(0) --cancel walkkick request flag

  mcm.set_walk_ismoving(1) --We started moving
  
  --Check the initial torso velocity
  local uTorsoVel = util.pose_relative(mcm.get_status_uTorsoVel(), {0,0,uTorso_now[3]})
  local velTorso = math.sqrt(uTorsoVel[1]*uTorsoVel[1]+uTorsoVel[2]*uTorsoVel[2])
  if velTorso>0.01 then
    if uTorsoVel[2]>0 then iStep = 3; --Torso moving to left, skip initial step
    else iStep = 4; --Torso moving to the right, skip the initial step handling
    end
  end
  mcm.set_motion_state(4)
  zmp_param_set = false
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t   -- Save this at the last update time

  -- Grab the phase of the current step
  local ph,is_next_step = zmp_solver:get_ph(t,t_last_step,ph_last)
  if ph_last<0.5 and ph>=0.5 then
    local rpy = Body.get_rpy()
    local roll = rpy[1]
    local delay_threshold_angle = 2.5*math.pi/180


-------------------------------------------------------
-- Adaptive timing support

    if supportLeg==0 then
      print("Right landing, roll angle",roll*RAD_TO_DEG)
      if roll>delay_threshold_angle then      
        print("LANDING DELAYED!")
        zmp_solver:set_landing_delay_factor(1.7)
      elseif roll<-delay_threshold_angle then
        print("LANDING FASTENED")
        zmp_solver:set_landing_delay_factor(0.8)
      else
        zmp_solver:set_landing_delay_factor(1)
      end
    elseif supportLeg==1 then
      print("Left landing, roll angle",roll*RAD_TO_DEG)

      if roll<-delay_threshold_angle then      
        print("LANDING DELAYED!")
        zmp_solver:set_landing_delay_factor(1.7)
      elseif roll>delay_threshold_angle then
        print("LANDING FASTENED")
        zmp_solver:set_landing_delay_factor(0.8)
      else
        zmp_solver:set_landing_delay_factor(1)
      end
    else
      zmp_solver:set_landing_delay_factor(1)
    end
  end
---------------------------------------------------

  ph_last = ph
  
  if is_next_step then
    --Should we stop now?
    local stoprequest = mcm.get_walk_stoprequest()
    local steprequest = mcm.get_walk_steprequest()    
    if stoprequest>0 then return"done"   end
    if steprequest>0 then return "done_step" end
    
    iStep = iStep + 1  -- Increment the step index  
    supportLeg = iStep % 2 -- supportLeg: 0 for left support, 1 for right support   

    local initial_step = false
    if iStep<=3 then initial_step = true end

    --step_planner:update_velocity(hcm.get_motion_velocity())
    velCurrent = step_planner:update_velocity(mcm.get_walk_vel())
    --Calculate next step and torso positions based on velocity      
    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport =
      step_planner:get_next_step_velocity(uLeft_next,uRight_next,uTorso_next,supportLeg,initial_step)

    --Update walk coefficients
    zmp_solver:set_param()
    zmp_param_set = true
    -- Compute the ZMP coefficients for the next step
    zmp_solver:compute( uSupport, uTorso_now, uTorso_next )
    t_last_step = Body.get_time() -- Update t_last_step
  end
  
  local uTorso = zmp_solver:get_com(ph)
  uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

  local phSingle = moveleg.get_ph_single(ph,Config.walk.phSingle[1],Config.walk.phSingle[2])
  if iStep<=2 then phSingle = 0 end --This kills compensation and holds the foot on the ground  
  local uLeft, uRight, zLeft, zRight = uLeft_now, uRight_now, 0,0
  if supportLeg == 0 then uRight,zRight = foot_traj_func(phSingle,uRight_now,uRight_next,stepHeight) --LS
  else uLeft,zLeft = foot_traj_func(phSingle,uLeft_now,uLeft_next,stepHeight)    -- RS
  end

  local uZMP = zmp_solver:get_zmp(ph)
  moveleg.store_stance(t,ph,uLeft,uTorso,uRight,supportLeg,uZMP, zLeft,zRight)


-- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )

  --Calculate how close the ZMP is to each foot
  local uLeftSupport,uRightSupport = step_planner.get_supports(uLeft,uRight)
  local dZmpL = math.sqrt( (uZMP[1]-uLeftSupport[1])^2+ (uZMP[2]-uLeftSupport[2])^2)
  local dZmpR = math.sqrt((uZMP[1]-uRightSupport[1])^2+(uZMP[2]-uRightSupport[2])^2)
  local supportRatio = dZmpL/(dZmpL+dZmpR)

  delta_legs, angleShift = moveleg.get_leg_compensation_new(
      supportLeg,
      ph,
      gyro_rpy, 
      angleShift,
      supportRatio,
      t_diff)

  

  moveleg.set_leg_positions()       
  update_odometry(uTorso)--Update the odometry variable
end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit') 
  print("Total time: ",Body.get_time()-t_entry) 
  if zmp_param_set then
    local uTorsoVel = zmp_solver:get_com_vel(1)   --Get the final COM velocity
    mcm.set_status_uTorsoVel(uTorsoVel)
  else
    print("ZMP PARMAM NOT SET YET")
  end
  step_planner:save_stance(uLeft_next,uRight_next,uTorso_next)  
end

return walk
