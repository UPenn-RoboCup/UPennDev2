--Step controller based on preview ZMP walking


local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libZMP = require'libPreviewZMP'
local zmp_solver

local libStep = require'libStep'
local step_planner

require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

--Gait parameters
local tStep
local stepHeight  = Config.walk.stepHeight

local zLeft,zRight --Step landing heights
local zLeft0,zRight0
local aLeft,aRight = 0,0
-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local iStep

-- What foot trajectory are we using?
local foot_traj_func  
--foot_traj_func = moveleg.foot_trajectory_base
foot_traj_func = moveleg.foot_trajectory_square_stair
--foot_traj_func = moveleg.foot_trajectory_square_stair_touchdown

local touched =false


local t, t_discrete

local debugdata
local t0

local read_test = false
--local read_test = true
local debug_on = false

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

  --updae odometry variable
  wcm.set_robot_utorso1(uTorso_in)
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

  tStep = Config.walk.tStep

  -- Initiate the ZMP solver
  zmp_solver = libZMP.new_solver({
    ['tStep'] = Config.walk.tStep,
    ['tZMP']  = Config.walk.tZMP,    
  })
  zmp_solver:precompute()

  step_planner = libStep.new_planner()

  
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, zLeft, zRight =
      step_planner:init_stance()
  zLeft0,zRight0 = zLeft,zRight
  print(string.format("Current torso: %.2f %.2f",
    uTorso_now[1],uTorso_now[2]))

  zmp_solver:init_preview_queue(uLeft_now,uRight_now, uTorso_now, Body.get_time(), step_planner)
  
  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
  mcm.set_walk_ismoving(1) --We started moving
  --Reset odometry varialbe
  init_odometry(uTorso_now)  

  roll_max = 0
  touched = false

--print("Init Y:",uLeft_now[2],uTorso_now[2],uRight_now[2])


  --SHM BASED
  local nFootHolds = mcm.get_step_nfootholds()
  local footQueue = mcm.get_step_footholds()
  print("step #:",nFootHolds)

  for i=1,nFootHolds do
    local offset = (i-1)*15;
    local foot_movement = {footQueue[offset+1],footQueue[offset+2],footQueue[offset+3]}
    local supportLeg = footQueue[offset+4]
    local t0 = footQueue[offset+5]
    local t1 = footQueue[offset+6]
    local t2 = footQueue[offset+7]
    local zmp_mod = {footQueue[offset+8],footQueue[offset+9],footQueue[offset+10]}
    local footparam = {footQueue[offset+11],footQueue[offset+12],footQueue[offset+13]}    

    local zmp_mod2 = {footQueue[offset+14],footQueue[offset+15],0}

    step_planner:step_enque_trapezoid(foot_movement, supportLeg, t0,t1,t2,zmp_mod,footparam,zmp_mod2)
  end

  t = Body.get_time()
  time_discrete_shift = zmp_solver:trim_preview_queue(step_planner,t )  
  t_discrete = t 
  t0 = t

  debugdata=''
 
  hcm.set_motion_estop(0)
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  t_update = t   -- Save this at the last update time
  local discrete_updated = false
  local com_pos 


  if hcm.get_motion_estop()==1 then
    zmp_solver:emergency_stop(step_planner,t_discrete + time_discrete_shift)
  end


  while t_discrete<t do
    zmp_solver:update_preview_queue_steps(step_planner,t_discrete + time_discrete_shift)
    t_discrete = t_discrete + zmp_solver.preview_tStep
    discrete_updated = true

    --Get step information
    uLeft_now, uRight_now, uLeft_next, uRight_next,
      supportLeg, ph, ended, walkParam = zmp_solver:get_current_step_info(t_discrete + time_discrete_shift)

    --TODO: get step duration information!!! 
    if ended and zmp_solver:can_stop() then return "done"  end
      --Get the current COM position
    com_pos,zmp_pos = zmp_solver:update_state()
  end

  if discrete_updated then
    local uTorso = {com_pos[1],com_pos[2],0}
    uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

    --Calculate Leg poses 
    local phSingle = moveleg.get_ph_single(ph,Config.walk.phSingle[1],Config.walk.phSingle[2])
    local uLeft, uRight = uLeft_now, uRight_now
    local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
--    print("Z force:",l_ft[3],r_ft[3])    

    local zLeg = mcm.get_status_zLeg()
    local zLeg0 = mcm.get_status_zLeg0()
--    print('f:',l_ft[3],r_ft[3])

    --Correctly set initial foot Z height 
    if phSingle<1 and walkParam then
      if supportLeg == 0 then  -- Left support    
        walkParam[1] = zLeg0[2]
      else
        walkParam[1] = zLeg0[1]
      end
    end

    if supportLeg == 0 then  -- Left support    
      uRight,zRight,aRight,touched = foot_traj_func(
        phSingle,uRight_now,uRight_next,stepHeight,walkParam, zLeg[2],r_ft[3],touched)    
    elseif supportLeg==1 then    -- Right support    
      uLeft,zLeft,aLeft,touched = foot_traj_func(
        phSingle,uLeft_now,uLeft_next,stepHeight,walkParam, zLeg[1], l_ft[3],touched)    
    elseif supportLeg == 2 then --Double support
      aLeft,aRight = 0,0
      zLeft = zLeg[1]
      zRight = zLeg[2]
    end


    step_planner:save_stance(uLeft,uRight,uTorso,zLeft,zRight)  

    --Update the odometry variable
    update_odometry(uTorso)

    local uZMP = zmp_solver:get_zmp()
    mcm.set_status_uTorso(uTorso)
    mcm.set_status_uZMP(uZMP)
    mcm.set_status_t(t)


    --Calculate how close the ZMP is to each foot
    local uLeftSupport,uRightSupport = 
      step_planner.get_supports(uLeft,uRight)
    local dZmpL = math.sqrt((uZMP[1]-uLeftSupport[1])^2+(uZMP[2]-uLeftSupport[2])^2);
    local dZmpR = math.sqrt((uZMP[1]-uRightSupport[1])^2+(uZMP[2]-uRightSupport[2])^2);
    local supportRatio = dZmpL/(dZmpL+dZmpR);

  -- Grab gyro feedback for these joint angles
    local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )


    moveleg.ft_compensate(t_diff)

    delta_legs, angleShift = moveleg.get_leg_compensation_new(
      supportLeg,
      ph,
      gyro_rpy, 
      angleShift,
      supportRatio)

    --Move legs
    local uTorsoComp = mcm.get_stance_uTorsoComp()
    local uTorsoCompensated = util.pose_global({uTorsoComp[1],uTorsoComp[2],0},uTorso)
    moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight, zLeft,zRight,delta_legs)    

    --moveleg.set_leg_positions_ankletilt(uTorsoCompensated,uLeft,uRight, zLeft,zRight,delta_legs)    

    --store last step height
    if phSingle==1 then 
      mcm.set_status_zLeg0({zLeft,zRight}) 
      touched = false
    end

    local rpy = Body.get_rpy()
    local roll = rpy[1] * RAD_TO_DEG
    if math.abs(roll)>roll_max then roll_max = math.abs(roll)  end
   -- local roll_threshold = 5 --this is degree
    local roll_threshold = 555555 --this is degree
    if roll_max>roll_threshold and hcm.get_motion_estop()==0 then
      print("EMERGENCY STOPPING")
      print("IMU roll angle:",roll_max)
      hcm.set_motion_estop(1)
    end
  end
end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit')  
  mcm.set_walk_ismoving(0) --We stopped moving
end

return walk
