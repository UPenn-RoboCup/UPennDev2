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

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local iStep

-- What foot trajectory are we using?
--[[
local foot_traj_func  
--foot_traj_func = moveleg.foot_trajectory_base
--foot_traj_func = moveleg.foot_trajectory_square
foot_traj_func = moveleg.foot_trajectory_square_stair
--foot_traj_func = moveleg.foot_trajectory_square_stair_2
--]]
local foot_traj_func  
if Config.walk.foot_traj==1 then foot_traj_func = moveleg.foot_trajectory_base
else foot_traj_func = moveleg.foot_trajectory_square end

local crossing_num
local last_side = 1

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

  local odom_mode = wcm.get_robot_odom_mode();
  if odom_mode==0 then wcm.set_robot_pose(pose_odom)
  else wcm.set_robot_pose(wcm.get_slam_pose())
  end

  --updae odometry variable
  wcm.set_robot_utorso1(uTorso_in)
end

local function calculate_footsteps()
  local nFootHolds = 4
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=step_planner:init_stance()
  local supportLeg = 0

  local step_queue={}
  local tSlope1 = Config.walk.tStep*Config.walk.phSingle[1]
  local tSlope2 = Config.walk.tStep*(1-Config.walk.phSingle[2])

  step_queue[1] = {{0,0,0},2, 0.1,1,0.1,{0,0,0},{0,0,0}}
  local step_queue_count = 1;

  for i=1,nFootHolds  do
    step_planner.velCurrent = vector.new({0,0,0})

    local new_step
    supportLeg = 1-supportLeg
    step_queue_count = step_queue_count + 1
    initial_step = false
    last_step = false

    uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport =
      step_planner:get_next_step_velocity(uLeft_next,uRight_next,uTorso_next,supportLeg,initial_step,last_step)
    local leg_movemet
    if supportLeg==0 then --Left support
      leg_movement = util.pose_relative(uRight_next,uRight_now)  
    else
      leg_movement = util.pose_relative(uLeft_next,uLeft_now)  
    end
    new_step={leg_movement, 
              supportLeg, 
              tSlope1, 
              Config.walk.tStep-tSlope1-tSlope2,
              tSlope2,
              {0,0,0},
              {0,Config.walk.stepHeight,0}
             }
    
    step_queue[step_queue_count]=new_step
  end

--Write to the SHM
  local maxSteps = 40
  step_queue_vector = vector.zeros(12*maxSteps)
  for i=1,#step_queue do    
    local offset = (i-1)*13;
    step_queue_vector[offset+1] = step_queue[i][1][1]
    step_queue_vector[offset+2] = step_queue[i][1][2]
    step_queue_vector[offset+3] = step_queue[i][1][3]

    step_queue_vector[offset+4] = step_queue[i][2]

    step_queue_vector[offset+5] = step_queue[i][3]
    step_queue_vector[offset+6] = step_queue[i][4]    
    step_queue_vector[offset+7] = step_queue[i][5]    

    step_queue_vector[offset+8] = step_queue[i][6][1]
    step_queue_vector[offset+9] = step_queue[i][6][2]
    step_queue_vector[offset+10] = step_queue[i][6][3]

    step_queue_vector[offset+11] = step_queue[i][7][1]
    step_queue_vector[offset+12] = step_queue[i][7][2]
    step_queue_vector[offset+13] = step_queue[i][7][3]
  end

  mcm.set_step_footholds(step_queue_vector)
  mcm.set_step_nfootholds(#step_queue)
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

  zLeft, zRight = 0,0
  uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next=
      step_planner:init_stance()

  zmp_solver:init_preview_queue(uLeft_now,uRight_now, uTorso_now, Body.get_time(), step_planner)
  
  iStep = 1   -- Initialize the step index  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
  mcm.set_walk_ismoving(1) --We started moving
  --Reset odometry varialbe
  init_odometry(uTorso_now)  
  roll_max = 0

  crossing_num = 0
  last_side = 1

  --This is for the initial step only, so we can hardcode footholds here
  calculate_footsteps()

  --SHM BASED
  
  local nFootHolds = mcm.get_step_nfootholds()
  local footQueue = mcm.get_step_footholds()

  for i=1,nFootHolds do
    local offset = (i-1)*13;
    local foot_movement = {footQueue[offset+1],footQueue[offset+2],footQueue[offset+3]}
    local supportLeg = footQueue[offset+4]
    local t0 = footQueue[offset+5]
    local t1 = footQueue[offset+6]
    local t2 = footQueue[offset+7]
    local zmp_mod = {footQueue[offset+8],footQueue[offset+9],footQueue[offset+10]}
    local footparam = {footQueue[offset+11],footQueue[offset+12],footQueue[offset+13]}    
    step_planner:step_enque_trapezoid(foot_movement, supportLeg, t0,t1,t2,zmp_mod,footparam)
  end

  t = Body.get_time()
  time_discrete_shift = zmp_solver:trim_preview_queue(step_planner,t )  
  t_discrete = t 
  t0 = t

  debugdata=''
 
  hcm.set_motion_estop(0)
  mcm.set_motion_state(3)
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
    
    if supportLeg == 0 then  -- Left support    
      uRight,zRight = foot_traj_func(phSingle,uRight_now,uRight_next,stepHeight,walkParam)    
--      if walkParam then print(unpack(walkParam))end
    elseif supportLeg==1 then    -- Right support    
      uLeft,zLeft = foot_traj_func(phSingle,uLeft_now,uLeft_next,stepHeight,walkParam)    
--      if walkParam then print(unpack(walkParam))end
    elseif supportLeg == 2 then --Double support
    end
    step_planner:save_stance(uLeft,uRight,uTorso)  

    --Update the odometry variable
    update_odometry(uTorso)

    local uZMP = zmp_solver:get_zmp()
    mcm.set_status_uTorso(uTorso)
    mcm.set_status_uZMP(uZMP)
    mcm.set_status_t(t)


    --Calculate how close the ZMP is to each foot
    local uLeftSupport,uRightSupport = step_planner.get_supports(uLeft,uRight)
    local dZmpL = math.sqrt(
      (uZMP[1]-uLeftSupport[1])^2+
      (uZMP[2]-uLeftSupport[2])^2);

    local dZmpR = math.sqrt(
      (uZMP[1]-uRightSupport[1])^2+
      (uZMP[2]-uRightSupport[2])^2);

    local supportRatio = dZmpL/(dZmpL+dZmpR);

--print(unpack(uTorso),unpack(uLeft),unpack(uRight))

  -- Grab gyro feedback for these joint angles
    local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )

    --delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,ph,gyro_rpy, angleShift)

    delta_legs, angleShift = moveleg.get_leg_compensation_new(
      supportLeg,
      ph,
      gyro_rpy, 
      angleShift,
      supportRatio,
      t_diff)

    --Move legs
    local uTorsoComp = mcm.get_stance_uTorsoComp()
    local uTorsoCompensated = util.pose_global({uTorsoComp[1],uTorsoComp[2],0},uTorso)

    moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,  
      zLeft,zRight,delta_legs)    

    local rpy = Body.get_rpy()
    local roll = rpy[1] * RAD_TO_DEG
    
    if math.abs(roll)>roll_max then
      roll_max = math.abs(roll)
      --print("IMU roll angle:",roll_max)
    end

    local roll_threshold = 5 --this is degree

    if roll_max>roll_threshold and hcm.get_motion_estop()==0 then
      print("EMERGENCY STOPPING")
      print("IMU roll angle:",roll_max)
      hcm.set_motion_estop(1)
    end

    --Check if torso crossed the center position
    local relL = util.pose_relative(uLeft,uTorso)
    local relR = util.pose_relative(uRight,uTorso)
    local distL = relL[1]*relL[1]+relL[2]*relL[2]
    local distR = relR[1]*relR[1]+relR[2]*relR[2]
    local current_side
    if distL<distR then current_side = 1
    else current_side = 0 end

    if current_side~=last_side then
      crossing_num = crossing_num + 1
      last_side=current_side
      print("Crossing #:",crossing_num)
      if crossing_num==2 then        
        mcm.set_status_uTorsoVel({zmp_solver.x[2][1],zmp_solver.x[2][2],0})
        return "done" 
      end

    end

  end  
end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit')  
  wcm.set_robot_reset_pose(0)  --Start updating localization
end

return walk